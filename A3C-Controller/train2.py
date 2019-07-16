import torch
import torch.nn as nn
from utils import v_wrap, set_init, push_and_pull, record
import torch.nn.functional as F
import torch.multiprocessing as mp
import threading as mt
from multiprocessing import Queue
from shared_adam import SharedAdam
from shared_rmsprop import SharedRMSProp
from environment import VrepEnvironment
import math, os
from parameters import *
import rospy
from std_msgs.msg import Float32MultiArray, Bool
from argparse import ArgumentParser
os.environ["OMP_NUM_THREADS"] = "1"

def pub_worker_state(pub, w_states):
    rate = rospy.Rate(5) # 10hz
    global ended
    while not rospy.is_shutdown() and not ended:
        debug = [0]*N_WORKERS
        for i in range(N_WORKERS):
            debug[i] = w_states[i].value
        mat = Float32MultiArray()
        mat.data = debug
        pub.publish(mat)
        rate.sleep()

def finish_train(msg):
    if not msg.data:
        return
    global runing
    runing = False

class Net(nn.Module):
    def __init__(self, s_dim, a_dim):
        super(Net, self).__init__()
        self.s_dim = s_dim
        self.a_dim = a_dim
        self.a1 = nn.Linear(s_dim, 256)
        self.a2 = nn.Linear(256, 256)
        self.mu = nn.Linear(256, a_dim)
        self.sigma = nn.Linear(256, a_dim)
        self.c1 = nn.Linear(s_dim, 256)
        self.c2 = nn.Linear(256, 256)
        self.v = nn.Linear(256, 1)
        set_init([self.a1, self.a2, self.mu, self.sigma, self.c1, self.c2, self.v])
        self.distribution = torch.distributions.Normal

    def forward(self, x):
        a1 = F.relu6(self.a1(x))
        a2 = F.relu6(self.a2(a1))
        mu = F.tanh(self.mu(a2))
        sigma = F.softplus(self.sigma(a2)) + 0.001      # avoid 0
        c1 = F.relu6(self.c1(x))
        c2 = F.relu6(self.c2(c1))
        values = self.v(c2)
        return mu, sigma, values

    def choose_action(self, s):
        self.training = False
        mu, sigma, _ = self.forward(s)
        m = self.distribution(mu.view(N_A, ).data, sigma.view(N_A, ).data)
        return m.sample().numpy()

    def loss_func(self, s, a, v_t):
        self.train()
        mu, sigma, values = self.forward(s)
        td = v_t - values
        c_loss = td.pow(2)

        m = self.distribution(mu, sigma)
        log_prob = m.log_prob(a)
        entropy = 0.5 + 0.5 * math.log(2 * math.pi) + torch.log(m.scale)  # exploration
        exp_v = log_prob * td.detach() + 0.005 * entropy
        a_loss = -exp_v
        total_loss = (a_loss + c_loss).mean()
        return total_loss


class Worker(mp.Process):
    def __init__(self, gnet, opt, global_ep, global_ep_r, res_queue, best_ep_r, idx, pub_queue, t_ori, t_acc, t_pos, t_joint, w_state):
        super(Worker, self).__init__()
        self.w_state = w_state
        self.w_state.value = 1
        self.name = 'w%i' % idx
        self.g_ep, self.g_ep_r, self.res_queue, self.best_ep_r = global_ep, global_ep_r, res_queue, best_ep_r
        self.gnet, self.opt = gnet, opt
        self.lnet = Net(N_S, N_A)           # local network
        self.env = VrepEnvironment(idx, pub_queue, t_ori, t_acc, t_pos, t_joint)
        self.w_state = w_state
        self.pub_queue = pub_queue
        self.exit = mp.Event()

    def run(self):
        total_step = 1
        while self.g_ep.value < MAX_EP and not self.exit.is_set():
            self.w_state.value = 2
            s = self.env.reset()
            buffer_s, buffer_a, buffer_r = [], [], []
            ep_r = 0.
            for t in range(MAX_EP_STEP):
                #if self.name == 'w0':
                #    self.env.render()
                self.w_state.value = 3
                a = self.lnet.choose_action(v_wrap(s[None, :]))
                s_, r, done, info = self.env.step(a.clip(-1, 1))
                #print(info['progress'])
                self.w_state.value = 4

                if math.isnan(r):
                    r = 0
                if done:
                    r = 0

                ep_r += r
                buffer_a.append(a)
                buffer_s.append(s)
                buffer_r.append(r)    # normalize

                if total_step%UPDATE_GLOBAL_ITER == 0 or done:  # update global and assign to local net
                    self.w_state.value = 5

                    # sync
                    push_and_pull(self.opt, self.lnet, self.gnet, done, s_, buffer_s, buffer_a, buffer_r, GAMMA)
                    buffer_s, buffer_a, buffer_r = [], [], []

                    # print information
                    record(self.g_ep, self.g_ep_r, ep_r, self.res_queue, self.best_ep_r, self.name, self.lnet.state_dict())
                    if done:  # done
                        break
                s = s_
                total_step += 1
        self.pub_queue.put(None)
        self.res_queue.put(None)

    def shutdown(self):
        print ("Shutdown %s initiated" %(self.name))
        self.exit.set()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-l", "--load",
                    action="store_true", dest="load_weight", default=False,
                    help="Load model weights from path in paramters.py")
    args = parser.parse_args()

    gnet = Net(N_S, N_A)        # global network
    if args.load_weight:
        print("Carregando pesos..")
        gnet.load_state_dict(torch.load(LOG_DIR))
        gnet.eval()
        print("Pesos carregados.")
    gnet.share_memory()         # share the global parameters in multiprocessing
    opt = SharedRMSProp(gnet.parameters(), lr=0.0001)  # global optimizer
    global_ep, global_ep_r, res_queue, pub_queue, best_ep_r = mp.Value('i', 0), mp.Value('d', 0.), mp.Queue(), mp.Queue(), mp.Value('d', 0.)
    ended = False

    # create publishers
    rospy.init_node('controller_A3C')
    pubs = []
    states = []
    w_states = []
    for i in range(N_WORKERS):
        name = 'w%i' % i
        reset_pub = rospy.Publisher(name+'/reset', Bool, queue_size=1) # define publisher para resetar simulação
        pos_pub = rospy.Publisher(name+'/joint_pos', Float32MultiArray, queue_size=1) #define publisher para as posições
        t_ori_last = mp.Array('d', [0]*3)
        t_acc_last = mp.Array('d', [0]*3)
        t_pos_last = mp.Array('d', [0]*2)
        t_joint_last = mp.Array('d', [0]*12)
        w_state = mp.Value('i', 0)
        states.append([t_ori_last, t_acc_last, t_pos_last, t_joint_last])
        pubs.append([pos_pub, reset_pub])
        w_states.append(w_state)

    # create worker state logger
    w_state_pub = rospy.Publisher('worker_state_logger', Float32MultiArray, queue_size=1)
    t = mt.Thread(target=pub_worker_state, args=(w_state_pub, w_states,))
    t.start()

    # end comand
    rospy.Subscriber("/finish_train", Bool, finish_train)

    # parallel training
    workers = [Worker(gnet, opt, global_ep, global_ep_r, res_queue, best_ep_r, i, pub_queue, states[i][0], states[i][1], states[i][2], states[i][3], w_states[i]) for i in range(N_WORKERS)]
    [w.start() for w in workers]
    res = []                    # record episode reward to plot

    runing = True
    while runing:
        try:
            msg = pub_queue.get()
            '''
            debug = [0]*N_WORKERS
            for i in range(N_WORKERS):
                debug[i] = w_states[i].value
            print (debug)
            '''
            if msg is not None:
                trueReset_falseJointPos = msg[0]
                i = msg[1]
                if (trueReset_falseJointPos): # reset msg
                    to_send = Bool()
                    to_send.data = msg[2]
                    pubs[i][1].publish(to_send)
                else: # joint position msg
                    to_send = Float32MultiArray()
                    to_send.data = msg[2]
                    pubs[i][0].publish(to_send)
            else:
                print("Acabou")
                break
        except Exception as identifier:
            break

    [w.shutdown() for w in workers]
    [w.join() for w in workers]
    ended = True
    while res_queue.qsize() != 0:
        msg = res_queue.get()
        if msg is not None:
            res.append(msg)
        else:
            break

    import matplotlib.pyplot as plt
    plt.plot(res)
    plt.ylabel('Moving average ep reward')
    plt.xlabel('Step')
    #plt.show()
    plt.savefig('log/reward_graph.png')
