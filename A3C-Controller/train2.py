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
import time
from tensorboardX import SummaryWriter
os.environ["OMP_NUM_THREADS"] = "1"

def pub_worker_state(pub, w_states):
    rate = rospy.Rate(5) # 10hz
    global runing
    while not rospy.is_shutdown() and runing:
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
    global stat_queue
    runing = False
    stat_queue.put(None)


def get_pub_msgs ():
    global runing
    global pub_queue
    global pubs

    while runing:
        try:
            msg = pub_queue.get()
            
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
        except Exception:
            break


class Net(nn.Module):
    def __init__(self, s_dim, a_dim):
        super(Net, self).__init__()
        self.s_dim = s_dim
        self.a_dim = a_dim
        self.a1 = nn.Linear(s_dim, 128)
        self.a2 = nn.Linear(128, 128)
        self.mu = nn.Linear(128, a_dim)
        self.sigma = nn.Linear(128, a_dim)
        self.c1 = nn.Linear(s_dim, 128)
        self.c2 = nn.Linear(128, 128)
        self.v = nn.Linear(128, 1)
        set_init([self.a1, self.a2, self.mu, self.sigma, self.c1, self.c2, self.v])
        self.distribution = torch.distributions.Normal

    def forward(self, x):
        a1 = F.relu6(self.a1(x))
        a2 = F.relu6(self.a2(a1))
        mu = 2 * F.tanh(self.mu(a2))
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
        return total_loss, c_loss.detach().mean(), a_loss.detach().mean(), td.detach().mean()


class Worker(mp.Process):
    def __init__(self, gnet, opt, global_ep, global_ep_r, stat_queue, best_ep_r, idx, pub_queue, t_ori, t_acc, t_pos, t_joint, t_force, t_pos_feet, w_state):
        super(Worker, self).__init__()
        self.w_state = w_state
        self.w_state.value = 1
        self.name = 'w%i' % idx
        self.g_ep, self.g_ep_r, self.stat_q, self.best_ep_r = global_ep, global_ep_r, stat_queue, best_ep_r
        self.gnet, self.opt = gnet, opt
        self.lnet = Net(N_S, N_A)           # local network
        self.lnet.load_state_dict(gnet.state_dict())
        self.env = VrepEnvironment(idx, pub_queue, t_ori, t_acc, t_pos, t_joint, t_force, t_pos_feet)
        self.w_state = w_state
        self.pub_queue = pub_queue
        self.exit = mp.Event()

    def run(self):
        #total_step = 1
        while self.g_ep.value < MAX_EP and not self.exit.is_set():
            total_step = 1
            self.w_state.value = 2
            s = self.env.reset()

            buffer_s, buffer_a, buffer_r = [], [], []

            num_updates = 0
            loss_ep_total = 0.
            ep_r = 0.
            time_ep_total = 0.
            c_ep_loss_total = 0.
            a_ep_loss_total = 0.
            adv_ep_loss_total = 0.

            for t in range(MAX_EP_STEP):
                if self.exit.is_set():
                    return
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
                time_ep_total += TIME_STEP_ACTION

                buffer_a.append(a)
                buffer_s.append(s)
                buffer_r.append(r)    # normalize

                if total_step%UPDATE_GLOBAL_ITER == 0 or done:  # update global and assign to local net
                    self.w_state.value = 5

                    # sync
                    loss, c_loss, a_loss, adv = push_and_pull(self.opt, self.lnet, self.gnet, done, s_, buffer_s, buffer_a, buffer_r, GAMMA)
                    buffer_s, buffer_a, buffer_r = [], [], []

                    num_updates += 1
                    loss_ep_total += loss
                    c_ep_loss_total += c_loss
                    a_ep_loss_total += a_loss
                    adv_ep_loss_total += adv

                    # print information
                    #record(self.g_ep, self.g_ep_r, ep_r, self.best_ep_r, self.name, self.lnet.state_dict())
                    if done:  # done
                        stats = {
                            'TimeEpisodeDuration' : time_ep_total,
                            'reward' : ep_r,
                            'GradientUpdates' : num_updates,
                            'MeanLoss' : loss_ep_total/num_updates,
                            'MeanValueLoss' : c_ep_loss_total/num_updates,
                            'MeanPolicyLoss' : a_ep_loss_total/num_updates,
                            'MeanAdvantage' :adv_ep_loss_total /num_updates 
                        }
                        ep_num = record(self.g_ep, self.g_ep_r, stats, self.best_ep_r, self.name, self.lnet.state_dict())
                        stats['Episode'] = ep_num
                        self.stat_q.put(stats)
                        break
                s = s_
                total_step += 1

    def shutdown(self):
        print ("Shutdown %s initiated" %(self.name))
        self.exit.set()
        self.env.close()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-l", "--load",
                    type=float, dest="load_weight", default=0.,
                    help="Load model weights from path (in paramters.py) with best_global_reward defined by value entered.")
    parser.add_argument('-n','--name', type=str, default='unnamed', help='Execution name (optional)')
    args = parser.parse_args()

    gnet = Net(N_S, N_A)        # global network
    gnet.share_memory()         # share the global parameters in multiprocessing
    opt = SharedAdam(gnet.parameters(), lr=0.0001)  # global optimizer
    global_ep, global_ep_r, stat_queue, pub_queue, best_ep_r = mp.Value('i', 0), mp.Value('d', 0.), mp.Queue(), mp.Queue(), mp.Value('d', 0.)
    runing = True

    # load weight
    if args.load_weight:
        print("Carregando pesos..")
        gnet.load_state_dict(torch.load(LOG_DIR))
        gnet.eval()
        best_ep_r.value = args.load_weight
        print("Pesos carregados.")

    #inicia writeer tensorboard
    stat_writer = SummaryWriter(comment="-A3C- "+args.name)

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
        t_force_last = mp.Array('d', [0]*8)
        t_pos_feet_last = mp.Array('d', [0]*4)
        w_state = mp.Value('i', 0)
        states.append([t_ori_last, t_acc_last, t_pos_last, t_joint_last, t_force_last, t_pos_feet_last])
        pubs.append([pos_pub, reset_pub])
        w_states.append(w_state)


    # create thread to get stats from workers
    t = mt.Thread(target=get_pub_msgs)
    t.daemon = True
    t.start()

    # create worker state logger
    w_state_pub = rospy.Publisher('worker_state_logger', Float32MultiArray, queue_size=1)
    t = mt.Thread(target=pub_worker_state, args=(w_state_pub, w_states,))
    t.daemon = True
    t.start()

    # end comand
    rospy.Subscriber("/finish_train", Bool, finish_train)

    # parallel training
    workers = [Worker(gnet, opt, global_ep, global_ep_r, stat_queue, best_ep_r, i, pub_queue, states[i][0], states[i][1], states[i][2], states[i][3], states[i][4], states[i][5], w_states[i]) for i in range(N_WORKERS)]
    for w in workers:
        w.daemon = True
    [w.start() for w in workers]
    #res = []                    # record episode reward to plot


    # get stats and in log tensorboard
    while runing:
        try:
            msg = stat_queue.get()
            if msg is not None:
                episode = msg['Episode']
                if msg['TimeEpisodeDuration']:
                    stat_writer.add_scalar("TimeEpisodeDuration/Episode", msg['TimeEpisodeDuration'], episode)
                if msg['reward']:
                    stat_writer.add_scalar("Reward/Episode", msg['reward'], episode)
                if msg["MeanLoss"]:
                    stat_writer.add_scalar("MeanLoss/Episode", msg['MeanLoss'], episode)
                if msg['GradientUpdates']:
                    stat_writer.add_scalar("GradientUpdates/Episode", msg['GradientUpdates'], episode)
                if msg['MeanValueLoss']:
                    stat_writer.add_scalar('MeanValueLoss/Episode', msg['MeanValueLoss'], episode)
                if msg['MeanPolicyLoss']:
                    stat_writer.add_scalar('MeanPolicyLoss/Episode', msg['MeanPolicyLoss'], episode)
                if msg['MeanAdvantage']:
                    stat_writer.add_scalar('MeanAdvantage/Episode', msg['MeanAdvantage'], episode)
        except Exception:
            pass

    #size_res_queue = res_queue.qsize()
    [w.shutdown() for w in workers]

    #save and close tensorboard stats
    print("Salvando estatísticas no tensorboard!")
    stat_writer.close()
    print("Saindo..")

    [w.join() for w in workers]