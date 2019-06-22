"""
Reinforcement Learning (A3C) using Pytroch + multiprocessing.
The most simple implementation for continuous action.

View more on my Chinese tutorial page [莫烦Python](https://morvanzhou.github.io/).
"""

import torch
import torch.nn as nn
from utils import v_wrap, set_init, push_and_pull, record
import torch.nn.functional as F
import torch.multiprocessing as mp
from multiprocessing import Queue
from shared_adam import SharedAdam
from environment import VrepEnvironment
import math, os
from parameters import *
import rospy
from std_msgs.msg import Float32MultiArray, Bool
os.environ["OMP_NUM_THREADS"] = "1"


class Net(nn.Module):
    def __init__(self, s_dim, a_dim):
        super(Net, self).__init__()
        self.s_dim = s_dim
        self.a_dim = a_dim
        self.a1 = nn.Linear(s_dim, 200)
        self.mu = nn.Linear(200, a_dim)
        self.sigma = nn.Linear(200, a_dim)
        self.c1 = nn.Linear(s_dim, 100)
        self.v = nn.Linear(100, 1)
        set_init([self.a1, self.mu, self.sigma, self.c1, self.v])
        self.distribution = torch.distributions.Normal

    def forward(self, x):
        a1 = F.relu6(self.a1(x))
        mu = 2 * F.tanh(self.mu(a1))
        sigma = F.softplus(self.sigma(a1)) + 0.001      # avoid 0
        c1 = F.relu6(self.c1(x))
        values = self.v(c1)
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
    def __init__(self, gnet, opt, global_ep, global_ep_r, res_queue, best_ep_r, idx, pub_queue, t_ori, t_acc, t_pos):
        super(Worker, self).__init__()
        self.name = 'w%i' % idx
        self.g_ep, self.g_ep_r, self.res_queue, self.best_ep_r = global_ep, global_ep_r, res_queue, best_ep_r
        self.gnet, self.opt = gnet, opt
        self.lnet = Net(N_S, N_A)           # local network
        self.env = VrepEnvironment(idx, pub_queue, t_ori, t_acc, t_pos)

    def run(self):
        total_step = 1
        while self.g_ep.value < MAX_EP:
            s = self.env.reset()
            buffer_s, buffer_a, buffer_r = [], [], []
            ep_r = 0.
            for t in range(MAX_EP_STEP):
                #if self.name == 'w0':
                #    self.env.render()
                a = self.lnet.choose_action(v_wrap(s[None, :]))
                s_, r, done, _ = self.env.step(a.clip(-1, 1))
                
                if done:
                    r = -1

                ep_r += r
                buffer_a.append(a)
                buffer_s.append(s)
                buffer_r.append(r)    # normalize

                if total_step % UPDATE_GLOBAL_ITER == 0 or done:  # update global and assign to local net
                    # sync
                    push_and_pull(self.opt, self.lnet, self.gnet, done, s_, buffer_s, buffer_a, buffer_r, GAMMA)
                    buffer_s, buffer_a, buffer_r = [], [], []

                    if done:  # done and print information
                        record(self.g_ep, self.g_ep_r, ep_r, self.res_queue, self.best_ep_r, self.name, self.lnet.state_dict())
                        break
                s = s_
                total_step += 1

        self.res_queue.put(None)


if __name__ == "__main__":
    gnet = Net(N_S, N_A)        # global network
    gnet.share_memory()         # share the global parameters in multiprocessing
    opt = SharedAdam(gnet.parameters(), lr=0.0001)  # global optimizer
    global_ep, global_ep_r, res_queue, pub_queue, best_ep_r = mp.Value('i', 0), mp.Value('d', 0.), mp.Queue(), mp.Queue(), mp.Value('d', 0.)

    # create publishers
    
    rospy.init_node('controller_A3C')
    pubs = []
    states = []
    for i in range(N_WORKERS):
        name = 'w%i' % i
        reset_pub = rospy.Publisher(name+'/reset', Bool, queue_size=1) # define publisher para resetar simulação
        pos_pub = rospy.Publisher(name+'/joint_pos', Float32MultiArray, queue_size=1) #define publisher para as posições
        t_ori_last = mp.Array('d', [0]*3)
        t_acc_last = mp.Array('d', [0]*3)
        t_pos_last = mp.Array('d', [0]*2)
        states.append([t_ori_last, t_acc_last, t_pos_last])
        pubs.append([pos_pub, reset_pub])
    

    # parallel training
    workers = [Worker(gnet, opt, global_ep, global_ep_r, res_queue, best_ep_r, i, pub_queue, states[i][0], states[i][1], states[i][2]) for i in range(N_WORKERS)]
    [w.start() for w in workers]
    res = []                    # record episode reward to plot
    while True:
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
        else:
            print("Acabou")
            break
    [w.join() for w in workers]

    import matplotlib.pyplot as plt
    plt.plot(res)
    plt.ylabel('Moving average ep reward')
    plt.xlabel('Step')
    plt.show()
