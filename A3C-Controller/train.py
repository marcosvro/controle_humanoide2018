# coding: utf-8
from environment import VrepEnvironment
import threading
import multiprocessing
import numpy as np
import os
import shutil
import matplotlib.pyplot as plt
import tensorflow as tf
from A3C import ACNet
#from worker import Worker
from parameters import *


# worker class that inits own environment, trains on it and updloads weights to global net
class Worker(object):
    def __init__(self, name, globalAC, sess):
        self.env = VrepEnvironment(name)   # make environment for each worker
        self.name = name
        self.AC = ACNet(name, sess, globalAC) # create ACNet for each worker
        self.sess=sess
   
    def work(self):
        global global_rewards, global_episodes
        total_step = 1
        #self.AC.pull_global()                            #TESTAR COM ESSA LINHA
        buffer_s, buffer_a, buffer_r = [], [], []

        while global_episodes < MAX_GLOBAL_EP:
            s = self.env.reset()
            ep_r = 0
            for ep_t in range(MAX_EP_STEP):
                # show first worker simulation
                '''
                if self.name == 'W_0' and RENDER:
                    self.env.render()
                '''
                a = self.AC.choose_action(s)         # estimate stochastic action based on policy 
                s_, r, done, info = self.env.step(a) # make step in environment
                
                #done = True if ep_t == MAX_EP_STEP - 1 else False

                ep_r += r
                # save actions, states and rewards in buffer
                buffer_s.append(s)
                buffer_a.append(a)
                buffer_r.append((r+8)/8)    # normalize reward

                if total_step % UPDATE_GLOBAL_ITER == 0 or done:   # update global and assign to local net
                    if done:
                        v_s_ = 0   # terminal
                    else:
                        v_s_ = self.sess.run(self.AC.v, {self.AC.s: s_[np.newaxis, :]})[0, 0]
                    buffer_v_target = []
                    for r in buffer_r[::-1]:    # reverse buffer r
                        v_s_ = r + GAMMA * v_s_
                        buffer_v_target.append(v_s_)
                    buffer_v_target.reverse()

                    buffer_s, buffer_a, buffer_v_target = np.vstack(buffer_s), np.vstack(buffer_a), np.vstack(buffer_v_target)
                    feed_dict = {
                        self.AC.s: buffer_s,
                        self.AC.a_his: buffer_a,
                        self.AC.v_target: buffer_v_target,
                    }
                    self.AC.update_global(feed_dict) # actual training step, update global ACNet
                    buffer_s, buffer_a, buffer_r = [], [], []
                    self.AC.pull_global() # get global parameters to local ACNet

                s = s_
                total_step += 1
                if done:
                    if len(global_rewards) < 5:  # record running episode reward
                        global_rewards.append(ep_r)
                    else:
                        global_rewards.append(ep_r)
                        global_rewards[-1] =(np.mean(global_rewards[-5:])) # smoothing 
                    print(
                        self.name,
                        "Ep:", global_episodes,
                        "| Ep_r: %i" % global_rewards[-1],
                          )
                    global_episodes += 1
                    break


if __name__ == "__main__":
    global_rewards = []
    global_episodes = 0

    sess = tf.Session()

    with tf.device("/cpu:0"):
        global_ac = ACNet(GLOBAL_NET_SCOPE,sess)  # we only need its params
        workers = []
        # Create workers
        for i in range(N_WORKERS):
            i_name = 'W_%i' % i   # worker name
            workers.append(Worker(i_name, global_ac,sess))

    coord = tf.train.Coordinator()
    sess.run(tf.global_variables_initializer())

    if OUTPUT_GRAPH: # write log file
        if os.path.exists(LOG_DIR):
            shutil.rmtree(LOG_DIR)
        tf.summary.FileWriter(LOG_DIR, sess.graph)

    worker_threads = []
    for worker in workers: #start workers
        job = lambda: worker.work()
        t = threading.Thread(target=job)
        t.start()
        worker_threads.append(t)
    coord.join(worker_threads)  # wait for termination of workers
    
    plt.plot(np.arange(len(global_rewards)), global_rewards) # plot rewards
    plt.xlabel('step')
    plt.ylabel('total moving reward')
    plt.show()
