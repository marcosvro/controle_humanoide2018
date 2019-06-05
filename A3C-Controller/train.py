from environment import VrepEnvironment
import threading
import numpy as np
import os
import shutil
import matplotlib.pyplot as plt
import tensorflow as tf
from A3C import ACNet
from worker import Worker
from parameters import *



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