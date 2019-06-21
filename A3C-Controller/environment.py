# coding: utf-8
import time
from parameters import *
from controlador import Controlador
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Bool
import os

class VrepEnvironment():
	def __init__ (self, idx, pub_queue, t_ori, t_acc, t_pos):
		#incialize vrep simulation and wait for confirmation
		simu_name_id = 'w%i' % idx

		if TESTING:
			os.system(VREP_PATH+"/vrep.sh -s -q -g"+simu_name_id+" "+SCENE_FILE_PATH+"&")
		else:
			os.system('xvfb-run --auto-servernum --server-num=1 -s "-screen 0 640x480x24" '+VREP_PATH+"/vrep.sh -h -q -s -g"+simu_name_id+" "+SCENE_FILE_PATH+"&")
		self.w_id = idx

		time.sleep(TIME_WAIT_INIT_PUBS)
		self.pub_queue = pub_queue
		'''
		rospy.init_node('controller_A3C'+simu_name_id, anonymous=True)
		self.reset_pub = rospy.Publisher(simu_name_id+'/reset', Bool, queue_size=1) # define publisher para resetar simulação
		self.pos_pub = rospy.Publisher(simu_name_id+'/joint_pos', Float32MultiArray, queue_size=1) #define publisher para as posições
		'''
		pub_rate = rospy.Rate(N_PUBS_STEP/TIME_STEP_ACTION)

		self.ack = False
		self.cmd = False

		self.sub_controller = Controlador(idx, pub_queue, pub_rate, t_ori, t_acc, t_pos, gravity_compensation_enable=True)
		rospy.Subscriber("/vrep_ros_interface/"+simu_name_id+'/ack', Bool, self.ack_callback)
		print("Ambiente inicializado!!")

	def reset(self):
		#reset robot on environment
		self.cmd =  not self.cmd
		#self.reset_pub.publish(Bool(self.cmd))
		self.pub_queue.put([True, self.w_id, self.cmd])
		self.wait_ack() #wait for sim confirmation
		init_state, d, r  = self.sub_controller.reset()
		return init_state

	def step(self, action):
		info = {}
		s, done, r = self.sub_controller.step(action, self.cmd)
		return s, r, done, info

	def ack_callback(self, msg):
		self.ack = True

	def wait_ack(self):
		#wait for vrep reset simulation
		self.ack = False
		timer = 0
		while (not self.ack and timer < TIME_WAIT_ACK_MAX):
			time.sleep(TIME_WAIT_ACK)
			timer += TIME_WAIT_ACK


if __name__ == "__main__":
	env = VrepEnvironment("test")
	s = env.reset()
	while 1:
		a = [0]*6
		env.step(a)
