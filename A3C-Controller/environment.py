# coding: utf-8
import time
from parameters import *
from controlador import Controlador
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Bool
import os
import vrep

class VrepEnvironment():
	def __init__ (self, idx, pub_queue, t_ori, t_acc, t_pos, t_joint, t_force, t_pos_feet):
		#incialize vrep simulation and wait for confirmation
		simu_name_id = 'w%i' % idx

		if idx == 17:
			porta = 19980+21
		else:
			porta = 19980+idx

		if TESTING:
			os.system(VREP_PATH+"/vrep.sh -q -g"+simu_name_id+" -gREMOTEAPISERVERSERVICE_"+str(porta)+"_FALSE_FALSE "+SCENE_FILE_PATH+"&")
			time.sleep(15)
			#print("Devia estar iniciando agora!!")
		else:
			os.system('DISPLAY=:0 '+VREP_PATH+"/vrep.sh -q -g"+simu_name_id+" -gREMOTEAPISERVERSERVICE_"+str(porta)+"_FALSE_FALSE "+SCENE_FILE_PATH+"&")
		self.w_id = idx

		time.sleep(TIME_WAIT_INIT_PUBS)
		self.clientID=vrep.simxStart('127.0.0.1',porta,True,True,5000,5) # Connect to V-REP

		if self.clientID!=-1:
			# start the simulation:
			vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_blocking)
		else:
			print ("Processo %i não iniciou uma conexão com o simulador, abortando!!" % (idx))
			exit()

		self.pub_queue = pub_queue
		'''
		rospy.init_node('controller_A3C'+simu_name_id, anonymous=True)
		self.reset_pub = rospy.Publisher(simu_name_id+'/reset', Bool, queue_size=1) # define publisher para resetar simulação
		self.pos_pub = rospy.Publisher(simu_name_id+'/joint_pos', Float32MultiArray, queue_size=1) #define publisher para as posições
		'''
		pub_rate = rospy.Rate(N_PUBS_STEP/TIME_STEP_ACTION)

		self.ack = False
		self.cmd = False

		self.sub_controller = Controlador(idx, pub_queue, pub_rate, t_ori, t_acc, t_pos, t_joint, t_force, t_pos_feet, gravity_compensation_enable=True)
		rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+'/ack', Bool, self.ack_callback)
		print("Ambiente inicializado!!")

	def reset(self):
		#reset robot on environment
		self.cmd =  False
		#self.reset_pub.publish(Bool(self.cmd))

		angles_noise = (np.random.rand(12)*2 -1)*ANGLE_NOISE
		orientation_noise = (np.random.rand(2)*2 -1)*ORIENTATION_NOISE
		msg = [1. if self.cmd else 0.]+angles_noise.tolist()+orientation_noise.tolist()

		init_state = self.sub_controller.reset(msg)
		self.pub_queue.put([True, self.w_id, msg])
		self.wait_ack() #wait for sim confirmation
		return init_state

	def step(self, action):
		s, done, r, info = self.sub_controller.step(action, self.cmd)
		self.cmd = not self.cmd
		return s, r, done, info

	def ack_callback(self, msg):
		self.ack = True

	def pause_simulation_dynamics(self):
		self.cmd = not self.cmd
		cmd_to_float = 1. if self.cmd else 0.
		self.pub_queue.put([False, self.w_id, [0.]*18+[cmd_to_float]]) #command to pause simulation

	def wait_ack(self):
		#wait for vrep reset simulation
		self.ack = False
		timer = 0
		while (not self.ack and timer < TIME_WAIT_ACK_MAX):
			time.sleep(TIME_WAIT_ACK)
			timer += TIME_WAIT_ACK

	def close(self):
		vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)

