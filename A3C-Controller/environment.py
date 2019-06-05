import time
from parameters import *
from controlador import Controlador
import rospy
from std_msgs.msg import Float32MultiArray, Bool
import os

class VrepEnvironment():
	def __init__ (self, simu_name_id):
		self.reset_pub = rospy.Publisher(simu_name_id+'/reset', Bool, queue_size=1) # define publisher para resetar simulação
		self.pos_pub = rospy.Publisher(simu_name_id+'/joint_pos', Float32MultiArray, queue_size=1) #define publisher para as posições
		rospy.init_node('controller_A3C')
		self.pub_rate = rospy.Rate(N_PUBS_STEP/TIME_STEP_ACTION)

		self.ack = False
		ack_confirm = not self.ack

		#incialize vrep simulation and wait for confirmation
		os.system(VREP_PATH+"/vrep.sh -h -s -g"+simu_name_id+" "+SCENE_FILE_PATH)

		self.sub_controller = Controlador(simu_name_id, self.pos_pub, self.pub_rate, gravity_compensation_enable=False)
		rospy.Subscriber("/vrep_ros_interface/"+simu_name_id+'/ack', Bool, self.ack_callback)


	def reset(self):
		#reset robot on environment
		ack_confirm = not self.ack
		self.reset_pub.publish(Bool(self.ack))
		self.sub_controller.reset()
		init_state, d, r = self.sub_controller.get_state()
		self.wait_ack(ack_confirm) #wait for sim confirmation
		return init_state

	def step(self, action):
		info = {}
		s, done, r = self.sub_controller.step(action)
		return s, r, done, info


	def ack_callback(self, msg):
		self.ack = msg.data

	def wait_ack(self, ack_confirm):
		#wait for vrep reset simulation
		while (self.ack != ack_confirm):
			time.sleep(TIME_WAIT_ACK)

