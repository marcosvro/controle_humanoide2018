from parameters import *
from controlador import Controlador
import rospy
from std_msgs.msg import Float32MultiArray, Bool

def GetStateSize():
	state_size = 0
	if COM_IN_STATE:
		state_size += 3
	if TARGETS_POS_IN_STATE:
		state_size += 9
	if COM_VELOCITY_IN_STATE:
		state_size += 3
	if COM_ACCELERATION_IN_STATE:
		state_size += 3
	if TORSO_ORIENTATION_IN_STATE:
		state_size += 3
	if LAST_ACTION_IN_STATE:
		state_size += GetActionSize()
	return state_size

def GetActionSize():
	action_size = 0
	if USING_MARCOS_CONTROLLER:
		action_size += 1 # Swing foot height
		action_size += 1 # Step length
		action_size += 1 # Side shift of hip 
		action_size += 1 # Hip height
		action_size += 1 # Angles to turn
		action_size += 1 # Step time
	else:
		action_size += 9 # Velocity of swing foot, hip of support leg and torso
	return action_size

class VrepEnvironment():
	def __init__ (self, simu_name_id):
		self.joint_pos_pub = rospy.Publisher(simu_name_id+'/joint_pos', Float32MultiArray, queue_size=1)
		self.reset_pub = rospy.Publisher(simu_name_id+'/reset', Bool, queue_size=1)
		self.rate_joint_pos = rospy.Rate(N_PUBS_STEP/TIME_STEP_ACTION)
		self.outer = False
		self.sub_controller = Controlador(self.joint_pos_pub, self.rate_joint_pos, gravity_compensation_enable=True)
		rospy.init_node('controller_A3C')

		def reset(self):
			#reset robot on environment
			self.outer = not self.outer
			self.reset_pub.publish(Bool(self.outer))
			self.sub_controller.reset()
			return self.get_init_state()

		def get_init_state(self):


		def step(self, action):
			info = None
			return s, r, done, info

