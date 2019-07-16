# coding: utf-8
#utils
DEG_2_RAD = 3.141592653589793/180.
RAD_2_DEG = 180./3.141592653589793

#State set
COM_IN_STATE = False # If true, center of mass position inserted in state array
TARGETS_POS_IN_STATE = False # If true, position of right leg, left leg and torso target are in state array
TORSO_ACCELERATION_IN_STATE = False #If true, vector acceleration of TORSO are in state array
TORSO_ORIENTATION_IN_STATE = True #If true, torso orientation(IMU response) are in state array
LAST_ACTION_IN_STATE = False #If true, last action are embbeded in state
LEG_JOINT_POSITION_IN_STATE = True #If true, real joint position of robot simulation are embbede in state

#action mode
USING_MARCOS_CONTROLLER = False #If true, the action are composed by variables set of Marcos controller. If false, the action is composed by 3 arrays (2 3D, 1 2D) indicating the position of swing foot, hip of support leg and torso angles velocity

#state init values
HEIGHT_INIT = 17.
TIME_STEP_INIT = 1.
DISTANCE_FOOT_INIT = 0.
SHIFT_X_FOOT_INIT = 0.
SHIFT_Y_HIP_INIT = 0
SHIFT_Z_FOOT_INIT = 0
ANGLE_Z_HIP_INIT = 0

#constaints of controller
DISTANCE_FOOT_MAX = 2.
SHIFT_X_FOOT_MAX = 2.
SHIFT_Z_FOOT_MAX = 2.
SHIFT_Y_HIP_MAX = 2.
ANGLE_Z_HIP_MAX = 50.
VELOCITY_POINTS_MAX = 1.
TIME_STEP_MAX = 1.5
TIME_STEP_MIN = 0.5

#marcos controller parameters
UPPER_LEG_LENGHT = 10.5
LOWER_LEG_LENGHT = 10.2
TIME_TO_IGNORE_GC = 0.1

#servo paramters
KP_CONST = 0.6

#simulation
TESTING = False
VREP_PATH = '~/vrep'
SCENE_FILE_PATH = '~/Documentos/controle_humanoide2018/teste_09_03.ttt'
TIME_STEP_ACTION = 0.25
N_PUBS_STEP = 5
ACTION_BOUND_LOW = -1
ACTION_BOUND_HIGH = 1
TIME_WAIT_ACK = 0.1
TIME_WAIT_ACK_MAX = 4
TIME_WAIT_INIT_PUBS = 5
ANGLE_FALLEN_THRESHOLD = 70*DEG_2_RAD
TARGET_BOUND_RANGE = 5.

#task rewards weight
W_ORI = 0.2
W_DIST = 50.
W_INC = 1.
W_ALIVE = 5.

#network
OUTPUT_GRAPH = True         # safe logs
RENDER=True                 # render one worker
LOG_DIR = './log/weigths'   # savelocation for logs
N_WORKERS = 20  	# number of workers
MAX_EP_STEP = 100            # maxumum number of steps per episode
MAX_EP = 1000000            # maximum number of episodes
MAX_GLOBAL_EP = MAX_EP      # idem MAX_EP, but to tensorflow A3C implementation.
GLOBAL_NET_SCOPE = 'Global_Net'
UPDATE_GLOBAL_ITER = 5      # sets how often the global net is updated
GAMMA = 0.90                # discount factor
ENTROPY_BETA = 0.01         # entropy factor
LR_A = 0.0001               # learning rate for actor
LR_C = 0.001                # learning rate for critic
A_BOUND = [ACTION_BOUND_LOW, ACTION_BOUND_HIGH] # action bounds

# number of actions
N_A = 0
if USING_MARCOS_CONTROLLER:
	N_A += 1 # Swing foot height
	N_A += 1 # Step length
	N_A += 1 # Side shift of hip
	N_A += 1 # Hip height
	N_A += 1 # Distance between feet
	#N_A += 1 # Angles to turn
	#N_A += 1 # Step time
else:
	N_A += 10 # Velocity of swing foot, hip of support leg and torso angles

# number of state parts
S_P = 4

# number of states
N_PS = 0
if COM_IN_STATE:
	N_PS += 6
if TARGETS_POS_IN_STATE:
	N_PS += 8
if TORSO_ACCELERATION_IN_STATE:
	N_PS += 3
if TORSO_ORIENTATION_IN_STATE:
	N_PS += 3
if LAST_ACTION_IN_STATE:
	N_PS += N_A
if USING_MARCOS_CONTROLLER:
	N_PS += 2
if LEG_JOINT_POSITION_IN_STATE:
	N_PS += 12
N_S = N_PS*S_P*2
