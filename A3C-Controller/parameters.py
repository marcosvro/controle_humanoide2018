# coding: utf-8
#utils
DEG_2_RAD = 3.141592653589793/180.
RAD_2_DEG = 180./3.141592653589793

#State set
COM_IN_STATE = True # If true, center of mass position inserted in state array
TARGETS_POS_IN_STATE = False # If true, position of right leg, left leg and torso target are in state array
TORSO_ACCELERATION_IN_STATE = True #If true, vector acceleration of TORSO are in state array
TORSO_ORIENTATION_IN_STATE = True #If true, torso orientation(IMU response) are in state array
LAST_ACTION_IN_STATE = True #If true, last action are embbeded in state

#action mode
USING_MARCOS_CONTROLLER = True #If true, the action are composed by variables set of Marcos controller. If false, the action is composed by 3 arrays (2 3D, 1 2D) indicating the position of swing foot, hip of support leg and torso angles velocity

#state init values
HEIGHT_INIT = 17.
TIME_STEP_INIT = 1.
DISTANCE_FOOT_INIT = 2.8
SHIFT_X_FOOT_INIT = 0.
SHIFT_Y_HIP_INIT = 0
SHIFT_Z_FOOT_INIT = 0
ANGLE_Z_HIP_INIT = 0

#constaints of controller
SHIFT_X_FOOT_MAX = 2.
SHIFT_Z_FOOT_MAX = 2.6
SHIFT_Y_HIP_MAX = 3.
ANGLE_Z_HIP_MAX = 50.
VELOCITY_POINTS_MAX = 1.
TIME_STEP_MAX = 2.

#marcos controller parameters
UPPER_LEG_LENGHT = 10.5
LOWER_LEG_LENGHT = 10.2
TIME_TO_IGNORE_GC = 0.1

#servo paramters
KP_CONST = 0.6

#simulation
TESTING = True
VREP_PATH = '~/vrep'
SCENE_FILE_PATH = '~/Documentos/controle_humanoide2018/teste_09_03.ttt'
TIME_STEP_ACTION = 0.2
N_PUBS_STEP = 5
ACTION_BOUND_LOW = -1
ACTION_BOUND_HIGH = 1
TIME_WAIT_ACK = 0.1
TIME_WAIT_ACK_MAX = 4
TIME_WAIT_INIT_PUBS = 15
ANGLE_FALLEN_THRESHOLD = 60*DEG_2_RAD
TARGET_BOUND_RANGE = 1.5

#task rewards weight
W_ORI = 0.25
W_DIST = 0.5
W_INC = 0.25

#network
OUTPUT_GRAPH = True         # safe logs
RENDER=True                 # render one worker
LOG_DIR = './log'           # savelocation for logs
N_WORKERS = 1				# number of workers
MAX_EP_STEP = 40            # maxumum number of steps per episode
MAX_GLOBAL_EP = 2000        # total number of episodes
GLOBAL_NET_SCOPE = 'Global_Net'
UPDATE_GLOBAL_ITER = 10     # sets how often the global net is updated
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
	N_A += 1 # Angles to turn
	N_A += 1 # Step time
else:
	N_A += 10 # Velocity of swing foot, hip of support leg and torso angles

# number of states
N_S = 2
if COM_IN_STATE:
	N_S += 6
if TARGETS_POS_IN_STATE:
	N_S += 8
if TORSO_ACCELERATION_IN_STATE:
	N_S += 3
if TORSO_ORIENTATION_IN_STATE:
	N_S += 3
if LAST_ACTION_IN_STATE:
	N_S += N_A
if USING_MARCOS_CONTROLLER:
	N_S += 1