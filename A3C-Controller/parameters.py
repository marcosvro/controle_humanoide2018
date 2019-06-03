from environment import GetStateSize, GetActionSize

#State set
COM_IN_STATE = True # If true, center of mass position inserted in state array
TARGETS_POS_IN_STATE = True # If true, position of swing foot, hip of support leg and torso are in state array
COM_VELOCITY_IN_STATE = False # If true, vector velocity of center of mass(CoM) are in state array
COM_ACCELERATION_IN_STATE = False #If true, vector acceleration of CoM are in state array
TORSO_ORIENTATION_IN_STATE = True #If true, torso orientation(IMU response) are in state array
LAST_ACTION_IN_STATE = True #If true, last action are embbeded in state 

#action mode
USING_MARCOS_CONTROLLER = False #If true, the action are composed by variables set of Marcos controller. If false, the action is composed by 3 arrays (3D) indicating the position of swing foot, hip of support leg and torso

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

#marcos controller parameters
UPPER_LEG_LENGHT = 10.5
LOWER_LEG_LENGHT = 10.2
TIME_TO_IGNORE_GC = 0.1

#servo paramters
KP_CONST = 0.6

#simulation
TIME_STEP_ACTION = 0.1
N_PUBS_STEP = 5
ACTION_BOUND_LOW = -30
ACTION_BOUND_HIGH = 30

#network
OUTPUT_GRAPH = True         # safe logs
RENDER=True                 # render one worker
LOG_DIR = './log'           # savelocation for logs
N_WORKERS = 4							  # number of workers
MAX_EP_STEP = 200           # maxumum number of steps per episode
MAX_GLOBAL_EP = 2000        # total number of episodes
GLOBAL_NET_SCOPE = 'Global_Net'
UPDATE_GLOBAL_ITER = 10     # sets how often the global net is updated
GAMMA = 0.90                # discount factor
ENTROPY_BETA = 0.01         # entropy factor
LR_A = 0.0001               # learning rate for actor
LR_C = 0.001                # learning rate for critic
N_S = GetStateSize()        # number of states
N_A = GetActionSize()       # number of actions
A_BOUND = [ACTION_BOUND_LOW, ACTION_BOUND_HIGH] # action bounds