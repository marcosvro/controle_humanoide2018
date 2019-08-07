# -*- coding:utf-8 -*-

import rospy
import numpy as np
import os

RAD_TO_DEG = 180.0/np.pi
RAD_TO_DEG_10 = 10.0* RAD_TO_DEG

PARAM_SERVER_PREFIX = "/Bioloid/params/angles/calibration/"

# NO_CORRECTION_PATH = "noCorrection/"

ANGLE_LIMIT_PATH = "angleLimit"

# ANGLE_MIN_MAX_CORRECTION_PATH = "torque_limit/"

STEP_TIME = "step_time"

NUM_STATES = "num_states"

GRAVITY_COMPENSATION_ENABLE = "gravity_compensation_enable"

# JOINT_TORQUE_CORRECTION_ENABLE = "torque_joint_correction_enable"

# IGNORE_JOINT_CORRECTION_TIME = "ignore_joint_correction"

JOINTS_KP = "joints_kp"

DISPLACEMENT_PATH = "displacement"

INERTIAL_FOOT_ENABLE = "inertial_foot_enable"

JOINT_LIMIT_ENFORCEMENT_ENABLED = "limit_enforcement_enabled"

# JOINT_DIRECTION_MULTIPLIER = "joint_direction_multiplier"

PELVES_Z_MAX_DISPLACEMENT = "pelves_z_max"

PELVES_Y_MAX_DISPLACEMENT = "pelves_y_max"

FOOT_Z_MAX_DISPLACEMENT = "foot_z_max"

FOOT_X_MAX_DISPLACEMENT = "foot_x_max"

MIN = "min"
MAX = "max"

PARAM_NAMES = [
    'RIGHT_ANKLE_ROLL ',
    'RIGHT_ANKLE_PITCH',
    'RIGHT_KNEE       ',
    'RIGHT_HIP_PITCH  ',
    'RIGHT_HIP_ROLL   ',
    'RIGHT_HIP_YALL   ',
    'LEFT_ANKLE_ROLL  ',
    'LEFT_ANKLE_PITCH ',
    'LEFT_KNEE        ',
    'LEFT_HIP_PITCH   ',
    'LEFT_HIP_ROLL    ',
    'LEFT_HIP_YALL    ',
    'LEFT_ARM_PITCH   ',
    'LEFT_ARM_YALL    ',
    'LEFT_ARM_ROLL    ',
    'RIGHT_ARM_PITCH  ',
    'RIGHT_ARM_YALL   ',
    'RIGHT_ARM_ROLL   '
]

STRIPPED_PARAM_NAMES = map(lambda x: x.strip(), PARAM_NAMES)

PARAMS_DICT = {
    DISPLACEMENT_PATH:{
        FOOT_X_MAX_DISPLACEMENT: 1.5,
        FOOT_Z_MAX_DISPLACEMENT: 3.0,
        PELVES_Y_MAX_DISPLACEMENT: 2.,
        PELVES_Z_MAX_DISPLACEMENT: 30.
    },
    # JOINT_DIRECTION_MULTIPLIER: 1,
    STEP_TIME: .3,
    NUM_STATES: 125,
    GRAVITY_COMPENSATION_ENABLE: False,
    # IGNORE_JOINT_CORRECTION_TIME: 0.1
}

for joint in STRIPPED_PARAM_NAMES:
    PARAMS_DICT[joint] = {
        # NO_CORRECTION_PATH:{
        #     MIN: 0.,
        #     MAX: 0.
        # },
        # ANGLE_MIN_MAX_CORRECTION_PATH:{
        #     MIN: 0.,
        #     MAX: 0.
        # },
        ANGLE_LIMIT_PATH:{
            MIN: 0.,
            MAX: 0.
        },
        # JOINT_DIRECTION_MULTIPLIER: 1,
        JOINTS_KP: 0.3
    }

PARAMS_LIST = []

PARAM_TYPES = {}

PARAMS_LIST = list()

def param_helper(prepend_str, _dict, append=False):
    if type(_dict) is not dict:
        _type = type(_dict)
        ex = False
        try:
            value = rospy.get_param(PARAM_SERVER_PREFIX + prepend_str)
        except KeyError as e:
            value = _dict
            ex = True
        if prepend_str not in PARAM_TYPES:
            PARAM_TYPES[prepend_str] = _type if value is None else type(value)
            PARAMS_LIST.append([prepend_str, value])
        if ex:
            rospy.set_param(PARAM_SERVER_PREFIX + "/" + prepend_str, PARAM_TYPES[prepend_str](value))
        else:
            if(prepend_str in PARAMS_LIST):
                PARAMS_LIST[PARAMS_LIST.index(prepend_str)] = value
    else:
        for x, y in _dict.items():
            param_helper(prepend_str + x + ('/' if type(y) is dict else ''), y, append=append)
    # print(json.dumps(_dict, indent=4, separators=(',',':')))

param_helper('', PARAMS_DICT, append=True)
class LoosenessCalibrator():
    def __init__(self):
        rospy.init_node("param_setter",anonymous=True)
        self.loop = True
        while (self.loop and not rospy.is_shutdown()):
            param_helper('', PARAMS_DICT)
                
            os.system("clear")
            os.system("rm -f ./temp_params")
            print("idx | PARÂMETRO                         | valor atual")
            for x in enumerate(PARAMS_LIST):
                idx, param_data = x
                name, value = param_data
                os.system("echo \"" + name + " | " + str(value) + " | " + str(PARAM_TYPES[name]) + "\" >> ./temp_params")
                print(idx, name, value)
            print("informe o indice e o valor desejado. exemplos: '0 0', '5 20.5'")
            print("digite q para sair")
            try:
                try:
                    read_data = raw_input()
                except Exception as e:
                    read_data = input()
                    pass
                if(read_data == "q"):
                    self.loop = False
                    break
                index, value = read_data.split(" ",1)
                index = int(index)
                value = PARAM_TYPES[PARAMS_LIST[index][0]](value)
                PARAMS_LIST[index][1] = value
                index = int(index)
                rospy.set_param(PARAM_SERVER_PREFIX + PARAMS_LIST[index][0], value)
            except Exception as e:
                print(e)
                pass
            

            

if __name__ == "__main__":
    calibrator = LoosenessCalibrator()
