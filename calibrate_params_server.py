# -*- coding:utf-8 -*-

import rospy
import numpy as np
import os

RAD_TO_DEG = 180/np.pi
RAD_TO_DEG_10 = 10* RAD_TO_DEG

PARAM_SERVER_PREFIX = "/Bioloid/params/angles/calibration/"

PARAM_NAMES = [
    'RIGHT_ANKLE_ROLL',
    'RIGHT_ANKLE_PITCH',
    'RIGHT_KNEE',
    'RIGHT_HIP_PITCH',
    'RIGHT_HIP_ROLL',
    'RIGHT_HIP_YALL',
    'LEFT_ANKLE_ROLL',
    'LEFT_ANKLE_PITCH',
    'LEFT_KNEE',
    'LEFT_HIP_PITCH',
    'LEFT_HIP_ROLL',
    'LEFT_HIP_YALL',
    'LEFT_ARM_PITCH',
    'LEFT_ARM_YALL',
    'LEFT_ARM_ROLL',
    'RIGHT_ARM_PITCH',
    'RIGHT_ARM_YALL',
    'RIGHT_ARM_ROLL'
]

class LoosenessCalibrator():
    def __init__(self):
        rospy.init_node("param_setter",anonymous=True)
        while not rospy.is_shutdown():
            for idx, param_name in enumerate(PARAM_NAMES):
                
                os.system("clear")
                
                print("angulo(em graus) de correção atual para " + param_name + ": ", RAD_TO_DEG_10 * rospy.get_param(PARAM_SERVER_PREFIX + param_name, 0))
                print("Informe o novo angulo(em graus) necessário para corrigir a folga da junta " + param_name)
                read_angle = input()
                read_angle = float(read_angle / RAD_TO_DEG_10)

                rospy.set_param(PARAM_SERVER_PREFIX + param_name, read_angle)

            

if __name__ == "__main__":
    calibrator = LoosenessCalibrator()
