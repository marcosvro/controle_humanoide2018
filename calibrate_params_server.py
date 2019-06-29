# -*- coding:utf-8 -*-

import rospy
import numpy as np
import os

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
        self.looseness_correction_angles = [0]*18
        while not rospy.is_shutdown():
            for idx, param_name in enumerate(PARAM_NAMES):
                
                os.system("clear")
                print("angulo de correção atual para " + param_name + ": ", self.looseness_correction_angles[idx])

                read_angle = input("Informe o novo angulo necessário para corrigir a folga da junta " + param_name)
                read_angle = float(read_angle)
                self.looseness_correction_angles[idx] = read_angle

                rospy.set_param(PARAM_SERVER_PREFIX + param_name, read_angle)

            

if __name__ == "__main__":
    calibrator = LoosenessCalibrator()
