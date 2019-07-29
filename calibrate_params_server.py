# -*- coding:utf-8 -*-

import rospy
import numpy as np
import os

RAD_TO_DEG = 180.0/np.pi
RAD_TO_DEG_10 = 10.0* RAD_TO_DEG

PARAM_SERVER_PREFIX = "/Bioloid/params/angles/calibration/"

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
class LoosenessCalibrator():
    def __init__(self):
        rospy.init_node("param_setter",anonymous=True)
        self.loop = True
        while (self.loop and not rospy.is_shutdown()):
            os.system("clear")
            print("idx | Junta             | angulo atual")
            for idx, param_name in enumerate(PARAM_NAMES):
                print("" + str(idx) + " | " + str(param_name) + " | " + str(RAD_TO_DEG_10 * rospy.get_param(PARAM_SERVER_PREFIX + STRIPPED_PARAM_NAMES[idx], 0)))
                
            print("informe o indice e o angulo desejado. exemplos: '0 0', '5 20.5'")
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
                index, new_angle = read_data.split(" ",1)
                new_angle = float(new_angle)  / RAD_TO_DEG_10
                index = int(index)
                rospy.set_param(PARAM_SERVER_PREFIX + STRIPPED_PARAM_NAMES[index], new_angle)
            except Exception as e:
                pass
            

            

if __name__ == "__main__":
    calibrator = LoosenessCalibrator()
