# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from k_solver.core import Actuator


class Body():
    def __init__(self, angulos_braco=None):

        # CALCULANDO CENTRO DE MASSA DA PARTE SUPERIOR QUE É CONSIDERADA STÁTICA DURANTE A CAMINHADA
        ###########################################################################################
        self.TORSO_COM = np.array([0, 0, +3.2125e-1])
        if angulos_braco is None:
            angulos_braco = [0, 0, 0, 0, 0, 0]
        joints_pos = np.array([
            [+1.8050e-2, -3.4000e-2, +3.5668e-1],
            [+1.8550e-2, -7.6400e-2, +3.4220e-1],
            [+2.8550e-2, -7.6400e-2, +2.9520e-1]])
        coms_pos = np.array([
            [+1.8000e-2, -6.8450e-2, +3.5173e-1],
            [+1.8353e-2, -7.5398e-2, +3.1635e-1],
            [+2.8550e-2, -7.6400e-2, +2.5370e-1]])
        atuador_braco_dir = Actuator(
            [
                joints_pos[0] - self.TORSO_COM,
                "y",
                joints_pos[1] - joints_pos[0],
                "x",
                joints_pos[2] - joints_pos[1],
                "y",
                coms_pos[2] - joints_pos[2]
            ],
            center_of_mass_shitfts=[[0, 0, 0], coms_pos[0] - joints_pos[0], coms_pos[1] - joints_pos[1],
                                    coms_pos[2] - joints_pos[2]],
            mass_parts=[1, 8.000e-3, 1.200e-1, 2.000e-2]
        )
        atuador_braco_dir.angles = angulos_braco[:3]
        com_sup_dir = atuador_braco_dir.com()
        joints_pos = np.array([
            [+1.8050e-2, +3.3625e-2, +3.5668e-1],
            [+1.7350e-2, +7.6400e-2, +3.4220e-1],
            [+2.8353e-2, +7.6401e-2, +2.9520e-1]])
        coms_pos = np.array([
            [+1.8000e-2, +6.8350e-2, +3.5173e-1],
            [+1.8352e-2, +7.5403e-2, +3.1634e-1],
            [+2.8354e-2, +7.5402e-2, +2.5370e-1]])
        atuador_braco_esq = Actuator(
            [
                joints_pos[0] - self.TORSO_COM,
                "y",
                joints_pos[1] - joints_pos[0],
                "x",
                joints_pos[2] - joints_pos[1],
                "y",
                coms_pos[2] - joints_pos[2]
            ],
            center_of_mass_shitfts=[[0, 0, 0], coms_pos[0] - joints_pos[0], coms_pos[1] - joints_pos[1],
                                    coms_pos[2] - joints_pos[2]],
            mass_parts=[1, 8.000e-3, 1.200e-1, 2.000e-2]
        )
        atuador_braco_esq.angles = angulos_braco[3:]
        com_sup_esq = atuador_braco_esq.com()

        self.pub = rospy.Publisher('Bioloid/body_solver/torso_angles', Float32MultiArray, queue_size=1)

        rospy.Subscriber("/Bioloid/joint_pos", Float32MultiArray,
                         self.update_current_angles_callback)

        self.comTroncoPos = ((com_sup_esq + com_sup_dir) / 2) + self.TORSO_COM

        # GERANDO MANIPULADORES COM BASE NO PÉ ESQUERDO E DIREITO
        #########################################################
        degree_of_freedom = [
            "x",# LEFT ANKLE ROLL
            "y",# LEFT ANKLE PITCH
            "y",# LEFT KNEE
            "y",# LEFT HIP PITCH
            "x",# LEFT HIP ROLL
            "z",# LEFT HIP YALL
            "z",# RIGHT ANKLE ROLL
            "x",# RIGHT ANKLE PITCH
            "y",# RIGHT KNEE
            "y",# RIGHT HIP PITCH
            "y",# RIGHT HIP ROLL
            "x"]# RIGHT HIP YALL
        # esquerda para direita (duas pernas)

        self.LEFT_ANKLE_ROLL = 0
        self.LEFT_ANKLE_PITCH = 1
        self.LEFT_KNEE = 2
        self.LEFT_HIP_PITCH = 3
        self.LEFT_HIP_ROLL = 4
        self.LEFT_HIP_YALL = 5
        self.RIGHT_ANKLE_ROLL = 6
        self.RIGHT_ANKLE_PITCH = 7
        self.RIGHT_KNEE = 8
        self.RIGHT_HIP_PITCH = 9
        self.RIGHT_HIP_ROLL = 10
        self.RIGHT_HIP_YALL = 11

        joints_pos = np.array([
            [+1.8734e-2, -3.3164e-2, +4.3530e-2],# LEFT ANKLE ROLL
            [+3.5700e-2, -3.5004e-2, +4.3526e-2],# LEFT ANKLE PITCH
            [+3.5700e-2, -3.5004e-2, +1.4353e-1],# LEFT KNEE
            [+3.5700e-2, -3.5003e-2, +2.4095e-1],# LEFT HIP PITCH
            [+1.8885e-2, -3.2980e-2, +2.4090e-1],# LEFT HIP ROLL
            [+1.6052e-2, -3.3005e-2, +2.8929e-1],# LEFT HIP YALL
            [+1.6049e-2, +3.2993e-2, +2.8930e-1],# RIGHT ANKLE ROLL
            [+1.8885e-2, +3.2982e-2, +2.4090e-1],# RIGHT ANKLE PITCH
            [+3.5700e-2, +3.5000e-2, +2.4095e-1],# RIGHT KNEE
            [+3.5700e-2, +3.5000e-2, +1.4353e-1],# RIGHT HIP PITCH
            [+3.5700e-2, +3.5000e-2, +4.3526e-2],# RIGHT HIP ROLL
            [+1.8734e-2, +3.3167e-2, +4.3530e-2],# RIGHT HIP YALL
            [+1.8734e-2, +3.3167e-2, 0.004663]  # posição do pé que está no ar, não é junta
        ])
        # coms_pos = np.array([
        #     [+1.8734e-2, -3.3163e-2, +3.3015e-2],
        #     [+1.9450e-2, -3.3124e-2, +5.7527e-2],
        #     [+3.5700e-2, -3.3149e-2, +9.3497e-2],
        #     [+3.5375e-2, -3.2882e-2, +1.8951e-1],
        #     [+1.9000e-2, -3.3002e-2, +2.2700e-1],
        #     [+1.8825e-2, -3.3000e-2, +2.5348e-1],
        #     [self.comTroncoPos],
        #     [+1.8825e-2, +3.2999e-2, +2.5348e-1],
        #     [+1.9000e-2, +3.3000e-2, +2.2700e-1],
        #     [+3.5375e-2, +3.3024e-2, +1.8951e-1],
        #     [+3.5700e-2, +3.3147e-2, +9.3500e-2],
        #     [+1.9450e-2, +3.3126e-2, +5.7527e-2],
        #     [+1.8728e-2, +3.3167e-2, +3.3010e-2]
        # ])

        v = [[0., 0., +3.8867e-2]]
        # t = [[0., 0., ]]
        for i, c in enumerate(degree_of_freedom):
            v.append(c)
            v.append(joints_pos[i + 1] - joints_pos[i])
        self.perna_esq_para_dir = Actuator(
            v,
            center_of_mass_shitfts=None,
            mass_parts=None
        )
        
        joints_pos = np.delete(joints_pos, 11, 0)
        joints_pos = np.insert(joints_pos, 0, [+1.8734e-2, -3.3164e-2, 0.004663], 0)

        v = [[0., 0., +3.8867e-2]]
        for i, c in enumerate(degree_of_freedom):
            v.append(c)
            v.append(joints_pos[12 - (i + 1)] - joints_pos[12 - i])
        self.perna_dir_para_esq = Actuator(
            v,
            center_of_mass_shitfts=None,
            mass_parts=None
        )
        print("CoM TORSO: %s"%self.comTroncoPos)
        a = self.perna_esq_para_dir
        print("perna_esq_para_dir:  angles: %s | ee: %s"%(a.angles, a.ee))
        a = self.perna_dir_para_esq
        print("perna_dir_para_esq: angles: %s | ee: %s"%(a.angles, a.ee))



    def update_current_angles_callback(self, msg):
        self.perna_esq_para_dir.angles = np.array(msg.data[:12])
        self.perna_dir_para_esq.angles = np.array(msg.data[11::-1])
        msg_to_publish = Float32MultiArray()
        msg_to_publish.data = np.concatenate(self.perna_esq_para_dir.ee, self.perna_dir_para_esq.ee)





if __name__ == "__main__":
    a = Body()

# '''
# ##COM PARTS POSITION
# 0, 0, +3.2125e-1 #torso
# #perna direita
# +1.8734e-2, -3.3163e-2, +3.3015e-2 # pé
# +1.9450e-2, -3.3124e-2, +5.7527e-2 # calcanhar
# +3.5700e-2, -3.3149e-2, +9.3497e-2 # canela
# +3.5375e-2, -3.2882e-2, +1.8951e-1 # coxa
# +1.9000e-2, -3.3002e-2, +2.2700e-1 # bunda
# +1.8825e-2, -3.3000e-2, +2.5348e-1 # Liga o toso e a bunda(ultimo link da perna)
# #perna esquerda
# +1.8728e-2, +3.3167e-2, +3.3010e-2 # pé
# +1.9450e-2, +3.3126e-2, +5.7527e-2 # calcanhar
# +3.5700e-2, +3.3147e-2, +9.3500e-2 # canela
# +3.5375e-2, +3.3024e-2, +1.8951e-1 # coxa
# +1.9000e-2, +3.3000e-2, +2.2700e-1 # bunda
# +1.8825e-2, +3.2999e-2, +2.5348e-1 # Liga o toso e a bunda(ultimo link da perna)
# #braço direito
# +2.8550e-2, -7.6400e-2, +2.5370e-1 # braço
# +1.8353e-2, -7.5398e-2, +3.1635e-1 # ante-braço
# +1.8000e-2, -6.8450e-2, +3.5173e-1 # ombro
# #braço esquerdo
# +2.8354e-2, +7.5402e-2, +2.5370e-1 # braço
# +1.8352e-2, +7.5403e-2, +3.1634e-1 # ante-braço
# +1.8000e-2, +6.8350e-2, +3.5173e-1 # ombro
#
#
# ##MASS (kg)
# 1.000e+0
#
# 1.000e-1
# 1.300e-1
# 4.000e-2
# 1.200e-1
# 1.300e-1
# 1.500e-2
#
# 1.000e-1
# 1.300e-1
# 4.000e-2
# 1.200e-1
# 1.300e-1
# 1.500e-2
#
# 2.000e-2
# 1.200e-1
# 8.000e-3
#
# 2.000e-2
# 1.200e-1
# 8.000e-3
#
# ##JOINTS POSITION
# #perna direita
# +1.8734e-2, -3.3164e-2, +4.3530e-2
# +3.5700e-2, -3.5004e-2, +4.3526e-2
# +3.5700e-2, -3.5004e-2, +1.4353e-1
# +3.5700e-2, -3.5003e-2, +2.4095e-1
# +1.8885e-2, -3.2980e-2, +2.4090e-1
# +1.6052e-2, -3.3005e-2, +2.8929e-1
# #perna esquerda
# +1.8734e-2, +3.3167e-2, +4.3530e-2
# +3.5700e-2, +3.5000e-2, +4.3526e-2
# +3.5700e-2, +3.5000e-2, +1.4353e-1
# +3.5700e-2, +3.5000e-2, +2.4095e-1
# +1.8885e-2, +3.2982e-2, +2.4090e-1
# +1.6049e-2, +3.2993e-2, +2.8930e-1
# #braço direito
# +2.8550e-2, -7.6400e-2, +2.9520e-1
# +1.8550e-2, -7.6400e-2, +3.4220e-1
# +1.8050e-2, -3.4000e-2, +3.5668e-1
# #braço esquerdo
# +2.8353e-2, +7.6401e-2, +2.9520e-1
# +1.7350e-2, +7.6400e-2, +3.4220e-1
# +1.8050e-2, +3.3625e-2, +3.5668e-1
# '''
#
# '''
# +5.711e+0
# +3.495e+1
# -6.876e+1
# +3.381e+1
# +5.711e+0
# '''
