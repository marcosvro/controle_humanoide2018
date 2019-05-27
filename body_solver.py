# -*- coding:utf-8 -*-

import numpy as np
from k_solver.core import Actuator
from std_msgs.msg import Float32MultiArray, Int8
try:
    import rospy
except Exception as e:
    print("erro ao importar rospy")
    raise e
import threading
import math


class Body():
    def __init__(self, angulos_braco=None, rate=1.5/125):

        self.perna = 0
        self.angulosJuntas = [0] * 18
        self.lastSentAngles = [0] * 12

        # CALCULANDO CENTRO DE MASSA DA PARTE SUPERIOR QUE É CONSIDERADA STÁTICA DURANTE A CAMINHADA
        ###########################################################################################
        if angulos_braco is None:
            angulos_braco = [0, 0, 0, 0, 0, 0]
        self.torsoCom = np.array([0, 0, +3.2125e-1])
        joints_pos = np.array([
            [+1.8050e-2, -3.4000e-2, +3.5668e-1],  # Right Shoulder Pitch
            [+1.8550e-2, -7.6400e-2, +3.4220e-1],  # Right Elbow Yall
            [+2.8550e-2, -7.6400e-2, +2.9520e-1]]) # Right Arm Roll
        coms_pos = np.array([
            [+1.8000e-2, -6.8450e-2, +3.5173e-1],
            [+1.8353e-2, -7.5398e-2, +3.1635e-1],
            [+2.8550e-2, -7.6400e-2, +2.5370e-1]])
        self.atuadorBracoDir = Actuator(
            [joints_pos[0] - self.torsoCom, "y", joints_pos[1] - joints_pos[0], "x", joints_pos[2] - joints_pos[1], "y",
             coms_pos[2] - joints_pos[2]],
            center_of_mass_shitfts=[[0, 0, 0], coms_pos[0] - joints_pos[0], coms_pos[1] - joints_pos[1],
                                    coms_pos[2] - joints_pos[2]], mass_parts=[1, 8.000e-3, 1.200e-1, 2.000e-2])
        self.atuadorBracoDir.angles = angulos_braco[:3]
        com_sup_dir = self.atuadorBracoDir.com()

        joints_pos = np.array([
            [+1.8050e-2, +3.3625e-2, +3.5668e-1],  # Left Shoulder Pitch
            [+1.7350e-2, +7.6400e-2, +3.4220e-1],  # Left Elbow Yall
            [+2.8353e-2, +7.6401e-2, +2.9520e-1]]) # Left Arm Roll
        coms_pos = np.array([
            [+1.8000e-2, +6.8350e-2, +3.5173e-1],
            [+1.8352e-2, +7.5403e-2, +3.1634e-1],
            [+2.8354e-2, +7.5402e-2, +2.5370e-1]])
        self.atuadorBracoEsq = Actuator(
            [joints_pos[0] - self.torsoCom, "y", joints_pos[1] - joints_pos[0], "x", joints_pos[2] - joints_pos[1], "y",
             coms_pos[2] - joints_pos[2]],
            center_of_mass_shitfts=[[0, 0, 0], coms_pos[0] - joints_pos[0], coms_pos[1] - joints_pos[1],
                                    coms_pos[2] - joints_pos[2]], mass_parts=[1, 8.000e-3, 1.200e-1, 2.000e-2])
        self.atuadorBracoEsq.angles = angulos_braco[3:]
        com_sup_esq = self.atuadorBracoEsq.com()

        self.comTroncoPos = ((com_sup_esq + com_sup_dir) / 2) + self.torsoCom
        self.TRONCO_MASS = self.atuadorBracoEsq.total_mass + self.atuadorBracoDir.total_mass - 1
        
        # GERANDO MANIPULADORES COM BASE NO PÉ ESQUERDO E ESQUERDO
        #########################################################
        degree_of_freedom = [
            "x",#  LEFT_ANKLE_ROLL
            "y",#  LEFT_ANKLE_PITCH
            "y",#  LEFT_KNEE_PITCH
            "y",#  LEFT_HIP_PITCH
            "x",#  LEFT_HIP_ROLL
            "z",#  LEFT_HIP_YALL
            "z",#  RIGHT_HIP_YALL
            "x",#  RIGHT_HIP_ROLL
            "y",#  RIGHT_HIP_PITCH
            "y",#  RIGHT_KNEE_PITCH
            "y",#  RIGHT_ANKLE_PITCH
            "x"]#  RIGHT_ANKLE_ROLL
        # esquerda para direita (duas perna)
        self.LEFT_ANKLE_ROLL        = 0
        self.LEFT_ANKLE_PITCH       = 1
        self.LEFT_KNEE_PITCH        = 2
        self.LEFT_HIP_PITCH         = 3
        self.LEFT_HIP_ROLL          = 4
        self.LEFT_HIP_YALL          = 5
        self.RIGHT_HIP_YALL         = 6
        self.RIGHT_HIP_ROLL         = 7
        self.RIGHT_HIP_PITCH        = 8
        self.RIGHT_KNEE_PITCH       = 9
        self.RIGHT_ANKLE_PITCH      = 10
        self.RIGHT_ANKLE_ROLL       = 11
        joints_pos = np.array([
            [+1.8734e-2, -3.3164e-2, +4.3530e-2],#  LEFT_ANKLE_ROLL
            [+3.5700e-2, -3.5004e-2, +4.3526e-2],#  LEFT_ANKLE_PITCH
            [+3.5700e-2, -3.5004e-2, +1.4353e-1],#  LEFT_KNEE_PITCH
            [+3.5700e-2, -3.5003e-2, +2.4095e-1],#  LEFT_HIP_PITCH
            [+1.8885e-2, -3.2980e-2, +2.4090e-1],#  LEFT_HIP_ROLL
            [+1.6052e-2, -3.3005e-2, +2.8929e-1],#  LEFT_HIP_YALL
            [+1.6049e-2, +3.2993e-2, +2.8930e-1],#  RIGHT_HIP_YALL
            [+1.8885e-2, +3.2982e-2, +2.4090e-1],#  RIGHT_HIP_ROLL
            [+3.5700e-2, +3.5000e-2, +2.4095e-1],#  RIGHT_HIP_PITCH
            [+3.5700e-2, +3.5000e-2, +1.4353e-1],#  RIGHT_KNEE_PITCH
            [+3.5700e-2, +3.5000e-2, +4.3526e-2],#  RIGHT_ANKLE_PITCH
            [+1.8734e-2, +3.3167e-2, +4.3530e-2],#  RIGHT_ANKLE_ROLL
            [+1.8734e-2, +3.3167e-2, 0.004663]])  # posição do pé que está no ar, não é junta
        coms_pos = np.array([
            [+1.8734e-2, -3.3163e-2, +3.3015e-2],
            [+1.9450e-2, -3.3124e-2, +5.7527e-2],
            [+3.5700e-2, -3.3149e-2, +9.3497e-2],
            [+3.5375e-2, -3.2882e-2, +1.8951e-1],
            [+1.9000e-2, -3.3002e-2, +2.2700e-1],
            [+1.8825e-2, -3.3000e-2, +2.5348e-1],
            self.comTroncoPos,
            [+1.8825e-2, +3.2999e-2, +2.5348e-1],
            [+1.9000e-2, +3.3000e-2, +2.2700e-1],
            [+3.5375e-2, +3.3024e-2, +1.8951e-1],
            [+3.5700e-2, +3.3147e-2, +9.3500e-2],
            [+1.9450e-2, +3.3126e-2, +5.7527e-2],
            [+1.8728e-2, +3.3167e-2, +3.3010e-2]])
        links_mass = np.array([
            1.000e-1,
            1.300e-1,
            4.000e-2,
            1.200e-1,
            1.300e-1,
            1.500e-2,
            self.TRONCO_MASS,
            1.500e-2,
            1.300e-1,
            1.200e-1,
            4.000e-2,
            1.300e-1,
            1.000e-1])

        v = []
        t = []
        v.append([0., 0., +3.8867e-2])
        t.append(np.array([+1.8734e-2, -3.3163e-2, +3.3015e-2]) - np.array(
            [+1.8734e-2, -3.3164e-2, 0.]))  # COM do primeiro link - Projeção da primeira junta no chão
        for i, c in enumerate(degree_of_freedom):
            v.append(c)
            v.append(joints_pos[i + 1] - joints_pos[i])
            t.append(coms_pos[i + 1] - joints_pos[i])

        self.pernaDirParaEsq = Actuator(v, center_of_mass_shitfts=t, mass_parts=links_mass)

        np.delete(joints_pos, 12, 0)
        np.insert(joints_pos, 0, [+1.8734e-2, -3.3164e-2, 0.004663], 0)
        v = []
        t = []
        v.append([0., 0., +3.8867e-2])
        t.append(np.array([+1.8728e-2, +3.3167e-2, +3.3010e-2]) - np.array([+1.8734e-2, +3.3167e-2, 0.]))
        for i, c in enumerate(degree_of_freedom):
            v.append(c)
            v.append(joints_pos[12 - (i + 1)] - joints_pos[12 - i])
            t.append(coms_pos[12 - (i + 1)] - joints_pos[12 - i])
        self.pernaEsqParaDir = Actuator(v, center_of_mass_shitfts=t, mass_parts=links_mass)
        
        rospy.init_node('gravity_compensator',anonymous=True)

        self.rate = rospy.Rate(rate) #(self.TEMPO_PASSO / self.N_ESTADOS)

        self.pub = rospy.Publisher('Bioloid/gravity_compensation', Float32MultiArray, queue_size=1)

        self.subLeg = rospy.Subscriber('Bioloid/support_leg', Int8, self.escuta_perna)

        self.subAngles = rospy.Subscriber('Bioloid/joint_pos', Float32MultiArray, self.atualiza_angulos)

        # t = threading.Thread(target = self.calcular_compensacao_gravitacional)
        # t.daemon = True
        # t.start()
        print("---")
        print(self.comTroncoPos)
        print("---")
        print(joints_pos)
        print("---")


    
    def atualiza_angulos(self, msg):
        self.angulosJuntas = msg.data

    # perna_deita_no_chao = 1 ou 0
    def escuta_perna(self, msg):
        self.perna = msg.data
        
    # angulos enviados para o micro
    def calcular_compensacao_gravitacional(self):

        hip_axis = [0,1,0]

        self.atuadorBracoDir.angles = self.angulosJuntas[15:18]

        self.atuadorBracoEsq.angles = self.angulosJuntas[12:13]

        angulos_aux = [
            self.angulosJuntas[6]  - self.lastSentAngles[0],      # LEFT_ANKLE_ROLL = 6               #  LEFT_ANKLE_ROLL   =  0
            self.angulosJuntas[7]  - self.lastSentAngles[1],      # LEFT_ANKLE_PITCH = 7              #  LEFT_ANKLE_PITCH  =  1
            self.angulosJuntas[8]  - self.lastSentAngles[2],      # LEFT_KNEE_PITCH = 8               #  LEFT_KNEE_PITCH   =  2
            self.angulosJuntas[9]  - self.lastSentAngles[3],      # LEFT_HIP_PITCH = 9                #  LEFT_HIP_PITCH    =  3
            self.angulosJuntas[10] - self.lastSentAngles[4],      # LEFT_HIP_ROLL = 10                #  LEFT_HIP_ROLL     =  4
            self.angulosJuntas[11] - self.lastSentAngles[5],      # LEFT_HIP_YALL = 11                #  LEFT_HIP_YALL     =  5
            self.angulosJuntas[5]  - self.lastSentAngles[6],      # RIGHT_HIP_YALL = 5                #  RIGHT_HIP_YALL    =  6
            self.angulosJuntas[4]  - self.lastSentAngles[7],      # RIGHT_HIP_ROLL = 4                #  RIGHT_HIP_ROLL    =  7
            self.angulosJuntas[3]  - self.lastSentAngles[8],      # RIGHT_HIP_PITCH = 3               #  RIGHT_HIP_PITCH   =  8
            self.angulosJuntas[2]  - self.lastSentAngles[9],      # RIGHT_KNEE_PITCH = 2              #  RIGHT_KNEE_PITCH  =  9
            self.angulosJuntas[1]  - self.lastSentAngles[10],     # RIGHT_ANKLE_PITCH = 1             #  RIGHT_ANKLE_PITCH =  10
            self.angulosJuntas[0]  - self.lastSentAngles[11]      # RIGHT_ANKLE_ROLL = 0              #  RIGHT_ANKLE_ROLL  =  11
        ]

        self.pernaEsqParaDir.angle = angulos_aux

        self.pernaDirParaEsq.angles = angulos_aux[11::-1] 

        r = self.comTroncoPos

        if self.perna:
            hip_axis = [
                math.sin(angulos_aux[6]), # RIGHT_HIP_YALL
                math.cos(angulos_aux[6])
            ]

        else:
            hip_axis = [
                math.sin(angulos_aux[5]), # LEFT_HIP_YALL
                math.sin(angulos_aux[5])
            ]
        
        

        torque = r * self.TRONCO_MASS * 9.80665 # GRAVIDADE

        # self.RIGHT_ANKLE_ROLL = 0
        # self.RIGHT_ANKLE_PITCH = 1
        # self.RIGHT_KNEE = 2
        # self.RIGHT_HIP_PITCH = 3
        # self.RIGHT_HIP_ROLL = 4
        # self.RIGHT_HIP_YALL = 5
        # self.LEFT_ANKLE_ROLL = 6
        # self.LEFT_ANKLE_PITCH = 7
        # self.LEFT_KNEE = 8
        # self.LEFT_HIP_PITCH = 9
        # self.LEFT_HIP_ROLL = 10
        # self.LEFT_HIP_YALL = 11
        # self.LEFT_ARM_PITCH = 12
        # self.LEFT_ARM_YALL = 13
        # self.LEFT_ARM_ROLL = 14
        # self.RIGHT_SHOULDER_PITCH = 15
        # self.RIGHT_ELBOW_YALL = 16
        # self.RIGHT_ARM_ROLL = 17



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
#
#
# '''
# +5.711e+0
# +3.495e+1
# -6.876e+1
# +3.381e+1
# +5.711e+0
# '''
