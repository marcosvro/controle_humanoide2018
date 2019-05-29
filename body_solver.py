# -*- coding:utf-8 -*-

import numpy as np
from k_solver.core import Actuator

GRAVITY_AC = 9.8


class Body():
    def __init__(self, angulos_braco=None):

        self.perna = 0
        self.angulosJuntas = [0] * 18
        self.lastSentAngles = [0] * 12
        self.pernaDirParaEsq = None
        self.pernaEsqParaDir = None

        # CALCULANDO CENTRO DE MASSA DA PARTE SUPERIOR QUE É CONSIDERADA STÁTICA DURANTE A CAMINHADA
        # ##########################################################################################
        if angulos_braco is None:
            angulos_braco = [0, 0, 0, 0, 0, 0]
        self.torsoCom = np.array([0, 0, +3.2125e-1])
        joints_pos = np.array([
            [+1.8050e-2, -3.4000e-2, +3.5668e-1],  # Right Shoulder Pitch
            [+1.8550e-2, -7.6400e-2, +3.4220e-1],  # Right Elbow Yall
            [+2.8550e-2, -7.6400e-2, +2.9520e-1]])  # Right Arm Roll
        coms_pos = np.array([
            [+1.8000e-2, -6.8450e-2, +3.5173e-1],  # Right Shoulder Pitch
            [+1.8353e-2, -7.5398e-2, +3.1635e-1],  # Right Elbow Yall
            [+2.8550e-2, -7.6400e-2, +2.5370e-1]])  # Right Arm Roll
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
            [+2.8353e-2, +7.6401e-2, +2.9520e-1]])  # Left Arm Roll
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
        self.degreeOfFreedom = [
            "x",  # LEFT_ANKLE_ROLL
            "y",  # LEFT_ANKLE_PITCH
            "y",  # LEFT_KNEE_PITCH
            "y",  # LEFT_HIP_PITCH
            "x",  # LEFT_HIP_ROLL
            "z",  # LEFT_HIP_YALL
            "z",  # RIGHT_HIP_YALL
            "x",  # RIGHT_HIP_ROLL
            "y",  # RIGHT_HIP_PITCH
            "y",  # RIGHT_KNEE_PITCH
            "y",  # RIGHT_ANKLE_PITCH
            "x"]  # RIGHT_ANKLE_ROLL
        # esquerda para direita (duas perna)
        joints_pos = np.array([
            [+1.8734e-2, -3.3164e-2, +4.3530e-2],  # LEFT_ANKLE_ROLL
            [+3.5700e-2, -3.5004e-2, +4.3526e-2],  # LEFT_ANKLE_PITCH
            [+3.5700e-2, -3.5004e-2, +1.4353e-1],  # LEFT_KNEE_PITCH
            [+3.5700e-2, -3.5003e-2, +2.4095e-1],  # LEFT_HIP_PITCH
            [+1.8885e-2, -3.2980e-2, +2.4090e-1],  # LEFT_HIP_ROLL
            [+1.6052e-2, -3.3005e-2, +2.8929e-1],  # LEFT_HIP_YALL
            [+1.6049e-2, +3.2993e-2, +2.8930e-1],  # RIGHT_HIP_YALL
            [+1.8885e-2, +3.2982e-2, +2.4090e-1],  # RIGHT_HIP_ROLL
            [+3.5700e-2, +3.5000e-2, +2.4095e-1],  # RIGHT_HIP_PITCH
            [+3.5700e-2, +3.5000e-2, +1.4353e-1],  # RIGHT_KNEE_PITCH
            [+3.5700e-2, +3.5000e-2, +4.3526e-2],  # RIGHT_ANKLE_PITCH
            [+1.8734e-2, +3.3167e-2, +4.3530e-2],  # RIGHT_ANKLE_ROLL
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
            1.000e-1]
        )

        v = []
        t = []
        v.append([0., 0., +3.8867e-2])
        # COM do primeiro link - Projeção da primeira junta no chão
        t.append(np.array([+1.8734e-2, -3.3163e-2, +3.3015e-2]) - np.array([+1.8734e-2, -3.3164e-2, 0.]))
        for i, c in enumerate(self.degreeOfFreedom):
            v.append(c)
            v.append(joints_pos[i + 1] - joints_pos[i])
            t.append(coms_pos[i + 1] - joints_pos[i])

            self.pernaDirParaEsq = Actuator(v, center_of_mass_shitfts=t, mass_parts=links_mass)

            joints_pos = np.delete(joints_pos, 12, 0)
            joints_pos = np.insert(joints_pos, 0, [+1.8734e-2, -3.3164e-2, 0.004663], 0)
            v = []
            t = []
            v.append([0., 0., +3.8867e-2])
            t.append(np.array([+1.8728e-2, +3.3167e-2, +3.3010e-2]) - np.array([+1.8734e-2, +3.3167e-2, 0.]))
            for i, c in enumerate(self.degreeOfFreedom):
                v.append(c)
                v.append(joints_pos[12 - (i + 1)] - joints_pos[12 - i])
                t.append(coms_pos[12 - (i + 1)] - joints_pos[12 - i])
            self.pernaEsqParaDir = Actuator(v, center_of_mass_shitfts=t, mass_parts=links_mass)

            print(self.pernaEsqParaDir.com())
            print(self.pernaDirParaEsq.com())

    # perna direita = 1, perna esquerda = 0
    # vec_bk é o vetor que indica a partir de qual junta o CoM está sendo calculado
    def get_torque_in_joint(self, perna_base, vec_bk=None):
        coms = self.pernaDirParaEsq.com(com_by_indices=vec_bk, return_mass=True) if perna_base \
            else self.pernaEsqParaDir.com(com_by_indices=vec_bk, return_mass=True)
        torques = []
        for i, tupla_com in enumerate(coms):
            torque = np.cross(tupla_com[0], np.array([0, 0, tupla_com[1] * GRAVITY_AC]))
            k = vec_bk[i] - 1 if vec_bk is not None else 0
            ek = None
            if self.degreeOfFreedom[k] == 'x':
                ek = np.array([-1, 0, 0])
            elif self.degreeOfFreedom[k] == 'y':
                ek = np.array([0, -1, 0])
            elif self.degreeOfFreedom[k] == 'z':
                ek = np.array([0, 0, -1])
            torque = np.sum(torque * ek)
            torques.append(torque)
        return torques


if __name__ == "__main__":
    a = Body()
    t = a.get_torque_in_joint(1, [5])
    print(t)

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
