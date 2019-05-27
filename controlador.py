# -*- coding:utf-8 -*-

import socket
import threading
import time
import os
import serial
import numpy as np
from functools import reduce
import struct
import csv

try:
    from std_msgs.msg import Float32MultiArray, Int8
except Exception as e:
    pass
import receiver

try:
    import rospy
except Exception as e:
    print("Falha ao importar a bibliotera 'rospy'!")
    print(e)
    raise e
import math

try:
    from Adafruit_BNO055 import BNO055
    import Adafruit_GPIO as AGPIO

    gpio = AGPIO.get_platform_gpio()
except Exception as e:
    print("Falha ao importar a bibliotera 'Adafruit_BNO055'!")

try:
    import RPi.GPIO as GPIO

    GPIO.setmode(GPIO.BCM)
    RASPBERRY = True
except Exception as e:
    RASPBERRY = False

RAD_TO_DEG = 180 / np.pi
DEG_TO_RAD = np.pi / 180.


def sigmoid_deslocada(x, periodo):
    return 1. / (1. + math.exp(-(12. / periodo) * (x - (periodo / 2.))))


class Controlador():

    def __init__(self,
                 simulador_enable=False,
                 time_id=17,
                 robo_id=0,
                 altura_inicial=17.,
                 tempo_passo=1.5,
                 deslocamento_ypelves=3.4,
                 deslocamento_zpes=2,
                 deslocamento_xpes=2.,
                 deslocamento_zpelves=30.,
                 inertial_foot_enable=False,
                 # step_mode=False,
                 nEstados=125):
        if (robo_id == -1):
            print("ERRO: ID do robo inválido")
            exit()

        self.simulador = simulador_enable
        # self.step_mode = step_mode
        self.state = 'IDDLE'
        self.TIME_ID = time_id
        self.ROBO_ID = robo_id
        self.ALTURA = altura_inicial
        self.posInicialPelves = [0., 1.7, altura_inicial]
        self.posInicialFoot = [0., 1.7, altura_inicial]
        self.deslocamentoXpes = 0.
        self.deslocamentoYpelves = 0
        self.deslocamentoZpes = 0
        self.deslocamentoZpelves = 0
        self.DESLOCAMENTO_X_PES_MAX = deslocamento_xpes
        self.DESLOCAMENTO_Z_PES_MAX = deslocamento_zpes
        self.DESLOCAMENTO_Y_PELVES_MAX = deslocamento_ypelves
        self.DESLOCAMENTO_Z_PELVES_MAX = deslocamento_zpelves

        self.N_ESTADOS = nEstados
        self.TEMPO_PASSO = tempo_passo
        # a e c: dimensoes da perna
        self.A = 10.5
        self.A = 10.2

        # self.visaoMsg = b''
        # self.msgFromMicro = []
        self.gravityCompensation = [0] * 20
        self.msgToMicro = [0] * 20
        # self.comParts = []
        self.fpsCount = 0
        self.lastTime = 0
        self.countFrames = 0
        self.timerFps = 0
        self.deltaTime = 0
        self.ON_PIN = 25
        if RASPBERRY:
            GPIO.setup(self.ON_PIN, GPIO.IN)

        self.VISAO_ATIVADA = False
        # self.microAtivado = False
        # self.inercialAtivado = False
        self.SIMULADOR_ATIVADO = True

        self.SIM_TRANS_RATE = self.TEMPO_PASSO / self.N_ESTADOS

        self.TEMPO_ACELERANDO = 4.
        self.TEMPO_MARCHANDO = 4.
        # self.TEMPO_VIRANDO = 3.

        # self.VISAO_SEARCH = False
        self.visaoBola = False
        self.turn90 = False
        self.MAX_YALL = 20
        self.MIN_YALL = 5
        # self.DIST_IDEAL = 10

        self.roboRoll = 0
        self.roboYall = 0
        self.roboYall = 0

        self.roboYall = 0
        self.roboPitch = -45

        #  quantos graus de diferença é permitida entre entre duas leituras consecutivas
        # self.LIMIAR_ERRO_INERCIAL = 20

        # self.gimbalYallLock = 0
        self.gimbalPitchLock = 0
        self.roboYallLock = 0
        self.roboPitchLock = 0
        self.LfootOrientation = [0, 0, 0]
        self.RfootOrientation = [0, 0, 0]
        self.INERTIAL_FOOT_ENABLE = inertial_foot_enable
        self.LfootPress = [0, 0, 0, 0]
        self.RfootPress = [0, 0, 0, 0]
        self.totalPress = 0

        self.currentStateTime = 0
        self.rotaDir = 0
        self.rotaEsq = 0
        self.ANGULO_VIRA = 3

        self.marchando = False
        self.recuando = False
        self.acelerando = False
        self.freando = False
        self.ladeando = False
        self.desladeando = False
        self.interpolando = False
        self.posicionando = False

        # Perna no chão: 1 = direita; 0 = esquerda
        self.perna = 0

        self.timerReposiciona = 0
        self.timerMovimentacao = 0

        self.rotDesvio = 0

        self.activate = True
        self.caiu = False


        self.RIGHT_ANKLE_ROLL = 0
        self.RIGHT_ANKLE_PITCH = 1
        self.RIGHT_KNEE = 2
        self.RIGHT_HIP_PITCH = 3
        self.RIGHT_HIP_ROLL = 4
        self.RIGHT_HIP_YALL = 5
        self.LEFT_ANKLE_ROLL = 6
        self.LEFT_ANKLE_PITCH = 7
        self.LEFT_KNEE = 8
        self.LEFT_HIP_PITCH = 9
        self.LEFT_HIP_ROLL = 10
        self.LEFT_HIP_YALL = 11
        self.LEFT_ARM_PITCH = 12
        self.LEFT_ARM_YALL = 13
        self.LEFT_ARM_ROLL = 14
        self.RIGHT_ARM_PITCH = 15
        self.RIGHT_ARM_YALL = 16
        self.RIGHT_ARM_ROLL = 17

        self.RST_IMU_PIN = 18

        try:
            with open('estados_levanta_frente.csv', newline='') as csvfile:
                tabela = list(csv.reader(csvfile, delimiter=','))
                tabela = np.array(tabela)
                self.ESTADOS_LEVANTA_FRENTE = tabela[1:, :]
                self.TEMPOS_LEVANTA_FRENTE = [4] * 19

            with open('estados_levanta_back.csv', newline='') as csvfile2:
                tabela = list(csv.reader(csvfile2, delimiter=','))
                tabela = np.array(tabela)
                self.ESTADOS_LEVANTA_COSTAS = tabela[1:, :]
                self.TEMPOS_LEVANTA_COSTAS = [4] * 19
        except Exception as e:
            self.ESTADOS_LEVANTA_COSTAS = []
            self.TEMPOS_LEVANTA_COSTAS = []
            self.ESTADOS_LEVANTA_FRENTE = []
            self.TEMPOS_LEVANTA_FRENTE = []

        if self.simulador:
            self.inicia_modulo_simulador()

    def inicia_modulo_juiz(self):
        "Iniciando moodulo juiz.."
        self.rec = receiver.GameStateReceiver(team=self.TIME_ID, player=self.ROBO_ID, control=self)
        t = threading.Thread(target=self.escuta_juiz)
        t.daemon = True
        t.start()

    def inicia_modulo_simulador(self):
        # INICIA PUBLISHER PARA ENVIAR POSIÇÕES DOS MOTORES
        print("Iniciando ROS node para execucao do simulador..")
        rospy.init_node('controller', anonymous=True)
        self.pub = rospy.Publisher('Bioloid/joint_pos', Float32MultiArray, queue_size=1)
        self.rate = rospy.Rate(self.TEMPO_PASSO / self.N_ESTADOS)
        t = threading.Thread(target=self.envia_para_simulador)
        t.daemon = True
        t.start()

        # INICIA PUBLISHER PARA ENVIAR INFORMAÇÃO DE QUAL PERNA ESTÁ NO CHÃO
        self.pubPerna = rospy.Publisher('Bioloid/support_leg', Int8, queue_size=1)
        thread_perna = threading.Thread(target = self.envia_perna)
        thread_perna.daemon = True
        thread_perna.start()

        # INICIA SUBSCRIBER PARA RECEBER DADOS DOS SENSORES INERCIAIS DOS PÉS
        rospy.Subscriber("/vrep_ros_interface/Bioloid/foot_inertial_sensor", Float32MultiArray,
                         self.foot_inertial_callback)

        # INICIA SUBSCRIBER PARA RECEBER DADOS DOS SENSORES DE PRESSÃO DOS PÉS
        rospy.Subscriber("/vrep_ros_interface/Bioloid/foot_pressure_sensor", Float32MultiArray,
                         self.foot_pressure_callback)

        # INICIA SUBSCRIBER PARA RECEBER DADOS DO SENSOR IMU DO ROBÔ
        rospy.Subscriber("/vrep_ros_interface/Bioloid/robot_inertial_sensor", Float32MultiArray,
                         self.robot_inertial_callback)

        # INICIA SUBSCRIBER PARA RECEBER COMANDOS DA VISÃO
        rospy.Subscriber("/Bioloid/visao_cmd", Float32MultiArray, self.visao_cmd_callback)

    def atualiza_fps(self):
        if self.timerFps >= 1:
            self.fpsCount = self.countFrames
            self.countFrames = 0
            self.timerFps = 0
            return self.fpsCount
        # self.deltaTime -> seconds
        self.deltaTime = time.time() - self.lastTime
        self.lastTime = time.time()
        self.countFrames += 1
        self.timerFps += self.deltaTime
        return None

    def escuta_juiz(self):
        print("Juiz OK!")
        try:
            self.rec.receive_forever()
        except Exception as e:
            print(str(e))
            self.inicia_modulo_juiz()

    def envia_para_simulador(self):
        try:
            print("Simulador OK!")
            # array contendo os angulos dos motores
            # mat.data[0]   = Right Ankle Roll
            # mat.data[1]   = Right Ankle Pitch
            # mat.data[2]   = Right Knee
            # mat.data[3]   = Right Hip Pitch
            # mat.data[4]   = Right Hip Roll
            # mat.data[5]   = Right Hip Yaw
            # mat.data[6]   = Left Ankle Roll
            # mat.data[7]   = Left Ankle Pitch
            # mat.data[8]   = Left Knee
            # mat.data[9]   = Left Hip Pitch
            # mat.data[10]  = Left Hip Roll
            # mat.data[11]  = Left Hip Yaw
            # mat.data[12]  = Left Arm Pitch
            # mat.data[13]  = Left Arm Yaw
            # mat.data[14]  = Left Arm Roll
            # mat.data[15]  = Right Arm Pitch
            # mat.data[16]  = Right Arm Yaw
            # mat.data[17]  = Right Arm Roll
            mat = Float32MultiArray()
            if self.SIMULADOR_ATIVADO:
                while not rospy.is_shutdown():
                    mat.data = self.msgToMicro[:18]

                    mat.data[10] = -mat.data[10]  # quadril esquerdo ROLL
                    mat.data[0] = -mat.data[0]  # calcanhar direito ROLL

                    mat.data[4] = -mat.data[4]
                    mat.data[10] = -mat.data[10]

                    self.pub.publish(mat)
                    rospy.sleep(self.SIM_TRANS_RATE)
        except Exception as e:
            print(e)
            pass

    def envia_perna(self):
        try:
            msg = Int8()
            if self.SIMULADOR_ATIVADO:
                while not rospy.is_shutdown():
                    msg.data = self.perna
                    self.pubPerna.publish(msg)
                    rospy.sleep(self.SIM_TRANS_RATE)
        except Exception as e:
            print(e)
            pass
    # '''
    # 	- descrição: função que recebe informações de onde está a bola,
    #     atualizando as variaveis globais referêntes ao gimbal
    #
    # 	- entrada: vetor "data" de 3 posições (sugeito a modificações, dependendo da lógica da visão)
    # 		data[0] = posição angular da bola no eixo pitch (y)
    # 		data[1] = posição angular da bola no eixo yall (z)
    # 		data[2] = flag que indica se está com a bola, usada para setar o
    #       estado do controle para IDDLE ou permitir que o robô ande
    # '''

    def visao_cmd_callback(self, msg):
        visao_msg = msg.data
        if self.roboYall + visao_msg[1] < 0:
            self.roboYall = self.roboYall + visao_msg[1] + 360
        elif self.roboYall + visao_msg[1] > 360:
            self.roboYall = (self.roboYall + visao_msg[1]) % 360
        else:
            self.roboYall = self.roboYall + visao_msg[1]
        self.roboPitch = visao_msg[0]
        self.visaoBola = visao_msg[2] != 0.

    # '''
    # 	- descrição: função que recebe dados do sensor inercial dos pés e atualiza as variaveis globais correspondentes.
    # 	- entrada: vetor "data" de 6 posições:
    # 		data [1:3] = orientação [x,y,z] do pé esquerdo
    # 		data [3:6] = orientação [x,y,z] do pé direito
    # '''
    #   Leitura IMU - pés
    def foot_inertial_callback(self, msg):
        self.LfootOrientation = np.array(msg.data[:3])
        self.RfootOrientation = np.array(msg.data[3:])

    # 	'''
    # 		- descrição: função que recebe dados do sensor de pressão dos pés e
    #         atualiza as variaveis globais correspondentes.
    # 		- entrada: vetor "data" de 8 posições:
    # 			data [1:4] = valores [p1,p2,p3,p4] que indicam o nivel de força
    #           detectados nos pontos na extremidade do pé esquerdo
    # 			data [4:8] = valores [p1,p2,p3,p4] que indicam o nivel de força
    #           detectados nos pontos na extremidade do pé direito
    # 	'''
    # 	Leitura sensores de pressão
    def foot_pressure_callback(self, msg):
        self.LfootPress = msg.data[:4]
        self.RfootPress = msg.data[4:]
        self.totalPress = np.sum(self.LfootPress) + np.sum(self.RfootPress)

    # 	Leitura IMU - robo
    def robot_inertial_callback(self, msg):
        self.roboYall = msg.data[2]
        self.roboYall = msg.data[1]
        self.roboRoll = msg.data[0]

        if (abs(self.roboPitch) > 45 or abs(self.roboRoll) > 45) and not self.interpolando:
            self.state = 'FALLEN'

        # 		'''
        # 		if self.roboYall > self.roboYall:
        # 			esq_angle = self.roboYall - self.roboYall
        # 			dir_angle = 360 - esq_angle
        # 		else:
        # 			dir_angle = self.roboYall - self.roboYall
        # 			esq_angle = 360 - dir_angle
        # 		if esq_angle > dir_angle:
        # 			self.roboYallLock = dir_angle
        # 		else:
        # 			self.roboYallLock = -esq_angle
        # 		'''

        if self.VISAO_ATIVADA:
            # manda mensagem para a rasp da visão dizendo o estado atual, a inclinação vertical e rotação horizontal
            try:
                self.visao_socket.send(
                    ("['" + self.state + "'," + str(self.roboYall) + ',' + str(self.roboYall) + ']').encode())
            except Exception as e:
                pass

    def classifica_estado(self):
        if self.state is 'IDDLE':
            if self.turn90:
                return 'MARCH'
            elif self.visaoBola:
                return 'MARCH'
            else:
                return -1
        elif self.state is 'TURN90':
            if abs(self.roboYallLock) <= self.MIN_YALL:
                return 'MARCH'
            else:
                return -1
        elif self.state is 'MARCH':
            if self.turn90:
                return 'TURN90'
            elif not self.visaoBola:
                return 'IDDLE'
            elif self.visaoBola and abs(self.roboYallLock) > self.MAX_YALL and self.VISAO_ATIVADA:
                return 'TURN'
            elif self.visaoBola and self.roboPitchLock > -45:
                return 'WALK'
            else:
                return -1
        elif self.state is 'WALK':
            if not self.visaoBola or abs(self.roboYallLock) > self.MAX_YALL or self.roboPitchLock <= -45:
                return 'MARCH'
            else:
                return -1
        elif self.state is 'TURN':
            if not self.visaoBola or abs(self.roboYallLock) < self.MIN_YALL:
                return 'MARCH'
            else:
                return -1
        else:
            print("ERRO: Estado invalido!!")

    # Define angulos para fazer o gimbal_lock
    def posiciona_robo(self):
        if self.roboYall > self.roboYall:
            esq_angle = self.roboYall - self.roboYall
            dir_angle = 360 - esq_angle
        else:
            dir_angle = self.roboYall - self.roboYall
            esq_angle = 360 - dir_angle
        if esq_angle > dir_angle:
            self.roboYallLock = dir_angle
        else:
            self.roboYallLock = -esq_angle

        self.roboPitchLock = self.roboPitch

    def run(self):
        # update function
        timer_main_loop = 0
        # perna direita (1) ou esquerda(0) no chão
        self.perna = 0
        # desvio para esquerda = 1 , desvio para direita = 2
        self.rotDesvio = 0
        while True:
            try:
                # print("%s GIMBAL_YALL:%.f  ROBO_YALL:%.2f  ANGULO PARA VIRAR:%.2f BOLA:%r" % (
                    # self.state, self.roboYall, self.roboYall, self.roboYallLock, self.visaoBola), flush=True)
                # print (np.array(self.RfootOrientation).astype(np.int), np.array(self.LfootOrientation).astype(np.int))
                if RASPBERRY:
                    # só executa se o dispositivo que estiver rodando for a raspberry
                    if GPIO.input(self.ON_PIN):
                        if not self.activate:
                            self.activate = True
                            gpio.set_low(self.RST_IMU_PIN)
                            time.sleep(1)
                            gpio.set_high(self.RST_IMU_PIN)
                    else:
                        self.activate = False
                        self.state = 'IDDLE'
                if self.state is 'FALLEN':
                    if not self.interpolando:
                        self.levanta()
                elif self.state is 'MARCH':
                    if self.deslocamentoYpelves != self.DESLOCAMENTO_Y_PELVES_MAX:
                        self.marchar()
                    elif self.deslocamentoXpes != 0:
                        self.freia_frente()
                    else:
                        novo_estado = self.classifica_estado()
                        if novo_estado != -1:
                            self.state = novo_estado
                elif self.state is 'IDDLE':
                    if self.rotaDir != 0 or self.rotaEsq != 0:
                        self.para_de_virar()
                    elif self.deslocamentoXpes != 0:
                        self.freia_frente()
                    elif self.deslocamentoYpelves != 0:
                        self.recuar()
                    else:
                        if self.activate:
                            novo_estado = self.classifica_estado()
                            if novo_estado != -1:
                                self.state = novo_estado
                elif self.state is 'WALK':
                    if self.deslocamentoXpes < self.DESLOCAMENTO_X_PES_MAX:
                        self.acelera_frente()
                    else:
                        novo_estado = self.classifica_estado()
                        if novo_estado != -1:
                            self.state = novo_estado
                elif self.state is 'TURN':
                    if abs(self.roboYallLock) > self.MIN_YALL:
                        self.vira()
                    elif self.rotaDir != 0 or self.rotaEsq != 0:
                        self.para_de_virar()
                    else:
                        novo_estado = self.classifica_estado()
                        if novo_estado != -1:
                            self.state = novo_estado
                elif self.state is 'TURN90':
                    if self.deslocamentoYpelves < self.DESLOCAMENTO_Y_PELVES_MAX:
                        self.marchar()
                    elif abs(self.roboYallLock) > self.MIN_YALL:
                        self.vira()
                    elif self.rotaDir != 0 or self.rotaEsq != 0:
                        self.para_de_virar()
                    else:
                        novo_estado = self.classifica_estado()
                        if novo_estado != -1:
                            self.turn90 = False
                            self.state = novo_estado
                elif self.state is 'UP':
                    if self.rotaDir != 0 or self.rotaEsq != 0:
                        self.para_de_virar()
                    elif self.deslocamentoXpes != 0:
                        self.freia_frente()
                    elif self.deslocamentoYpelves != 0:
                        self.recuar()
                    else:
                        # robo pronto para levantar
                        pass
                elif self.state is 'PENALIZED':
                    if self.rotaDir != 0 or self.rotaEsq != 0:
                        self.para_de_virar()
                    elif self.deslocamentoXpes != 0:
                        self.freia_frente()
                    elif self.deslocamentoYpelves != 0:
                        self.recuar()

                self.atualiza_fps()
                self.chage_state()
                self.atualiza_cinematica()
                # self.posiciona_gimbal()
                self.posiciona_robo()
                timer_main_loop += self.deltaTime
                time.sleep(self.SIM_TRANS_RATE)

            except KeyboardInterrupt as e:
                print("Main loop finalizado!!")
                break
            except Exception as e:
                raise e

    # Anda de lado para alinhar com o gol
    def posiciona(self):
        if not self.posicionando:
            self.posicionando = True
            self.timerReposiciona = 0
        if self.posicionando:
            self.timerReposiciona += self.deltaTime
            if self.roboYall > 270:
                # anda de lado para a esquerda
                if self.perna:
                    self.anda_de_lado_esquerda()
                else:
                    self.desanda_de_lado_esquerda()
            elif self.roboYall < 90:
                # anda de lado para a direita
                if not self.perna:
                    self.anda_de_lado_direita()
                else:
                    self.desanda_de_lado_direita()
        if self.timerReposiciona > self.TEMPO_PASSO * 6:
            if abs(self.posInicialPelves[1]) < 0.01:
                self.posInicialPelves[1] = 0
                self.posicionando = False
                self.state = 'IDDLE'
            else:
                if self.posInicialPelves[1] > 0 and not self.perna:
                    self.desanda_de_lado_esquerda()
                if self.posInicialPelves[1] < 0 and self.perna:
                    self.desanda_de_lado_direita()

    def levanta(self):
        if not self.interpolando:
            self.interpolando = True
            if self.roboYall > 0:
                t = threading.Thread(target=self.interpola_estados,
                                     args=[self.ESTADOS_LEVANTA_FRENTE, self.TEMPOS_LEVANTA_FRENTE])
                t.daemon = True
                t.start()
            else:
                t = threading.Thread(target=self.interpola_estados,
                                     args=[self.ESTADOS_LEVANTA_COSTAS, self.TEMPOS_LEVANTA_COSTAS])
                t.daemon = True
                t.start()

    def anda_de_lado_esquerda(self):
        if (not self.ladeando or self.desladeando) and self.posInicialPelves[1] != self.DESLOCAMENTO_Y_PELVES_MAX / 8:
            self.ladeando = True
            self.desladeando = False
            self.timerMovimentacao = 0
        if self.posInicialPelves[1] != self.DESLOCAMENTO_Y_PELVES_MAX / 8:
            self.timerMovimentacao += self.deltaTime
            self.posInicialPelves[1] = sigmoid_deslocada(self.timerMovimentacao,
                                                         self.TEMPO_PASSO) * self.DESLOCAMENTO_Y_PELVES_MAX / 8
        if abs(self.posInicialPelves[1] - self.DESLOCAMENTO_Y_PELVES_MAX / 8) <= 0.01:
            self.posInicialPelves[1] = self.DESLOCAMENTO_Y_PELVES_MAX / 8
            self.ladeando = False

    def desanda_de_lado_esquerda(self):
        if (not self.desladeando or self.ladeando) and self.posInicialPelves[1] != 0.:
            self.desladeando = True
            self.ladeando = False
            self.timerMovimentacao = 0
        if self.posInicialPelves[1] != 0.:
            self.timerMovimentacao += self.deltaTime
            self.posInicialPelves[1] = (1 - sigmoid_deslocada(self.timerMovimentacao,
                                                              self.TEMPO_PASSO)) * self.DESLOCAMENTO_Y_PELVES_MAX / 8
        if self.posInicialPelves[1] <= 0.01:
            self.posInicialPelves[1] = 0.
            self.desladeando = False

    def anda_de_lado_direita(self):
        if (not self.ladeando or self.desladeando) and self.posInicialPelves[1] != self.DESLOCAMENTO_Y_PELVES_MAX / 8:
            self.ladeando = True
            self.desladeando = False
            self.timerMovimentacao = 0
        if self.posInicialPelves[1] != self.DESLOCAMENTO_Y_PELVES_MAX / 8:
            self.timerMovimentacao += self.deltaTime
            self.posInicialPelves[1] = -sigmoid_deslocada(self.timerMovimentacao,
                                                          self.TEMPO_PASSO) * self.DESLOCAMENTO_Y_PELVES_MAX / 8
        if abs(self.posInicialPelves[1] - self.DESLOCAMENTO_Y_PELVES_MAX / 8) <= 0.01:
            self.posInicialPelves[1] = self.DESLOCAMENTO_Y_PELVES_MAX / 8
            self.ladeando = False

    def desanda_de_lado_direita(self):
        if (not self.desladeando or self.ladeando) and self.posInicialPelves[1] != 0.:
            self.desladeando = True
            self.ladeando = False
            self.timerMovimentacao = 0
        if self.posInicialPelves[1] != 0.:
            self.timerMovimentacao += self.deltaTime
            self.posInicialPelves[1] = (-1 + sigmoid_deslocada(self.timerMovimentacao,
                                                               self.TEMPO_PASSO)) * self.DESLOCAMENTO_Y_PELVES_MAX / 8
        if self.posInicialPelves[1] <= 0.01:
            self.posInicialPelves[1] = 0.
            self.desladeando = False

    # 	'''
    # 		- Define para qual lado o robô deve virar com base no yall lock
    # 	'''
    def vira(self):
        if self.roboYallLock < 0:
            self.rotDesvio = 1
        else:
            self.rotDesvio = -1

    # 	'''
    # 		- Vai parando de virar pelo tempo definido no construtor
    # 	'''
    def para_de_virar(self):
        self.rotDesvio = 0

    # 	'''
    # 		- Interpola distância de deslocamento dos pés, da atual até o max setado no contrutor
    # 	'''
    def acelera_frente(self):
        if not self.acelerando and self.deslocamentoXpes != self.DESLOCAMENTO_X_PES_MAX:
            self.acelerando = True
            self.timerMovimentacao = 0
        if self.deslocamentoXpes != self.DESLOCAMENTO_X_PES_MAX:
            self.timerMovimentacao += self.deltaTime
            self.deslocamentoXpes = sigmoid_deslocada(self.timerMovimentacao,
                                                      self.TEMPO_ACELERANDO) * self.DESLOCAMENTO_X_PES_MAX
        if abs(self.deslocamentoXpes - self.DESLOCAMENTO_X_PES_MAX) <= 0.01:
            self.deslocamentoXpes = self.DESLOCAMENTO_X_PES_MAX
            self.acelerando = False

    # 	'''
    # 		- Interpola distância de deslocamento dos pés, diminuindo este valor até que se torne 0
    # 	'''
    def freia_frente(self):
        if not self.freando and self.deslocamentoXpes != 0:
            self.freando = True
            self.timerMovimentacao = 0
        if self.deslocamentoXpes != 0:
            self.timerMovimentacao += self.deltaTime
            self.deslocamentoXpes = (1. - sigmoid_deslocada(self.timerMovimentacao,
                                                            self.TEMPO_ACELERANDO)) * self.DESLOCAMENTO_X_PES_MAX
        if self.deslocamentoXpes <= 0.01:
            self.deslocamentoXpes = 0
            self.freando = False

    # 	'''
    # 		- Interpola deslocamento lateral da pelves e o deslocamento para cima dos pés, da atual até o max
    # 	'''
    def marchar(self):
        if (not self.marchando) and self.deslocamentoYpelves != self.DESLOCAMENTO_Y_PELVES_MAX:
            self.marchando = True
            self.timerMovimentacao = 0
        if self.deslocamentoYpelves != self.DESLOCAMENTO_Y_PELVES_MAX:
            self.timerMovimentacao += self.deltaTime
            self.deslocamentoZpes = sigmoid_deslocada(self.timerMovimentacao,
                                                      self.TEMPO_MARCHANDO) * self.DESLOCAMENTO_Z_PES_MAX
            self.deslocamentoYpelves = sigmoid_deslocada(self.timerMovimentacao,
                                                         self.TEMPO_MARCHANDO) * self.DESLOCAMENTO_Y_PELVES_MAX
        if abs(self.deslocamentoYpelves - self.DESLOCAMENTO_Y_PELVES_MAX) <= 0.01:
            self.deslocamentoZpes = self.DESLOCAMENTO_Z_PES_MAX
            self.deslocamentoYpelves = self.DESLOCAMENTO_Y_PELVES_MAX
            self.marchando = False

    # 	'''
    # 		- Interpola deslocamento lateral da pelves e o deslocamento para cima dos pés,
    #           diminuindo estes valores até chegar em 0
    # 	'''
    def recuar(self):
        if not self.recuando and self.deslocamentoYpelves != 0:
            self.recuando = True
            self.timerMovimentacao = 0
        if self.deslocamentoYpelves != 0:
            self.timerMovimentacao += self.deltaTime
            self.deslocamentoZpes = (1. - sigmoid_deslocada(self.timerMovimentacao,
                                                            self.TEMPO_MARCHANDO)) * self.DESLOCAMENTO_Z_PES_MAX
            self.deslocamentoYpelves = (1. - sigmoid_deslocada(self.timerMovimentacao,
                                                               self.TEMPO_MARCHANDO)) * self.DESLOCAMENTO_Y_PELVES_MAX
        if self.deslocamentoYpelves <= 0.01:
            self.deslocamentoZpes = 0
            self.deslocamentoYpelves = 0
            self.recuando = False

    # Change state
    def chage_state(self):
        # incrementa currentStateTime até tempoPasso (até trocar voltar à fase de suporte duplo)
        self.currentStateTime += self.deltaTime
        if self.currentStateTime >= self.TEMPO_PASSO:
            self.currentStateTime = 0
            # indica se é a perna direita (1) ou esquerda(0) no chão
            self.perna = (self.perna + 1) % 2
            if self.rotDesvio != 0:
                if self.rotDesvio > 0:
                    if self.perna:
                        self.rotaDir = -1
                        self.rotaEsq *= 2
                    else:
                        self.rotaEsq = -1
                        self.rotaDir *= 2
                else:
                    if self.perna:
                        self.rotaDir = 1
                        self.rotaEsq *= 2
                    else:
                        self.rotaEsq = 1
                        self.rotaDir *= 2
            else:
                if math.fabs(self.rotaEsq) == 2:
                    self.rotaEsq = 0
                elif math.fabs(self.rotaEsq) == 1:
                    self.rotaEsq *= 2
                if math.fabs(self.rotaDir) == 2:
                    self.rotaDir = 0
                elif math.fabs(self.rotaDir) == 1:
                    self.rotaDir *= 2

    # 	'''
    # 		- Retorna os 6 angulos de da perna, calculando a cinematica inversa.
    #           Considerando o pé como base e o quadril como ponto variável
    # 	'''

    def foot_to_hip(self, point_hip):
        angulos = []
        x, y, z = point_hip

        # ankle roll
        theta = math.atan(y / z)
        angulos.append(theta)

        # ankle pitch
        b = math.sqrt(x ** 2 + y ** 2 + z ** 2)
        angulo_a = math.acos((self.A ** 2 - (b ** 2 + self.A ** 2)) / (-2 * b * self.A))
        betha = math.atan(x / z)
        angulo_a = betha + angulo_a
        angulos.append(angulo_a)

        # knee
        angulo_b = math.acos((b ** 2 - (self.A ** 2 + self.A ** 2)) / (-2 * self.A * self.A))
        angulo_b = angulo_b - math.pi
        angulos.append(angulo_b)

        # hip pitch
        angulo_c = math.acos((self.A ** 2 - (self.A ** 2 + b ** 2)) / (-2 * self.A * b))
        angulo_c = angulo_c - betha
        angulos.append(angulo_c)

        # hip roll
        angulos.append(theta)

        # hip yall
        angulos.append(0)

        return angulos

    # 	'''
    # 		- Pega o proximo "estado" da função de trajetória, a função de trajetória muda
    #         de acordo com as variaveis que definem o deslocamento e rotação do robô

    # 		Entrada: tempo float/int t
    # 		Saída: 2 vetores de 3 posições (x,y,z). O primeiro indica a posição da pelves
    #              considerando o pé em contato com o chão como base,
    # 			   o segundo vetor indica a posição do pé de balanço considerando a pelves do pé de balanço como base.
    # 	'''

    def getTragectoryPoint(self, x, aux_estados):
        pos_pelves = self.posInicialPelves[:]

        # nEstados * [-0.5,0.5]
        # aux_estados = (x-self.N_ESTADOS/2)

        aux_estados_div_25 = aux_estados / 25
        aux_pelves = self.deslocamentoYpelves * math.sin(x * math.pi / self.N_ESTADOS)

        # deslocamentoXpes/2 * tgh(x)
        p1 = (self.deslocamentoXpes / 2) * ((math.exp(aux_estados_div_25) - math.exp(-aux_estados_div_25)) / (
            math.exp(aux_estados_div_25) + math.exp(-aux_estados_div_25)))
        pos_pelves[0] = p1
        pos_pelves[1] -= aux_pelves

        pos_foot = self.posInicialPelves[:]
        p2 = - p1
        pos_foot[0] = p2
        pos_foot[1] += aux_pelves
        pos_foot[2] = self.ALTURA - self.deslocamentoZpes * math.exp(-(aux_estados ** 2) / 600)
        return pos_pelves, pos_foot

    # interpolação simples entre estados
    def interpola_estados(self, estados, tempos):
        if len(estados) > 0:
            p_ant = estados[0]
        else:
            return
        for i in range(1, len(estados)):
            p_atual = estados[i]
            t = tempos[i - 1]
            timer = 0
            m = []
            for j in range(len(p_atual)):
                m.append((p_ant[j] - p_atual[j]) / (0 - t))

            while timer < t:
                timer += self.deltaTime
                for j in range(len(p_atual)):
                    self.msgToMicro[j] = m[j] * timer + p_ant[j]
        self.state = 'IDDLE'
        self.interpolando = False

    def atualiza_cinematica(self):
        # currentStateTime = segundos desde o inicio do passo
        # de onde veio esse 125 ?
        x = (self.currentStateTime * 125) / self.TEMPO_PASSO

        aux_estados = (x - self.N_ESTADOS / 2)
        pelv_point, foot_point = self.getTragectoryPoint(x, aux_estados)

        # ANGULO_VIRA * tgh(2*(nEstados)/50)
        # ANGULO_VIRA * tgh(x-nEstados/2)/25
        angulo_vira_x_tgh_div_4 = self.ANGULO_VIRA / 2. * ((np.exp(aux_estados / 25) - np.exp(aux_estados / -25)) / (
            np.exp(aux_estados / 25) + np.exp(aux_estados / -25)))
        # CINEMÁTICA INVERSA
        data_pelv = self.foot_to_hip(pelv_point)
        data_foot = self.foot_to_hip(foot_point)
        if self.perna:
            # CONTROLE PÉ SUSPENSO

            if self.INERTIAL_FOOT_ENABLE:
                if self.totalPress == 0:
                    influencia = 0
                else:
                    influencia = np.sum(self.LfootPress) / self.totalPress
                data_foot[:2] = np.array(data_foot[:2]) + np.array(self.LfootOrientation[:2]) * DEG_TO_RAD * (
                    1 - influencia)

            # ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A ESQUERDA
            if self.rotaDir == 1:
                data_pelv[5] = (self.ANGULO_VIRA / 2. + angulo_vira_x_tgh_div_4) * DEG_TO_RAD
            elif self.rotaDir == -1:
                data_pelv[5] = (-self.ANGULO_VIRA / 2. - angulo_vira_x_tgh_div_4) * DEG_TO_RAD
            else:
                data_pelv[5] = 0

            # ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A DIREITA
            if self.rotaEsq == 2:
                # data_foot[5] = self.ANGULO_VIRA - (self.ANGULO_VIRA/2. + angulo_vira_x_tgh_div_4)
                # data_foot[5] = self.ANGULO_VIRA - self.ANGULO_VIRA/2. - angulo_vira_x_tgh_div_4
                # data_foot[5] = self.ANGULO_VIRA/2 - angulo_vira_x_tgh_div_4
                # data_foot[5] = data_foot[5] * math.pi/180.
                data_foot[5] = (self.ANGULO_VIRA / 2 - angulo_vira_x_tgh_div_4) * DEG_TO_RAD
            elif self.rotaEsq == -2:
                # data_foot[5] = -self.ANGULO_VIRA - (-self.ANGULO_VIRA/2. - angulo_vira_x_tgh_div_4)
                # data_foot[5] = -self.ANGULO_VIRA + self.ANGULO_VIRA/2. + angulo_vira_x_tgh_div_4
                # data_foot[5] = -self.ANGULO_VIRA/2 + angulo_vira_x_tgh_div_4
                # data_foot[5] = data_foot[5] * math.pi/180.
                data_foot[5] = (-self.ANGULO_VIRA / 2 + angulo_vira_x_tgh_div_4) * DEG_TO_RAD
            else:
                data_foot[5] = 0

            # PÉ DIREITO ESTÁ EM CONTATO COM O CHÃO E PÉ ESQUERDO ESTÁ SE MOVENDO.
            data = data_pelv + data_foot + [0] * 6
        else:

            # CONTROLE PÉ SUSPENSO
            if self.INERTIAL_FOOT_ENABLE:
                if self.totalPress == 0:
                    influencia = 0
                else:
                    influencia = np.sum(self.RfootPress) / self.totalPress
                data_foot[:2] = np.array(data_foot[:2]) + np.array(self.RfootOrientation[:2]) * DEG_TO_RAD * (
                    1 - influencia)

            # ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A ESQUERDA
            if self.rotaEsq == 1:
                data_pelv[5] = (self.ANGULO_VIRA / 2. + angulo_vira_x_tgh_div_4) * DEG_TO_RAD
            elif self.rotaEsq == -1:
                data_pelv[5] = (-self.ANGULO_VIRA / 2. - angulo_vira_x_tgh_div_4) * DEG_TO_RAD
            else:
                data_pelv[5] = 0

            # ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A DIREITA
            if self.rotaDir == 2:
                # data_foot[5] =  self.ANGULO_VIRA - (self.ANGULO_VIRA/2. + angulo_vira_x_tgh_div_4)
                # data_foot[5] =  self.ANGULO_VIRA - self.ANGULO_VIRA/2 - angulo_vira_x_tgh_div_4
                # data_foot[5] =  self.ANGULO_VIRA/2 - angulo_vira_x_tgh_div_4
                # data_foot[5] = data_foot[5] * math.pi/180.
                data_foot[5] = (self.ANGULO_VIRA / 2 - angulo_vira_x_tgh_div_4) * DEG_TO_RAD
            elif self.rotaDir == -2:
                # data_foot[5] = -self.ANGULO_VIRA - (-self.ANGULO_VIRA/2. - angulo_vira_x_tgh_div_4)
                # data_foot[5] = -self.ANGULO_VIRA + self.ANGULO_VIRA/2. + angulo_vira_x_tgh_div_4
                # data_foot[5] = -self.ANGULO_VIRA/2. + angulo_vira_x_tgh_div_4
                # data_foot[5] = data_foot[5] * math.pi/180.
                data_foot[5] = (-self.ANGULO_VIRA / 2. + angulo_vira_x_tgh_div_4) * DEG_TO_RAD
            else:
                data_foot[5] = 0

            # PÉ ESQUERDO ESTÁ EM CONTATO COM O CHÃO E PÉ DIREITO ESTÁ SE MOVENDO.
            data = data_foot + data_pelv + [0] * 6
            
        self.msgToMicro[:18] = data

    # def atualiza_compensador_gravitacional(self, msg):
        
        # LEFT_ANKLE_ROLL
        # LEFT_ANKLE_PITCH
        # LEFT_KNEE
        # LEFT_HIP_PITCH
        # LEFT_HIP_ROLL
        # LEFT_HIP_YALL
        # RIGHT_ANKLE_ROLL
        # RIGHT_ANKLE_PITCH
        # RIGHT_KNEE
        # RIGHT_HIP_PITCH
        # RIGHT_HIP_ROLL
        # RIGHT_HIP_YALL




# '''
# 	- descrição: Calcula posição do centro de massa em relação ao pé que está em contato com o chão
#
# def centro_de_massa(self, ith_joint=0):
# 	if (ith_joint != 0): #calcula centro de massa a partir da junta ith_joint
# 		if self.perna: #pé direito no chão e será a perna de referência
#
# 		else:
#
# '''


if __name__ == '__main__':
    control = Controlador(time_id=17, robo_id=0, simulador_enable=True, inertial_foot_enable=True)
    control.run()
