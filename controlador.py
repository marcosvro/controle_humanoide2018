# -*- coding:utf-8 -*-
import socket
import threading
import time
import os
import numpy as np
import matplotlib.pyplot as plt
from functools import reduce
import struct
import csv
try:
	from std_msgs.msg import Float32MultiArray, Int16MultiArray, Int8
except Exception as e:
	print("Falha ao importar a bibliotera 'std_msgs.msg'!")
try:
	import rclpy
	from rclpy.node import Node
except Exception as e:
	print("Falha ao importar a bibliotera 'rospy'!")
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
from body_solver import Body
from enum import Enum

RAD_TO_DEG = 180 / np.pi
DEG_TO_RAD = np.pi / 180.

KP_CONST = 0.3


def sigmoid_deslocada(x, periodo, inc=12.):
	return 1./(1.+math.exp(-(inc/periodo)*(x-(periodo/2.))))

class OrientationMode(Enum):
    VISAO = 1
    POSICAO_ALVO = 2

class Controlador():

	def __init__(self,
				simulador_enable=False,
				time_id=17,
				robo_id=0,
				altura_inicial=17.,
				tempo_passo = 0.4, # 0.4
				deslocamento_ypelves = 3.5, # 3.5
				deslocamento_zpes = 3., # 3.
				deslocamento_xpes= 1.5, # 1.5
				deslocamento_zpelves = 30.,
				inertial_foot_enable = False,
				gravity_compensation_enable = False,
				step_mode=False,
				orientation_mode=OrientationMode.POSICAO_ALVO):
		if (robo_id == -1):
			print("ERRO: ID do robo inválido")
			exit()


		self.simulador = simulador_enable
		self.step_mode = step_mode
		self.orientation_mode = orientation_mode
		self.state = 'IDDLE'
		self.time_id = time_id
		self.robo_id = robo_id
		self.altura = altura_inicial
		self.pos_inicial_pelves = [0., 1.4, altura_inicial]
		self.pos_inicial_foot = [0., 1.4, altura_inicial]
		self.deslocamentoXpes = 0.
		self.deslocamentoYpelves = 0
		self.deslocamentoZpes = 0
		self.deslocamentoZpelves = 0
		self.deslocamentoXpesMAX = deslocamento_xpes
		self.deslocamentoZpesMAX = deslocamento_zpes
		self.deslocamentoYpelvesMAX = deslocamento_ypelves
		self.deslocamentoZpelvesMAX = deslocamento_zpelves

		self.nEstados = 125
		self.tempoPasso = tempo_passo
		self.a = 10.5
		self.c = 10.2

		self.visao_msg = b''
		self.msg_from_micro = []
		self.msg_to_micro = [0]*20
		self.CoM_parts = []
		self.fps_count = 0
		self.last_time = 0
		self.count_frames = 0
		self.timer_fps = 0
		self.deltaTime = 0
		self.time_ignore_GC = 0.1 #entre 0 e 1 - porcentagem de tempo para ignorar o gravity compensation
		self.ON_PIN = 25
		if RASPBERRY:
			GPIO.setup(self.ON_PIN, GPIO.IN)

		self.visao_ativada = True
		self.micro_ativado = False
		self.incercial_ativado = False
		self.simulador_ativado = True

		self.simTransRate = 1/self.nEstados*self.tempoPasso

		self.tempo_acelerando = 4.
		self.tempo_marchando = 4.
		self.tempo_virando = 3.

		self.visao_search = False
		self.chegou_no_alvo = True
		self.turn90 = False
		self.max_yall = 20
		self.min_yall = 5
		self.dist_ideal = 10

		self.robo_roll = 0
		self.robo_yall = 0
		self.robo_pitch = 0

		self.gimbal_yall = 0
		self.gimbal_pitch = -45

		self.limiar_erro_inercial = 20 # quantos graus de diferença é permitida entre entre duas leituras consecutivas

		self.gimbal_yall_lock = 0
		self.gimbal_pitch_lock = 0
		self.robo_yall_lock = 0
		self.robo_pitch_lock = 0
		self.Lfoot_orientation = [0,0,0]
		self.Rfoot_orientation = [0,0,0]
		self.inertial_foot_enable = inertial_foot_enable
		self.gravity_compensation_enable = gravity_compensation_enable
		self.Lfoot_press = [0,0,0,0]
		self.Rfoot_press = [0,0,0,0]
		self.total_press = 0

		self.t_state = 0
		self.rota_dir = 0
		self.rota_esq = 0
		self.angulo_vira = 3

		self.marchando = False
		self.recuando = False
		self.acelerando = False
		self.freando = False
		self.interpolando = False

		# Perna no chão: 1 = direita; 0 = esquerda
		self.perna = 0

		self.timerReposiciona = 0
		self.timerMovimentacao = 0

		self.rotDesvio = 0

		self.activate = True
		self.caiu = False

		self.rst_imu_pin = 18

		self.body = Body()
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

		self.state_encoder = {
			"IDDLE" : 1,
			"MARCH" : 2,
			"WALK"  : 3,
			"TURN"  : 4,
			"FALLEN": 5,
			"UP"    : 6,
			"PENALIZED": 7,
			"TURN90": 8
		}

		try:
			with open ('estados_levanta_frente.csv', newline='') as csvfile:
				tabela = list(csv.reader(csvfile, delimiter=','))
				tabela = np.array(tabela)
				self.estados_levanta_frente = tabela[1:, :]
				self.tempos_levanta_frente = [4]*19

			with open('estados_levanta_back.csv', newline='') as csvfile2:
				tabela = list(csv.reader(csvfile2, delimiter=','))
				tabela = np.array(tabela)
				self.estados_levanta_costas = tabela[1:, :]
				self.tempos_levanta_costas = [4]*19
		except Exception as e:
			self.estados_levanta_costas = []
			self.tempos_levanta_costas = []
			self.estados_levanta_frente = []
			self.tempos_levanta_frente = []

		#if self.simulador:
		self.setup_ros()

		self.posicao_alvo = [0, 0, 0]
		self.posicao_robo = [0, 0, 0]


	def setup_ros(self):
		#INICIA PUBLISHER PARA ENVIAR POSIÇÕES DOS MOTORES
		rclpy.init()
		node = Node('controller')

		if self.simulador:
			print("Iniciando ROS node para execucao do simulador")
			self.pub = node.create_publisher(Float32MultiArray, 'Bioloid/joint_pos', 1)
		else:
			print("Iniciando ROS node para execucao do micro-controlador")
			self.pub = node.create_publisher(Int16MultiArray, 'Bioloid/joint_pos', 1)

		self.rate = node.create_rate(1/(self.tempoPasso/self.nEstados))
		self.spin_t = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
		self.spin_t.start()

		if self.simulador:
			self.sim_t = threading.Thread(target=self.envia_para_simulador)
		else:
			self.sim_t = threading.Thread(target=self.envia_para_micro)
		self.sim_t.daemon = True
		self.sim_t.start()

		if self.simulador:
			#INICIA SUBSCRIBER PARA RECEBER DADOS DOS SENSORES INERCIAIS DOS PÉS
			node.create_subscription(Float32MultiArray, "/Bioloid/foot_inertial_sensor", self.foot_inertial_callback, 1)

			#INICIA SUBSCRIBER PARA RECEBER DADOS DOS SENSORES DE PRESSÃO DOS PÉS
			node.create_subscription(Float32MultiArray, "/Bioloid/foot_pressure_sensor", self.foot_pressure_callback, 1)

			#INICIA SUBSCRIBER PARA RECEBER DADOS DA POSIÇÃO DO ROBÔ
			node.create_subscription(Float32MultiArray, "/Bioloid/robot_position", self.robot_position_callback, 1)

			#INICIA SUBSCRIBER PARA RECEBER DADOS DA POSIÇÃO ALVO
			node.create_subscription(Float32MultiArray, "/Bioloid/target_position", self.target_position_callback, 1)

		#INICIA SUBSCRIBER PARA RECEBER DADOS DO SENSOR IMU DO ROBÔ
		node.create_subscription(Float32MultiArray, "/Bioloid/robot_inertial_sensor", self.robot_inertial_callback, 1)

		#INICIA SUBSCRIBER PARA RECEBER COMANDOS DA VISÃO
		node.create_subscription(Float32MultiArray, "/Bioloid/visao_cmd", self.visao_cmd_callback, 1)

		#INICIA SUBSCRIBER PARA RECEBER COMANDOS ESTADO DO ROBÔ
		node.create_subscription(Int8, "/Bioloid/state_cmd", self.state_cmd_callback, 1)

	def atualiza_fps(self):
		if self.timer_fps >= 1:
			self.fps_count = self.count_frames
			self.count_frames = 0
			self.timer_fps = 0
			return self.fps_count
		self.deltaTime = time.time() - self.last_time
		self.last_time = time.time()
		self.count_frames += 1
		self.timer_fps += self.deltaTime
		return None
	
	def calcula_centro_pressao(self):
		l_foot_y = 11
		l_foot_x = 6.2

		LD = [-l_foot_x/2., -l_foot_y/2.]
		LT = [-l_foot_x/2., l_foot_y/2.]
		RD = [l_foot_x/2., -l_foot_y/2.]
		RT = [l_foot_x/2., l_foot_y/2.]

		div_l = np.max(self.Lfoot_press)-np.min(self.Lfoot_press)
		div_r = np.max(self.Rfoot_press)-np.min(self.Rfoot_press)
		div_l = div_l if div_l != 0.0 else 0.00001
		div_r = div_r if div_r != 0.0 else 0.00001
		l_press_weights_norm = (self.Lfoot_press-np.min(self.Lfoot_press))/div_l
		r_press_weights_norm = (self.Rfoot_press-np.min(self.Rfoot_press))/div_r

		l_press_vectors = np.array([LD, LT, RD, RT]) * np.array(l_press_weights_norm)[:, np.newaxis]
		self.l_center_of_press = np.sum(l_press_vectors, axis=0)

		r_press_vectors = np.array([LD, LT, RD, RT]) * np.array(r_press_weights_norm)[:, np.newaxis]
		self.r_center_of_press = np.sum(r_press_vectors, axis=0)

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
			if self.simulador_ativado:
				while rclpy.ok():
					mat.data = np.array(self.msg_to_micro[:18]).astype(np.float32).tolist()

					# mat.data[10] = -mat.data[10] # quadril esquerdo ROLL
					mat.data[0] = -mat.data[0] #calcanhar direito ROLL

					mat.data[4] = -mat.data[4]
					# mat.data[10] = -mat.data[10]

					self.pub.publish(mat)
					self.rate.sleep()
		except Exception as e:
			pass

	def envia_para_micro(self):
		try:
			print("Publicando no topico para o micro!!")
			mat = Int16MultiArray()
			self.simulador_ativado = True
			while rclpy.ok():
				data = (np.array(self.msg_to_micro[:19])*(1800/np.pi)).astype(np.int16).tolist()
				data[18] = self.state_encoder[self.state]
				mat.data = data

				mat.data[0] = -mat.data[0] #calcanhar direito ROLL

				mat.data[4] = -mat.data[4]

				mat.data[self.RIGHT_HIP_PITCH] += 150
				mat.data[self.LEFT_HIP_PITCH] += 150

				self.pub.publish(mat)
				self.rate.sleep()
		except Exception as e:
			raise e

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
		if self.robo_yall + visao_msg[1] < 0:
			self.gimbal_yall = self.robo_yall + visao_msg[1] + 360
		elif self.robo_yall + visao_msg[1] > 360:
			self.gimbal_yall = (self.robo_yall + visao_msg[1])% 360
		else:
			self.gimbal_yall = self.robo_yall + visao_msg[1]
		self.gimbal_pitch = visao_msg[0]
		self.chegou_no_alvo = visao_msg[2] == 0.


	# '''
	# 	- descrição: função que recebe próximo estado do robô,
	#     forçando a atualização do estado atual para o estado fornecido.
	#
	# 	- entrada: inteiro "data" com o codigo para o próximo estado
	# '''

	def state_cmd_callback(self, msg):
		state_decoder = {
			1 : "IDDLE",
			2 : "MARCH",
			3 : "WALK",
			5 : "FALLEN",
			6 : "UP", 
			7 : "PENALIZED",
			8 : "TURN_R",
			9 : "TURN_L"
		}
		state_msg = state_decoder[msg.data]
		if state_msg == "IDDLE":
			self.visao_cmd_callback(Float32MultiArray(data = [-45, 0, 0]))
			self.state = "IDDLE"
		elif state_msg == "MARCH":
			self.visao_cmd_callback(Float32MultiArray(data = [-45, 0, 1]))
		elif state_msg == "WALK":
			self.visao_cmd_callback(Float32MultiArray(data = [0, 0, 1]))
		elif state_msg == "TURN_R":
			self.visao_cmd_callback(Float32MultiArray(data = [0, 90, 1]))
		elif state_msg == "TURN_L":
			self.visao_cmd_callback(Float32MultiArray(data = [0, -90, 1]))		


	# '''
	# 	- descrição: função que recebe dados do sensor inercial dos pés e atualiza as variaveis globais correspondentes.
	# 	- entrada: vetor "data" de 6 posições:
	# 		data [1:3] = orientação [x,y,z] do pé esquerdo
	# 		data [3:6] = orientação [x,y,z] do pé direito
	# '''
	#   Leitura IMU - pés
	def foot_inertial_callback(self, msg):
		self.Lfoot_orientation = np.array(msg.data[:3])
		self.Rfoot_orientation = np.array(msg.data[3:])

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
		self.Lfoot_press = [(v if v != np.nan else 0.000001) for v in msg.data[:4]]
		self.Rfoot_press = [(v if v != np.nan else 0.000001) for v in msg.data[4:]]
		self.total_press = np.sum(self.Lfoot_press)+np.sum(self.Rfoot_press)

	# 	Leitura IMU - robo
	def robot_inertial_callback(self, msg):
		self.robo_yall = msg.data[2]
		self.robo_pitch = msg.data[1]
		self.robo_roll = msg.data[0]
		print(self.robo_yall)
		if (abs(self.robo_pitch) > 45 or abs(self.robo_roll) > 45) and not self.interpolando:
			self.state = 'FALLEN'

	def robot_position_callback(self, msg):
		self.posicao_robo = msg.data[:3]

	def target_position_callback(self, msg):
		self.posicao_alvo = msg.data[:3]

	def classifica_estado(self):
		if self.state == 'IDDLE':
			if self.turn90:
				return 'MARCH'
			elif not self.chegou_no_alvo:
				return 'MARCH'
			else:
				return -1
		elif self.state == 'TURN90':
			if abs(self.robo_yall_lock) <= self.min_yall:
				return 'MARCH'
			else:
				return -1
		elif self.state == 'MARCH':
			if self.turn90:
				return 'TURN90'
			elif self.chegou_no_alvo:
				return 'IDDLE'
			elif not self.chegou_no_alvo and abs(self.robo_yall_lock) > self.max_yall and self.visao_ativada:
				return 'TURN'
			elif not self.chegou_no_alvo and self.robo_pitch_lock > -45:
				return 'WALK'
			else:
				return -1
		elif self.state == 'WALK':
			if self.chegou_no_alvo or abs(self.robo_yall_lock) > self.max_yall or self.robo_pitch_lock <= -45:
				return 'MARCH'
			else:
				return -1
		elif self.state == 'TURN':
			if self.chegou_no_alvo or abs(self.robo_yall_lock) < self.min_yall:
				return 'MARCH'
			else:
				return -1
		else:
			print("ERRO: Estado invalido!!")


	def gravity_compensation(self):
		#return
		if not self.gravity_compensation_enable or (self.t_state < self.tempoPasso/2 and self.t_state < self.tempoPasso*self.time_ignore_GC) or (self.t_state >= self.tempoPasso/2 and self.t_state > self.tempoPasso*(1-self.time_ignore_GC)) or (self.state == "IDDLE" and not self.recuando and not self.freando):
			return

		torques = self.body.get_torque_in_joint(self.perna,[3,5])

		dQ = (np.array(torques)/KP_CONST)/15
		#print(dQ[1], self.perna)

		dQ *= math.sin(self.t_state*math.pi/self.tempoPasso)
		dQ *= (self.deslocamentoZpes / self.deslocamentoZpesMAX)

		if self.perna:
			self.msg_to_micro[self.RIGHT_ANKLE_PITCH] += dQ[0]
			self.msg_to_micro[self.RIGHT_HIP_ROLL] += (dQ[1]*-1)
		else:
			self.msg_to_micro[self.LEFT_KNEE] += dQ[0]
			self.msg_to_micro[self.LEFT_HIP_ROLL] += dQ[1]

	def compura_direcao_pelo_gimbal(self):
		if self.robo_yall > self.gimbal_yall:
			esq_angle = self.robo_yall - self.gimbal_yall
			dir_angle = 360 - esq_angle
		else:
			dir_angle = self.gimbal_yall - self.robo_yall
			esq_angle = 360 - dir_angle
		if esq_angle > dir_angle:
			self.robo_yall_lock = dir_angle
		else:
			self.robo_yall_lock = -esq_angle

		self.robo_pitch_lock = self.gimbal_pitch

	def computa_direcao_pela_posicao_alvo(self):
		robo_para_ponto_alvo = np.array(self.posicao_alvo[:2]) - np.array(self.posicao_robo[:2])
		theta = math.atan2(robo_para_ponto_alvo[1], robo_para_ponto_alvo[0]) * (180/math.pi) - self.robo_yall
		if theta < -180:
			theta += 360
		elif theta > 180:
			theta -= 360
		self.robo_yall_lock = -theta

		distancia_ponto_alvo = math.sqrt((self.posicao_alvo[0] - self.posicao_robo[0])**2 + (self.posicao_alvo[1] - self.posicao_robo[1])**2)
		if (distancia_ponto_alvo > 0.2):
			self.chegou_no_alvo = False
			self.robo_pitch_lock = 0
		else:
			self.chegou_no_alvo = True
			self.robo_pitch_lock = -45

	def run(self):
		#update function
		timer_main_loop = 0
		# perna direita (1) ou esquerda(0) no chão
		self.perna = 0
		self.rot_desvio = 0
		while (True):
			try:
				if RASPBERRY:
					# só executa se o dispositivo que estiver rodando for a raspberry
					if GPIO.input(self.ON_PIN):
						if not self.activate:
							self.activate = True
							gpio.set_low(self.rst_imu_pin)
							time.sleep(1)
							gpio.set_high(self.rst_imu_pin)
					else:
						self.activate = False
						self.state = 'IDDLE'
				if (self.state == 'FALLEN'):
					if not self.interpolando:
						self.levanta()
				elif self.state == 'MARCH':
					if self.deslocamentoZpes != self.deslocamentoZpesMAX:
						self.marchar()
					elif self.deslocamentoXpes != 0:
						self.freia_frente()
					else:
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.state = novo_estado
				elif self.state == 'IDDLE':
					if self.rota_dir != 0 or self.rota_esq != 0:
						self.para_de_virar()
					if self.deslocamentoXpes != 0:
						self.freia_frente()
					elif self.deslocamentoYpelves != 0:
						self.para_de_machar()
					else:
						if self.activate:
							novo_estado = self.classifica_estado()
							if novo_estado != -1:
								self.state = novo_estado
				elif self.state == 'WALK':
					if self.deslocamentoXpes < self.deslocamentoXpesMAX:
						self.acelera_frente()
					else:
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.state = novo_estado
				elif self.state == 'TURN':
					if abs(self.robo_yall_lock) > self.min_yall:
						self.vira()
					elif self.rota_dir != 0 or self.rota_esq != 0:
						self.para_de_virar()
					else:
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.state = novo_estado
				elif self.state == 'TURN90':
					if self.deslocamentoZpes != self.deslocamentoZpesMAX:
						self.marchar()
					elif abs(self.robo_yall_lock) > self.min_yall:
						self.vira()
					elif self.rota_dir != 0 or self.rota_esq != 0:
						self.para_de_virar()
					else:
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.turn90 = False
							self.state = novo_estado
				elif self.state == 'UP':
					if self.rota_dir != 0 or self.rota_esq != 0:
						self.para_de_virar()
					elif self.deslocamentoXpes != 0:
						self.freia_frente()
					elif self.deslocamentoYpelves != 0:
						self.para_de_machar()
					else:
						#robo pronto para levantar
						pass
				elif self.state == 'PENALIZED':
					if self.rota_dir != 0 or self.rota_esq != 0:
						self.para_de_virar()
					elif self.deslocamentoXpes != 0:
						self.freia_frente()
					elif self.deslocamentoYpelves != 0:
						self.para_de_machar()

				self.atualiza_fps()
				self.atualiza_tempo_marcha()
				self.atualiza_cinematica()
				self.gravity_compensation()
				if (self.orientation_mode == OrientationMode.VISAO):
					self.compura_direcao_pelo_gimbal()
				elif (self.orientation_mode) == OrientationMode.POSICAO_ALVO:
					self.computa_direcao_pela_posicao_alvo()
				self.calcula_centro_pressao()
				timer_main_loop += self.deltaTime
				time.sleep(self.simTransRate)


			except KeyboardInterrupt as e:
				print("Main loop finalizado!!")
				rclpy.shutdown()
				self.sim_t.join()
				break
			except Exception as e:
				raise e

	def levanta(self):
		if not self.interpolando:
			self.interpolando = True
			if (self.robo_pitch >0):
				t = threading.Thread(target=self.interpola_estados, args=[self.estados_levanta_frente, self.tempos_levanta_frente])
				t.daemon = True
				t.start()
			else:
				t = threading.Thread(target=self.interpola_estados, args=[self.estados_levanta_costas, self.tempos_levanta_costas])
				t.daemon = True
				t.start()

	# 	'''
	# 		- Define para qual lado o robô deve virar com base no yall lock
	# 	'''
	def vira(self):
		if self.robo_yall_lock < 0:
			self.rot_desvio = 1
		else:
			self.rot_desvio = -1

	# 	'''
	# 		- Vai parando de virar pelo tempo definido no construtor
	# 	'''
	def para_de_virar(self):
		self.rot_desvio = 0

	# 	'''
	# 		- Interpola distância de deslocamento dos pés, da atual até o max setado no contrutor
	# 	'''
	def acelera_frente(self):
		if not self.acelerando and self.deslocamentoXpes != self.deslocamentoXpesMAX:
			self.acelerando = True
			self.timer_movimentacao = 0
		if self.deslocamentoXpes != self.deslocamentoXpesMAX:
			self.timer_movimentacao += self.deltaTime
			self.deslocamentoXpes = sigmoid_deslocada(self.timer_movimentacao, self.tempo_acelerando)*self.deslocamentoXpesMAX
		if abs(self.deslocamentoXpes - self.deslocamentoXpesMAX) <= 0.01:
			self.deslocamentoXpes = self.deslocamentoXpesMAX
			self.acelerando = False

	# 	'''
	# 		- Interpola distância de deslocamento dos pés, diminuindo este valor até que se torne 0
	# 	'''
	def freia_frente(self):
		if not self.freando and self.deslocamentoXpes != 0:
			self.freando = True
			self.timer_movimentacao = 0
		if self.deslocamentoXpes != 0:
			self.timer_movimentacao += self.deltaTime
			self.deslocamentoXpes = (1. - sigmoid_deslocada(self.timer_movimentacao, self.tempo_acelerando))*self.deslocamentoXpesMAX
		if self.deslocamentoXpes  <= 0.01:
			self.deslocamentoXpes = 0
			self.freando = False

	# 	'''
	# 		- Interpola deslocamento lateral da pelves e o deslocamento para cima dos pés, da atual até o max
	# 	'''
	def marchar(self):
		if (not self.marchando) and self.deslocamentoZpes != self.deslocamentoZpesMAX:
			self.marchando = True
			self.timer_movimentacao = 0
		if self.deslocamentoZpes != self.deslocamentoZpesMAX:
			self.timer_movimentacao += self.deltaTime
			self.deslocamentoZpes = sigmoid_deslocada(self.timer_movimentacao - (self.tempo_marchando/5), self.tempo_marchando, inc=9.)*self.deslocamentoZpesMAX
			self.deslocamentoYpelves = sigmoid_deslocada(self.timer_movimentacao, self.tempo_marchando, inc=9.)*self.deslocamentoYpelvesMAX
		if abs(self.deslocamentoZpes - self.deslocamentoZpesMAX) <= 0.01:
			self.deslocamentoZpes = self.deslocamentoZpesMAX
			self.deslocamentoYpelves = self.deslocamentoYpelvesMAX
			self.marchando = False

	# 	'''
	# 		- Interpola deslocamento lateral da pelves e o deslocamento para cima dos pés,
	#           diminuindo estes valores até chegar em 0
	# 	'''
	def para_de_machar(self):
		if not self.recuando and self.deslocamentoYpelves != 0:
			self.recuando = True
			self.timer_movimentacao = 0
		if self.deslocamentoYpelves != 0:
			self.timer_movimentacao += self.deltaTime
			self.deslocamentoZpes = (1. - sigmoid_deslocada(self.timer_movimentacao, self.tempo_marchando, inc=9.))*self.deslocamentoZpesMAX
			self.deslocamentoYpelves = (1. - sigmoid_deslocada(self.timer_movimentacao, self.tempo_marchando, inc=9.))*self.deslocamentoYpelvesMAX
		if self.deslocamentoYpelves <= 0.01:
			self.deslocamentoZpes = 0
			self.deslocamentoYpelves = 0
			self.recuando = False


	#Change state
	def atualiza_tempo_marcha(self):
		# incrementa currentStateTime até tempoPasso (até trocar voltar à fase de suporte duplo)
		self.t_state += self.deltaTime
		if self.t_state >= self.tempoPasso:
			self.t_state = 0
			# indica se é a perna direita (1) ou esquerda(0) no chão
			self.perna = (self.perna+1)%2
			if self.rot_desvio != 0:
				if self.rot_desvio > 0:
					if self.perna:
						self.rota_dir = -1
						self.rota_esq *= 2
					else:
						self.rota_esq = -1
						self.rota_dir *= 2
				else:
					if self.perna:
						self.rota_dir = 1
						self.rota_esq *= 2
					else:
						self.rota_esq = 1
						self.rota_dir *= 2
			else:
				if math.fabs(self.rota_esq) == 2:
					self.rota_esq = 0
				elif math.fabs(self.rota_esq) == 1:
					self.rota_esq *= 2
				if math.fabs(self.rota_dir) == 2:
					self.rota_dir = 0
				elif math.fabs(self.rota_dir) == 1:
					self.rota_dir *= 2

# 	'''
# 		- Retorna os 6 angulos de da perna, calculando a cinematica inversa.
#           Considerando o pé como base e o quadril como ponto variável
# 	'''
	def footToHip(self, pointHip):
		angulos = []
		x,y,z = pointHip

		#ankle roll
		theta = math.atan(y/z)
		angulos.append(theta)

		#ankle pitch
		b = math.sqrt(x**2+y**2+z**2)
		a_2 = self.a**2
		b_2 = b**2
		c_2 = self.c**2
		anguloA = math.acos((a_2-(b_2+c_2))/(-2*b*self.c))
		betha = math.atan(x/z)
		anguloA = betha + anguloA
		angulos.append(anguloA)

		#knee
		anguloB = math.acos((b_2-(a_2+c_2))/(-2*self.a*self.c))
		anguloB = anguloB - math.pi
		angulos.append(anguloB)

		#hip pitch
		anguloC = math.acos((c_2-(a_2+b_2))/(-2*self.a*b))
		anguloC = anguloC - betha
		angulos.append(anguloC)

		#hip roll
		angulos.append(theta)

		#hip yall
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
	def getTragectoryPoint(self, x):
		pos_pelves = self.pos_inicial_pelves[:]

		dif_estado = (x-self.nEstados/2)

		aux = (2*dif_estado)/50
		aux2 = ((math.exp(aux) - math.exp(- aux))/(math.exp(aux)+math.exp(-aux)))

		p1 = (self.deslocamentoXpes/2)*aux2
		pos_pelves[0] = p1
		pos_pelves[1] += -self.deslocamentoYpelves*math.sin(x*math.pi/self.nEstados)

		pos_foot = self.pos_inicial_pelves[:]
		p2 = (-self.deslocamentoXpes/2)*aux2
		pos_foot[0] = p2
		pos_foot[1] += self.deslocamentoYpelves*math.sin(x*math.pi/self.nEstados)*1.3
		pos_foot[2] = self.altura - self.deslocamentoZpes*math.exp(-(dif_estado**2)/600)
		return pos_pelves, pos_foot

	# interpolação simples entre estados
	def interpola_estados(self, estados, tempos):
		if len(estados) > 0:
			p_ant = estados[0]
		else:
			return
		for i in range(1, len(estados)):
			p_atual = estados[i]
			t = tempos[i-1]
			timer = 0
			m = []
			for j in range(len(p_atual)):
				m.append((p_ant[j] - p_atual[j])/(0 - t))

			while(timer < t):
				timer += self.deltaTime
				for j in range(len(p_atual)):
					self.msg_to_micro[j] = m[j]*timer + p_ant[j]
		self.state = 'IDDLE'
		self.interpolando = False


	def atualiza_cinematica(self):
		x = (self.t_state*125)/self.tempoPasso
		pelv_point, foot_point = self.getTragectoryPoint(x)
		if self.perna:
			#CINEMÁTICA INVERSA
			data_pelv = self.footToHip(pelv_point)
			data_foot = self.footToHip(foot_point)

			#CONTROLE PÉ SUSPENSO
			
			if (self.inertial_foot_enable):
				if self.total_press == 0:
					influencia = 0
				else:
					influencia = np.sum(self.Lfoot_press)/self.total_press
				data_foot[:2] = np.array(data_foot[:2]) + np.array(self.Lfoot_orientation[:2])*(np.pi/180.)*(1-influencia)
	
			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A ESQUERDA
			if self.rota_dir == 1:
				data_pelv[5] = self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			elif self.rota_dir == -1:
				data_pelv[5] = -self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A DIREITA
			if self.rota_esq == 2:
				data_foot[5] = self.angulo_vira - (self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			elif self.rota_esq == -2:
				data_foot[5] = -self.angulo_vira - (-self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			else:
				data_foot[5] = 0

			#PÉ DIREITO ESTÁ EM CONTATO COM O CHÃO E PÉ ESQUERDO ESTÁ SE MOVENDO.
			data = data_pelv + data_foot + [0]*6

			#CONFIGURA BODY SOLVER PARA INVOCAR FUNÇÕES DO MODELO DINÂMICO DO ROBÔ
			self.body.set_angles(self.perna, data_pelv, data_foot)
		else:
			#CINEMÁTICA INVERSA
			data_pelv = self.footToHip(pelv_point)
			data_foot = self.footToHip(foot_point)

			#CONTROLE PÉ SUSPENSO
			if (self.inertial_foot_enable):
				if self.total_press == 0:
					influencia = 0
				else:
					influencia = np.sum(self.Rfoot_press)/self.total_press
				data_foot[:2] = np.array(data_foot[:2]) + np.array(self.Rfoot_orientation[:2])*(np.pi/180.)*(1-influencia)

			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A ESQUERDA
			if self.rota_esq == 1:
				data_pelv[5] =  self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			elif self.rota_esq == -1:
				data_pelv[5] =  -self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A DIREITA
			if self.rota_dir == 2:
				data_foot[5] =  self.angulo_vira - (self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			elif self.rota_dir == -2:
				data_foot[5] =  -self.angulo_vira - (-self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))		
				data_foot[5] = data_foot[5] * math.pi/180.
			else:
				data_foot[5] = 0

			#PÉ ESQUERDO ESTÁ EM CONTATO COM O CHÃO E PÉ DIREITO ESTÁ SE MOVENDO.
			data = data_foot + data_pelv + [0]*6

			#CONFIGURA BODY SOLVER PARA INVOCAR FUNÇÕES DO MODELO DINÂMICO DO ROBÔ
			self.body.set_angles(self.perna, data_foot, data_pelv)

		self.msg_to_micro[:18] = data



if __name__ == '__main__':
	control = Controlador(time_id = 17,
						robo_id = 0,
						simulador_enable=True,
						inertial_foot_enable=False,
						gravity_compensation_enable=True)
	control.run()
