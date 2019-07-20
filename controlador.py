# -*- coding:utf-8 -*-

import socket
import threading
import time
import os
import numpy as np
from functools import reduce
import struct
import csv

try:
	from std_msgs.msg import Float32MultiArray
except Exception as e:
	pass
try:
	import rospy
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

RAD_TO_DEG = 180 / np.pi
DEG_TO_RAD = np.pi / 180.

KP_CONST = 0.6

RIGHT_ANKLE_ROLL = 0
RIGHT_ANKLE_PITCH = 1
RIGHT_KNEE = 2
RIGHT_HIP_PITCH = 3
RIGHT_HIP_ROLL = 4
RIGHT_HIP_YALL = 5
LEFT_ANKLE_ROLL = 6
LEFT_ANKLE_PITCH = 7
LEFT_KNEE = 8
LEFT_HIP_PITCH = 9
LEFT_HIP_ROLL = 10
LEFT_HIP_YALL = 11
LEFT_ARM_PITCH = 12
LEFT_ARM_YALL = 13
LEFT_ARM_ROLL = 14
RIGHT_ARM_PITCH = 15
RIGHT_ARM_YALL = 16
RIGHT_ARM_ROLL = 17

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


def sigmoid_deslocada(x, periodo):
	return 1. / (1. + math.exp(-(12. / periodo) * (x - (periodo / 2.))))


class Controlador():

	def __init__(self,
				 simulador_enable=False,
				 time_id=17,
				 robo_id=0,
				 altura_inicial=17.,
				 tempo_passo=0.3,
				 deslocamento_ypelves=1.4,
				 deslocamento_zpes=2.,
				 deslocamento_xpes=1.5,
				 deslocamento_zpelves=30.,
				 inertial_foot_enable=False,
				 gravity_compensation_enable=False,
				 step_mode=False):
		if (robo_id == -1):
			print("ERRO: ID do robo inválido")
			exit()

		self.simulador = simulador_enable
		self.step_mode = step_mode
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
		self.msg_to_micro = [0] * 20
		self.CoM_parts = []
		self.fps_count = 0
		self.last_time = 0
		self.count_frames = 0
		self.timer_fps = 0
		self.deltaTime = 0
		self.time_ignore_GC = 0.1  # entre 0 e 1 - porcentagem de tempo para ignorar o gravity compensation
		self.ON_PIN = 25
		if RASPBERRY:
			GPIO.setup(self.ON_PIN, GPIO.IN)

		self.visao_ativada = False
		self.micro_ativado = False
		self.incercial_ativado = False
		self.simulador_ativado = True

		self.simTransRate = 1 / self.nEstados * self.tempoPasso

		self.tempo_acelerando = 4.
		self.tempo_marchando = 4.
		self.tempo_virando = 3.

		self.visao_search = False
		self.visao_bola = False
		self.turn90 = False
		self.max_yall = 20
		self.min_yall = 5
		self.dist_ideal = 10

		self.robo_roll = 0
		self.robo_yall = 0
		self.robo_pitch = 0

		self.gimbal_yall = 0
		self.gimbal_pitch = -45

		self.limiar_erro_inercial = 20  # quantos graus de diferença é permitida entre entre duas leituras consecutivas

		self.gimbal_yall_lock = 0
		self.gimbal_pitch_lock = 0
		self.robo_yall_lock = 0
		self.robo_pitch_lock = 0
		self.Lfoot_orientation = [0, 0, 0]
		self.Rfoot_orientation = [0, 0, 0]
		self.inertial_foot_enable = inertial_foot_enable
		self.gravity_compensation_enable = gravity_compensation_enable
		self.Lfoot_press = [0, 0, 0, 0]
		self.Rfoot_press = [0, 0, 0, 0]
		self.total_press = 0

		self.t_state = 0
		self.rota_dir = 0
		self.rota_esq = 0
		self.angulo_vira = 3

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

		self.rst_imu_pin = 18

		self.body = Body()

		self.last_sent_angles = [0] * 18

		self.last_looseness_control_multipliers = [0] * 18

		self.JOINT_ANGLES_MIN_MAX = [[None, None]] * 18

		self.DEFAULT_JOINT_LOOSENESS_CONTROL_ANGLES = [0] * 18

		self.RST_IMU_PIN = 18

		try:
			with open('estados_levanta_frente.csv', newline='') as csvfile:
				tabela = list(csv.reader(csvfile, delimiter=','))
				tabela = np.array(tabela)
				self.estados_levanta_frente = tabela[1:, :]
				self.tempos_levanta_frente = [4] * 19

			with open('estados_levanta_back.csv', newline='') as csvfile2:
				tabela = list(csv.reader(csvfile2, delimiter=','))
				tabela = np.array(tabela)
				self.estados_levanta_costas = tabela[1:, :]
				self.tempos_levanta_costas = [4] * 19
		except Exception as e:
			self.estados_levanta_costas = []
			self.tempos_levanta_costas = []
			self.estados_levanta_frente = []
			self.tempos_levanta_frente = []

		if self.simulador:
			self.inicia_modulo_simulador()

	def inicia_modulo_simulador(self):
		# INICIA PUBLISHER PARA ENVIAR POSIÇÕES DOS MOTORES
		print("Iniciando ROS node para execucao do simulador..")
		rospy.init_node('controller', anonymous=True)
		self.pub = rospy.Publisher('Bioloid/joint_pos', Float32MultiArray, queue_size=1)
		self.rate = rospy.Rate(self.tempoPasso / self.nEstados)
		t = threading.Thread(target=self.envia_para_simulador)
		t.daemon = True
		t.start()

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
				while not rospy.is_shutdown():
					mat.data = self.msg_to_micro[:18]

					# mat.data[10] = -mat.data[10] # quadril esquerdo ROLL
					mat.data[0] = -mat.data[0]  # calcanhar direito ROLL

					mat.data[4] = -mat.data[4]
					# mat.data[10] = -mat.data[10]

					self.pub.publish(mat)
					rospy.sleep(self.simTransRate)
		except Exception as e:
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
		if self.robo_yall + visao_msg[1] < 0:
			self.gimbal_yall = self.robo_yall + visao_msg[1] + 360
		elif self.robo_yall + visao_msg[1] > 360:
			self.gimbal_yall = (self.robo_yall + visao_msg[1]) % 360
		else:
			self.gimbal_yall = self.robo_yall + visao_msg[1]
		self.gimbal_pitch = visao_msg[0]
		self.visao_bola = visao_msg[2] != 0.

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
		self.Lfoot_press = msg.data[:4]
		self.Rfoot_press = msg.data[4:]
		self.total_press = np.sum(self.Lfoot_press) + np.sum(self.Rfoot_press)

	# 	Leitura IMU - robo
	def robot_inertial_callback(self, msg):
		self.robo_yall = msg.data[2]
		self.robo_pitch = msg.data[1]
		self.robo_roll = msg.data[0]

		if (abs(self.robo_pitch) > 45 or abs(self.robo_roll) > 45) and not self.interpolando:
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

		if self.visao_ativada:
			# manda mensagem para a rasp da visão dizendo o estado atual, a inclinação vertical e rotação horizontal
			self.visao_socket.send(
				("['" + self.state + "'," + str(self.robo_pitch) + ',' + str(self.robo_yall) + ']').encode())

	def classifica_estado(self):
		if self.state is 'IDDLE':
			if self.turn90:
				return 'MARCH'
			elif self.visao_bola:
				return 'MARCH'
			else:
				return -1
		elif self.state is 'TURN90':
			if abs(self.robo_yall_lock) <= self.min_yall:
				return 'MARCH'
			else:
				return -1
		elif self.state is 'MARCH':
			if self.turn90:
				return 'TURN90'
			elif not self.visao_bola:
				return 'IDDLE'
			elif self.visao_bola and abs(self.robo_yall_lock) > self.max_yall and self.visao_ativada:
				return 'TURN'
			elif self.visao_bola and self.robo_pitch_lock > -45:
				return 'WALK'
			else:
				return -1
		elif self.state is 'WALK':
			if not self.visao_bola or abs(self.robo_yall_lock) > self.max_yall or self.robo_pitch_lock <= -45:
				return 'MARCH'
			else:
				return -1
		elif self.state is 'TURN':
			if not self.visao_bola or abs(self.robo_yall_lock) < self.min_yall:
				return 'MARCH'
			else:
				return -1
		else:
			print("ERRO: Estado invalido!!")

	def gravity_compensation(self):
		# return
		if not self.gravity_compensation_enable or (
				self.t_state < self.tempoPasso / 2 and self.t_state < self.tempoPasso * self.time_ignore_GC) or (
				self.t_state >= self.tempoPasso / 2 and self.t_state > self.tempoPasso * (
				1 - self.time_ignore_GC)) or self.deslocamentoYpelves != self.deslocamentoYpelvesMAX or self.state is "IDDLE":
			return

		torques = self.body.get_torque_in_joint(self.perna, [3, 5])

		dQ = (np.array(torques) / KP_CONST) / 15
		# print(dQ[1], self.perna)

		dQ *= math.sin(self.t_state * math.pi / self.tempoPasso)
		if self.perna:
			self.msg_to_micro[RIGHT_KNEE] += dQ[0]
			self.msg_to_micro[RIGHT_HIP_ROLL] -= dQ[1]
		else:
			self.msg_to_micro[LEFT_KNEE] += dQ[0]
			self.msg_to_micro[LEFT_HIP_ROLL] += dQ[1]

	def posiciona_robo(self):
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

	def run(self):
		# update function
		timer_main_loop = 0
		# perna direita (1) ou esquerda(0) no chão
		self.perna = 0
		self.rot_desvio = 0
		while (True):
			try:
				# print ("%s GIMBAL_YALL:%.f  ROBO_YALL:%.2f  ANGULO PARA VIRAR:%.2f BOLA:%r"%(self.state, self.gimbal_yall, self.robo_yall, self.robo_yall_lock, self.visao_bola), flush=True)
				# print (np.array(self.Rfoot_orientation).astype(np.int), np.array(self.Lfoot_orientation).astype(np.int))
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
				if (self.state is 'FALLEN'):
					if not self.interpolando:
						self.levanta()
				elif self.state is 'MARCH':
					if self.deslocamentoYpelves != self.deslocamentoYpelvesMAX:
						self.marchar()
					elif self.deslocamentoXpes != 0:
						self.freia_frente()
					else:
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.state = novo_estado
				elif self.state is 'IDDLE':
					if self.rota_dir != 0 or self.rota_esq != 0:
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
					if self.deslocamentoXpes < self.deslocamentoXpesMAX:
						self.acelera_frente()
					else:
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.state = novo_estado
				elif self.state is 'TURN':
					if abs(self.robo_yall_lock) > self.min_yall:
						self.vira()
					elif self.rota_dir != 0 or self.rota_esq != 0:
						self.para_de_virar()
					else:
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.state = novo_estado
				elif self.state is 'TURN90':
					if self.deslocamentoYpelves < self.deslocamentoYpelvesMAX:
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
				elif self.state is 'UP':
					if self.rota_dir != 0 or self.rota_esq != 0:
						self.para_de_virar()
					elif self.deslocamentoXpes != 0:
						self.freia_frente()
					elif self.deslocamentoYpelves != 0:
						self.recuar()
					else:
						# robo pronto para levantar
						pass
				elif self.state is 'PENALIZED':
					if self.rota_dir != 0 or self.rota_esq != 0:
						self.para_de_virar()
					elif self.deslocamentoXpes != 0:
						self.freia_frente()
					elif self.deslocamentoYpelves != 0:
						self.recuar()

				self.atualiza_fps()
				self.chage_state()
				self.atualiza_cinematica()
				self.gravity_compensation()
				# self.posiciona_gimbal()
				self.posiciona_robo()
				timer_main_loop += self.deltaTime
				time.sleep(self.simTransRate)


			except KeyboardInterrupt as e:
				print("Main loop finalizado!!")
				break
			except Exception as e:
				raise e

	# Anda de lado para alinhar com o gol
	def posiciona(self):
		if not self.posicionando:
			self.posicionando = True
			self.timer_reposiciona = 0
		if self.posicionando:
			self.timer_reposiciona += self.deltaTime
			if self.robo_yall > 270:
				# anda de lado para a esquerda
				if self.perna:
					self.anda_de_lado_esquerda()
				else:
					self.desanda_de_lado_esquerda()
			elif self.robo_yall < 90:
				# anda de lado para a direita
				if not self.perna:
					self.anda_de_lado_direita()
				else:
					self.desanda_de_lado_direita()
		if self.timer_reposiciona > self.tempoPasso * 6:
			if abs(self.pos_inicial_pelves[1]) < 0.01:
				self.pos_inicial_pelves[1] = 0
				self.posicionando = False
				self.state = 'IDDLE'
			else:
				if self.pos_inicial_pelves[1] > 0 and not self.perna:
					self.desanda_de_lado_esquerda()
				if self.pos_inicial_pelves[1] < 0 and self.perna:
					self.desanda_de_lado_direita()

	def levanta(self):
		if not self.interpolando:
			self.interpolando = True
			if (self.robo_pitch > 0):
				t = threading.Thread(target=self.interpola_estados,
									 args=[self.estados_levanta_frente, self.tempos_levanta_frente])
				t.daemon = True
				t.start()
			else:
				t = threading.Thread(target=self.interpola_estados,
									 args=[self.estados_levanta_costas, self.tempos_levanta_costas])
				t.daemon = True
				t.start()

	def anda_de_lado_esquerda(self):
		if (not self.ladeando or self.desladeando) and self.pos_inicial_pelves[1] != self.deslocamentoYpelvesMAX / 8:
			self.ladeando = True
			self.desladeando = False
			self.timer_movimentacao = 0
		if self.pos_inicial_pelves[1] != self.deslocamentoYpelvesMAX / 8:
			self.timer_movimentacao += self.deltaTime
			self.pos_inicial_pelves[1] = sigmoid_deslocada(self.timer_movimentacao,
														   self.tempoPasso) * self.deslocamentoYpelvesMAX / 8
		if abs(self.pos_inicial_pelves[1] - self.deslocamentoYpelvesMAX / 8) <= 0.01:
			self.pos_inicial_pelves[1] = self.deslocamentoYpelvesMAX / 8
			self.ladeando = False

	def desanda_de_lado_esquerda(self):
		if (not self.desladeando or self.ladeando) and self.pos_inicial_pelves[1] != 0.:
			self.desladeando = True
			self.ladeando = False
			self.timer_movimentacao = 0
		if self.pos_inicial_pelves[1] != 0.:
			self.timer_movimentacao += self.deltaTime
			self.pos_inicial_pelves[1] = (1 - sigmoid_deslocada(self.timer_movimentacao,
																self.tempoPasso)) * self.deslocamentoYpelvesMAX / 8
		if self.pos_inicial_pelves[1] <= 0.01:
			self.pos_inicial_pelves[1] = 0.
			self.desladeando = False

	def anda_de_lado_direita(self):
		if (not self.ladeando or self.desladeando) and self.pos_inicial_pelves[1] != self.deslocamentoYpelvesMAX / 8:
			self.ladeando = True
			self.desladeando = False
			self.timer_movimentacao = 0
		if self.pos_inicial_pelves[1] != self.deslocamentoYpelvesMAX / 8:
			self.timer_movimentacao += self.deltaTime
			self.pos_inicial_pelves[1] = -sigmoid_deslocada(self.timer_movimentacao,
															self.tempoPasso) * self.deslocamentoYpelvesMAX / 8
		if abs(self.pos_inicial_pelves[1] - self.deslocamentoYpelvesMAX / 8) <= 0.01:
			self.pos_inicial_pelves[1] = self.deslocamentoYpelvesMAX / 8
			self.ladeando = False

	def desanda_de_lado_direita(self):
		if (not self.desladeando or self.ladeando) and self.pos_inicial_pelves[1] != 0.:
			self.desladeando = True
			self.ladeando = False
			self.timer_movimentacao = 0
		if self.pos_inicial_pelves[1] != 0.:
			self.timer_movimentacao += self.deltaTime
			self.pos_inicial_pelves[1] = (-1 + sigmoid_deslocada(self.timer_movimentacao,
																 self.tempoPasso)) * self.deslocamentoYpelvesMAX / 8
		if self.pos_inicial_pelves[1] <= 0.01:
			self.pos_inicial_pelves[1] = 0.
			self.desladeando = False

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
			self.deslocamentoXpes = sigmoid_deslocada(self.timer_movimentacao,
													  self.tempo_acelerando) * self.deslocamentoXpesMAX
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
			self.deslocamentoXpes = (1. - sigmoid_deslocada(self.timer_movimentacao,
															self.tempo_acelerando)) * self.deslocamentoXpesMAX
		if self.deslocamentoXpes <= 0.01:
			self.deslocamentoXpes = 0
			self.freando = False

	# 	'''
	# 		- Interpola deslocamento lateral da pelves e o deslocamento para cima dos pés, da atual até o max
	# 	'''
	def marchar(self):
		if (not self.marchando) and self.deslocamentoYpelves != self.deslocamentoYpelvesMAX:
			self.marchando = True
			self.timer_movimentacao = 0
		if self.deslocamentoYpelves != self.deslocamentoYpelvesMAX:
			self.timer_movimentacao += self.deltaTime
			self.deslocamentoZpes = sigmoid_deslocada(self.timer_movimentacao,
													  self.tempo_marchando) * self.deslocamentoZpesMAX
			self.deslocamentoYpelves = sigmoid_deslocada(self.timer_movimentacao,
														 self.tempo_marchando) * self.deslocamentoYpelvesMAX
		if abs(self.deslocamentoYpelves - self.deslocamentoYpelvesMAX) <= 0.01:
			self.deslocamentoZpes = self.deslocamentoZpesMAX
			self.deslocamentoYpelves = self.deslocamentoYpelvesMAX
			self.marchando = False

	# 	'''
	# 		- Interpola deslocamento lateral da pelves e o deslocamento para cima dos pés,
	#           diminuindo estes valores até chegar em 0
	# 	'''
	def recuar(self):
		if not self.recuando and self.deslocamentoYpelves != 0:
			self.recuando = True
			self.timer_movimentacao = 0
		if self.deslocamentoYpelves != 0:
			self.timer_movimentacao += self.deltaTime
			self.deslocamentoZpes = (1. - sigmoid_deslocada(self.timer_movimentacao,
															self.tempo_marchando)) * self.deslocamentoZpesMAX
			self.deslocamentoYpelves = (1. - sigmoid_deslocada(self.timer_movimentacao,
															   self.tempo_marchando)) * self.deslocamentoYpelvesMAX
		if self.deslocamentoYpelves <= 0.01:
			self.deslocamentoZpes = 0
			self.deslocamentoYpelves = 0
			self.recuando = False

	# Change state
	def chage_state(self):
		# incrementa currentStateTime até tempoPasso (até trocar voltar à fase de suporte duplo)
		self.t_state += self.deltaTime
		if self.t_state >= self.tempoPasso:
			self.t_state = 0
			# indica se é a perna direita (1) ou esquerda(0) no chão
			self.perna = (self.perna + 1) % 2
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
		x, y, z = pointHip

		# ankle roll
		theta = math.atan(y / z)
		angulos.append(theta)

		# ankle pitch
		b = math.sqrt(x ** 2 + y ** 2 + z ** 2)
		a_2 = self.a ** 2
		b_2 = b ** 2
		c_2 = self.c ** 2
		anguloA = math.acos((a_2 - (b_2 + c_2)) / (-2 * b * self.c))
		betha = math.atan(x / z)
		anguloA = betha + anguloA
		angulos.append(anguloA)

		# knee
		anguloB = math.acos((b_2 - (a_2 + c_2)) / (-2 * self.a * self.c))
		anguloB = anguloB - math.pi
		angulos.append(anguloB)

		# hip pitch
		anguloC = math.acos((c_2 - (a_2 + b_2)) / (-2 * self.a * b))
		anguloC = anguloC - betha
		angulos.append(anguloC)

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
	def getTragectoryPoint(self, x):
		pos_pelves = self.pos_inicial_pelves[:]

		# nEstados * [-0.5,0.5]
		# aux_estados = (x-self.N_ESTADOS/2)

		# deslocamentoXpes/2 * tgh(x)

		dif_estado = (x - self.nEstados / 2)

		aux = (2 * dif_estado) / 50
		aux2 = ((math.exp(aux) - math.exp(- aux)) / (math.exp(aux) + math.exp(-aux)))

		pos1_aux = self.deslocamentoYpelves * math.sin(x * math.pi / self.nEstados)

		p1 = (self.deslocamentoXpes / 2) * aux2
		pos_pelves[0] = p1
		pos_pelves[1] += -pos1_aux

		pos_foot = self.pos_inicial_pelves[:]
		p2 = (-self.deslocamentoXpes / 2) * aux2
		pos_foot[0] = p2
		pos_foot[1] += pos1_aux
		pos_foot[2] = self.altura - self.deslocamentoZpes * math.exp(-(dif_estado ** 2) / 600)
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

			while (timer < t):
				timer += self.deltaTime
				for j in range(len(p_atual)):
					self.msg_to_micro[j] = m[j] * timer + p_ant[j]
		self.state = 'IDDLE'
		self.interpolando = False

	def atualiza_cinematica(self):
		x = (self.t_state * 125) / self.tempoPasso
		pelv_point, foot_point = self.getTragectoryPoint(x)
		if self.perna:
			# CINEMÁTICA INVERSA
			data_pelv = self.footToHip(pelv_point)
			data_foot = self.footToHip(foot_point)

			# CONTROLE PÉ SUSPENSO

			if (self.inertial_foot_enable):
				if self.total_press == 0:
					influencia = 0
				else:
					influencia = np.sum(self.Lfoot_press) / self.total_press
				data_foot[:2] = np.array(data_foot[:2]) + np.array(self.Lfoot_orientation[:2]) * (np.pi / 180.) * (
							1 - influencia)

			# ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A ESQUERDA
			if self.rota_dir == 1:
				data_pelv[5] = self.angulo_vira / 2. + self.angulo_vira / 2. * ((np.exp(
					(2 * (x - self.nEstados / 2)) / 50) - np.exp((2 * (x - self.nEstados / 2)) / -50)) / (np.exp(
					(2 * (x - self.nEstados / 2)) / 50) + np.exp((2 * (x - self.nEstados / 2)) / -50)))
				data_pelv[5] = data_pelv[5] * math.pi / 180.
			elif self.rota_dir == -1:
				data_pelv[5] = -self.angulo_vira / 2. - self.angulo_vira / 2. * ((np.exp(
					(2 * (x - self.nEstados / 2)) / 50) - np.exp((2 * (x - self.nEstados / 2)) / -50)) / (np.exp(
					(2 * (x - self.nEstados / 2)) / 50) + np.exp((2 * (x - self.nEstados / 2)) / -50)))
				data_pelv[5] = data_pelv[5] * math.pi / 180.
			else:
				data_pelv[5] = 0

			# ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A DIREITA
			if self.rota_esq == 2:
				data_foot[5] = self.angulo_vira - (self.angulo_vira / 2. + self.angulo_vira / 2. * ((np.exp(
					(2 * (x - self.nEstados / 2)) / 50) - np.exp((2 * (x - self.nEstados / 2)) / -50)) / (np.exp(
					(2 * (x - self.nEstados / 2)) / 50) + np.exp((2 * (x - self.nEstados / 2)) / -50))))
				data_foot[5] = data_foot[5] * math.pi / 180.
			elif self.rota_esq == -2:
				data_foot[5] = -self.angulo_vira - (-self.angulo_vira / 2. - self.angulo_vira / 2. * ((np.exp(
					(2 * (x - self.nEstados / 2)) / 50) - np.exp((2 * (x - self.nEstados / 2)) / -50)) / (np.exp(
					(2 * (x - self.nEstados / 2)) / 50) + np.exp((2 * (x - self.nEstados / 2)) / -50))))
				data_foot[5] = data_foot[5] * math.pi / 180.
			else:
				data_foot[5] = 0

			# PÉ DIREITO ESTÁ EM CONTATO COM O CHÃO E PÉ ESQUERDO ESTÁ SE MOVENDO.
			data = data_pelv + data_foot + [0] * 6

			# CONFIGURA BODY SOLVER PARA INVOCAR FUNÇÕES DO MODELO DINÂMICO DO ROBÔ
			self.body.set_angles(self.perna, data_pelv, data_foot)
		else:
			# CINEMÁTICA INVERSA
			data_pelv = self.footToHip(pelv_point)
			data_foot = self.footToHip(foot_point)

			# CONTROLE PÉ SUSPENSO
			if (self.inertial_foot_enable):
				if self.total_press == 0:
					influencia = 0
				else:
					influencia = np.sum(self.Rfoot_press) / self.total_press
				data_foot[:2] = np.array(data_foot[:2]) + np.array(self.Rfoot_orientation[:2]) * (np.pi / 180.) * (
							1 - influencia)

			# ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A ESQUERDA
			if self.rota_esq == 1:
				data_pelv[5] = self.angulo_vira / 2. + self.angulo_vira / 2. * ((np.exp(
					(2 * (x - self.nEstados / 2)) / 50) - np.exp((2 * (x - self.nEstados / 2)) / -50)) / (np.exp(
					(2 * (x - self.nEstados / 2)) / 50) + np.exp((2 * (x - self.nEstados / 2)) / -50)))
				data_pelv[5] = data_pelv[5] * math.pi / 180.
			elif self.rota_esq == -1:
				data_pelv[5] = -self.angulo_vira / 2. - self.angulo_vira / 2. * ((np.exp(
					(2 * (x - self.nEstados / 2)) / 50) - np.exp((2 * (x - self.nEstados / 2)) / -50)) / (np.exp(
					(2 * (x - self.nEstados / 2)) / 50) + np.exp((2 * (x - self.nEstados / 2)) / -50)))
				data_pelv[5] = data_pelv[5] * math.pi / 180.
			else:
				data_pelv[5] = 0

			# ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A DIREITA
			if self.rota_dir == 2:
				data_foot[5] = self.angulo_vira - (self.angulo_vira / 2. + self.angulo_vira / 2. * ((np.exp(
					(2 * (x - self.nEstados / 2)) / 50) - np.exp((2 * (x - self.nEstados / 2)) / -50)) / (np.exp(
					(2 * (x - self.nEstados / 2)) / 50) + np.exp((2 * (x - self.nEstados / 2)) / -50))))
				data_foot[5] = data_foot[5] * math.pi / 180.
			elif self.rota_dir == -2:
				data_foot[5] = -self.angulo_vira - (-self.angulo_vira / 2. - self.angulo_vira / 2. * ((np.exp(
					(2 * (x - self.nEstados / 2)) / 50) - np.exp((2 * (x - self.nEstados / 2)) / -50)) / (np.exp(
					(2 * (x - self.nEstados / 2)) / 50) + np.exp((2 * (x - self.nEstados / 2)) / -50))))
				data_foot[5] = data_foot[5] * math.pi / 180.
			else:
				data_foot[5] = 0

			# PÉ ESQUERDO ESTÁ EM CONTATO COM O CHÃO E PÉ DIREITO ESTÁ SE MOVENDO.
			data = data_foot + data_pelv + [0] * 6

			# CONFIGURA BODY SOLVER PARA INVOCAR FUNÇÕES DO MODELO DINÂMICO DO ROBÔ
			self.body.set_angles(self.perna, data_foot, data_pelv)

		self.last_sent_angles[:18] = data

	# os multiplicadores de correção são inicializados em 0.
	for idx, name in enumerate(PARAM_NAMES, 0):
		curr_angle = data[idx]
		curr_angle_mult = self.last_looseness_control_multipliers[idx]
		# Se a última movimentação aumentou o ângulo na junta:
		if (curr_angle_mult >= 0):
			# E a movimentação anterior foi na direção oposta:
			if (curr_angle > self.last_sent_angles[idx]):
				# O multiplicador do ângulo de correção da junta se torna 1
				self.last_looseness_control_multipliers[idx] = 1
		# Se a última movimentação reduziu o ângulo na junta:
		if (curr_angle_mult <= 0):
			# E a movimentação anterior foi na direção oposta:
			if (curr_angle < self.last_sent_angles[idx]):
				# O multiplicador do ângulo de correção da junta se torna -1
				self.last_looseness_control_multipliers[idx] = -1

		# Para cada parâmetro de correção de folga de junta:
		# Novo ângulo =
		# Ângulo calculado pelo controlador +
		# Multiplicador de correção de folga * ângulo de correção de folga
		# rospy.get_param: pega parâmetro do servidor de parâmetros.
		# (param_name, default_value)
		# Após calibrar corretamente os ângulos, os mesmos podem ser aplicados diretamente no código(setar DEFAULT_JOINT_LOOSENESS_CONTROL e remover o get_param), em vez de tentar pegar
		new_joint_angle = \
			curr_angle + \
			self.last_looseness_control_multipliers[idx] * rospy.get_param(PARAM_SERVER_PREFIX + PARAM_NAMES[idx],
																		   self.DEFAULT_JOINT_LOOSENESS_CONTROL_ANGLES[
																			   idx])

		# Verifica se os ângulos das juntas não ultrapassam os limites de ângulo possíveis para ser executados nas juntas
		# Caso sejam ultrapassados, utiliza os valores máximos ou mímino definidos
		# Tuplas (min,max) definidas na lista self.JOINT_ANGLES_MIN_MAX
		min_angle, max_angle = self.JOINT_ANGLES_MIN_MAX[idx]
		if (min_angle is not None and new_joint_angle < min_angle):
			self.msg_to_micro[idx] = min_angle
		elif (max_angle is not None and new_joint_angle > max_angle):
			self.msg_to_micro[idx] = max_angle
		else:
			self.msg_to_micro[idx] = new_joint_angle

		# for idx, label in enumerate(PARAM_NAMES):
		#     print(label, "| msg_to_micro: ", self.msg_to_micro[idx], " | old_angle: ", self.last_sent_angles[idx])

	if (self.simulador_ativado):
		rospy.sleep(self.simTransRate)


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
	control = Controlador(time_id=17,
						  robo_id=0,
						  simulador_enable=True,
						  inertial_foot_enable=False,
						  gravity_compensation_enable=True)
	control.run()
