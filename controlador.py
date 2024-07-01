# -*- coding:utf-8 -*-
import threading
import time
import os
import numpy as np
import csv
try:
	from std_msgs.msg import Float32MultiArray
except Exception as e:
	print("Falha ao importar a bibliotera 'std_msgs.msg'!")
try:
	import rclpy
	from rclpy.node import Node
except Exception as e:
	print("Falha ao importar a bibliotera 'rospy'!")
import math
try:
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
from tensorboardX import SummaryWriter
from geometry_msgs.msg import Vector3
import vrep
import pandas as pd
import matplotlib.pyplot as plt



RAD_TO_DEG = 180 / np.pi
DEG_TO_RAD = np.pi / 180.

KP_CONST = 0.6


def sigmoid_deslocada(x, periodo):
	return 1./(1.+math.exp(-(12./periodo)*(x-(periodo/2.))))

class Controlador():

	def __init__(self,
				simulador_enable=False,
				time_id=17,
				robo_id=0,
				altura_inicial=17.,
				tempo_passo = 0.3,
				deslocamento_ypelves = 1.4,
				deslocamento_zpes = 2.,
				deslocamento_xpes= 1.5,
				deslocamento_zpelves = 30.,
				inertial_foot_enable = False,
				gravity_compensation_enable = False,
				step_mode=False):
		if (robo_id == -1):
			print("ERRO: ID do robo inválido")
			exit()


		self.simulador = simulador_enable
		self.step_mode = step_mode
		self.state = 'WALK'
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

		self.visao_ativada = False
		self.micro_ativado = False
		self.incercial_ativado = False
		self.simulador_ativado = True

		self.simTransRate = 1/self.nEstados*self.tempoPasso

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

		self.stat_writer = SummaryWriter(comment="amostras-controle-euristico")
		self.cont_samples = 1
		self.amostras_ori_x = []
		self.amostras_ori_y = []
		self.dist_em_x = 0

		porta = 19700
		os.system("bash /home/marcos/Documents/CoppeliaSim_Edu_V4_7_0_rev2_Ubuntu22_04/coppeliaSim.sh -gREMOTEAPISERVERSERVICE_"+str(porta)+"_FALSE_FALSE ~/Documents/dev/controle_humanoide2018/teste_09_03.ttt&")
		time.sleep(20)
		self.clientID=vrep.simxStart('127.0.0.1',porta,True,True,5000,5) # Connect to V-REP

		if self.clientID!=-1:
			# start the simulation:
			vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_blocking)
		else:
			print ("Não foi possivel estabelecer conexão com o vrep")
			exit()

		if self.simulador:
			self.inicia_modulo_simulador()


	def inicia_modulo_simulador(self):
		#INICIA PUBLISHER PARA ENVIAR POSIÇÕES DOS MOTORES
		print("Iniciando ROS node para execucao do simulador..")

		rclpy.init()
		node = Node('controller')

		self.pub = node.create_publisher(Float32MultiArray, 'Bioloid/joint_pos', 1)
		self.rate = node.create_rate(1/(self.tempoPasso/self.nEstados))
		#self.rate = rclpy.Rate(self.tempoPasso/self.nEstados)
		self.spin_t = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
		self.spin_t.start()
		self.sim_t = threading.Thread(target=self.envia_para_simulador)
		self.sim_t.daemon = True
		self.sim_t.start()

		#INICIA SUBSCRIBER PARA RECEBER DADOS DOS SENSORES INERCIAIS DOS PÉS
		node.create_subscription(Float32MultiArray, "/vrep_ros_interface/Bioloid/foot_inertial_sensor", self.foot_inertial_callback, 1)

		#INICIA SUBSCRIBER PARA RECEBER DADOS DOS SENSORES DE PRESSÃO DOS PÉS
		node.create_subscription(Float32MultiArray, "/vrep_ros_interface/Bioloid/foot_pressure_sensor", self.foot_pressure_callback, 1)

		#INICIA SUBSCRIBER PARA RECEBER DADOS DO SENSOR IMU DO ROBÔ
		node.create_subscription(Float32MultiArray, "/vrep_ros_interface/Bioloid/robot_inertial_sensor", self.robot_inertial_callback, 1)

		#INICIA SUBSCRIBER PARA RECEBER COMANDOS DA VISÃO
		node.create_subscription(Float32MultiArray, "/Bioloid/visao_cmd", self.visao_cmd_callback, 1)

		node.create_subscription(Vector3, "/vrep_ros_interface/Bioloid/robot_position", self.robot_position_callback, 1)


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
				while rclpy.ok():
					mat.data = np.array(self.msg_to_micro[:18]).astype(np.float32).tolist()

					# mat.data[10] = -mat.data[10] # quadril esquerdo ROLL
					mat.data[0] = -mat.data[0] #calcanhar direito ROLL

					mat.data[4] = -mat.data[4]
					# mat.data[10] = -mat.data[10]
					self.pub.publish(mat)
					self.rate.sleep()
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
		if self.robo_yall + visao_msg[1] < 0:
			self.gimbal_yall = self.robo_yall + visao_msg[1] + 360
		elif self.robo_yall + visao_msg[1] > 360:
			self.gimbal_yall = (self.robo_yall + visao_msg[1])% 360
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
		self.total_press = np.sum(self.Lfoot_press)+np.sum(self.Rfoot_press)

	# 	Leitura IMU - robo
	def robot_inertial_callback(self, msg):
		self.robo_yall = msg.data[2]
		self.robo_pitch = msg.data[1]
		self.robo_roll = msg.data[0]

		if (abs(self.robo_pitch) > 60 or abs(self.robo_roll) > 60) and not self.interpolando:
			self.state = 'FALLEN'

		self.amostras_ori_x.append(self.robo_roll)
		self.amostras_ori_y.append(self.robo_pitch)

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


	def robot_position_callback(self, vetor):
		self.dist_em_x = vetor.x

	def classifica_estado(self):
		if self.state == 'IDDLE':
			if self.turn90:
				return 'MARCH'
			elif self.visao_bola:
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
			elif not self.visao_bola:
				return 'IDDLE'
			elif self.visao_bola and abs(self.robo_yall_lock) > self.max_yall and self.visao_ativada:
				return 'TURN'
			elif self.visao_bola and self.robo_pitch_lock > -45:
				return 'WALK'
			else:
				return -1
		elif self.state == 'WALK':
			return -1
			if not self.visao_bola or abs(self.robo_yall_lock) > self.max_yall or self.robo_pitch_lock <= -45:
				return 'MARCH'
			else:
				return -1
		elif self.state == 'TURN':
			if not self.visao_bola or abs(self.robo_yall_lock) < self.min_yall:
				return 'MARCH'
			else:
				return -1
		else:
			print("ERRO: Estado invalido!!")


	def gravity_compensation(self):
		#return
		if not self.gravity_compensation_enable or (self.t_state < self.tempoPasso/2 and self.t_state < self.tempoPasso*self.time_ignore_GC) or (self.t_state >= self.tempoPasso/2 and self.t_state > self.tempoPasso*(1-self.time_ignore_GC)) or self.deslocamentoYpelves != self.deslocamentoYpelvesMAX or self.state == "IDDLE":
			return

		torques = self.body.get_torque_in_joint(self.perna,[3,5])

		dQ = (np.array(torques)/KP_CONST)/15
		#print(dQ[1], self.perna)

		dQ *= math.sin(self.t_state*math.pi/self.tempoPasso)
		if self.perna:
			self.msg_to_micro[self.RIGHT_ANKLE_PITCH] += dQ[0]
			self.msg_to_micro[self.RIGHT_HIP_ROLL] += (dQ[1]*-1)
		else:
			self.msg_to_micro[self.LEFT_KNEE] += dQ[0]
			self.msg_to_micro[self.LEFT_HIP_ROLL] += dQ[1]

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
		#update function
		timer_main_loop = 0
		# perna direita (1) ou esquerda(0) no chão

		duracoes = []
		dp_ori_x = []
		dp_ori_y = []
		me_ori_x = []
		me_ori_y = []
		distancias = []

		self.perna = 0
		self.rot_desvio = 0
		self.tempo_anterior = time.time()
		while (True):
			try:
				#print ("%s GIMBAL_YALL:%.f  ROBO_YALL:%.2f  ANGULO PARA VIRAR:%.2f BOLA:%r"%(self.state, self.gimbal_yall, self.robo_yall, self.robo_yall_lock, self.visao_bola), flush=True)
				#print (np.array(self.Rfoot_orientation).astype(np.int), np.array(self.Lfoot_orientation).astype(np.int))
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

				if time.time() - self.tempo_anterior > 30:
					self.state = 'FALLEN'
				if (self.state == 'FALLEN'):
					if (self.cont_samples <= 40):
						duracao = time.time()-self.tempo_anterior
						print("salvando dados da simulação %i" % self.cont_samples)
						duracoes.append(duracao)
						dp_ori_x.append(np.std(self.amostras_ori_x))
						dp_ori_y.append(np.std(self.amostras_ori_y))
						me_ori_x.append(np.mean(self.amostras_ori_x))
						me_ori_y.append(np.mean(self.amostras_ori_y))
						distancias.append(self.dist_em_x)

						self.stat_writer.add_scalar("Tempo de simulação/Episode", duracao, self.cont_samples)
						self.stat_writer.add_scalar("Desvio padrão angular X/Episode", np.std(self.amostras_ori_x), self.cont_samples)
						self.stat_writer.add_scalar("Desvio padrão angular Y/Episode", np.std(self.amostras_ori_y), self.cont_samples)
						self.stat_writer.add_scalar("Média angular X/Episode", np.mean(self.amostras_ori_x), self.cont_samples)
						self.stat_writer.add_scalar("Média angular Y/Episode", np.mean(self.amostras_ori_y), self.cont_samples)
						self.stat_writer.add_scalar("Distância percorrida X/Episode", self.dist_em_x, self.cont_samples)
					
						# Reinicia simulação
						vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_blocking)
						time.sleep(5)
						vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_blocking)

						self.amostras_ori_x = []
						self.amostras_ori_y = []
						self.tempo_anterior = time.time()
						self.state = 'WALK'
					else:
						print("Finalizando teste")
						print (len(duracoes))
						print (len(dp_ori_x))
						print (len(dp_ori_y))
						print (len(me_ori_x))
						print (len(me_ori_y))
						print (len(distancias))
						duracoes = pd.Series(np.array(duracoes))
						dp_ori_x = pd.Series(np.array(dp_ori_x))
						dp_ori_y = pd.Series(np.array(dp_ori_y))
						me_ori_x = pd.Series(np.array(me_ori_x))
						me_ori_y = pd.Series(np.array(me_ori_y))
						distancias = pd.Series(np.array(distancias))

						num_inter = 6
						intervalo = np.amax(duracoes)-np.amin(duracoes)
						r_width = intervalo/num_inter
						if intervalo > 0.5:
							duracoes.plot.hist(grid=True, bins=10, rwidth=0.9, color='#607c8e')
							plt.title('Duração da simulação em segundos')
							plt.xlabel('Frequência')
							plt.ylabel('Tempo (s)')
							plt.grid(axis='y', alpha=0.75)
							#plt.show()
							plt.savefig('runs/time_duration.png')
							plt.clf()

						intervalo = np.amax(dp_ori_x)-np.amin(dp_ori_x)
						r_width = intervalo/num_inter
						dp_ori_x.plot.hist(grid=True, bins=10, rwidth=0.9, color='#607c8e')
						plt.title('Desvio padrão angular do tronco (X)')
						plt.xlabel('Frequência')
						plt.ylabel('Desvio padrão angular (X)')
						plt.grid(axis='y', alpha=0.75)
						#plt.show()
						plt.savefig('runs/dp_x.png')
						plt.clf()

						intervalo = np.amax(dp_ori_y)-np.amin(dp_ori_y)
						r_width = intervalo/num_inter
						dp_ori_y.plot.hist(grid=True, bins=10, rwidth=0.9, color='#607c8e')
						plt.title('Desvio padrão angular do tronco (Y)')
						plt.xlabel('Frequência')
						plt.ylabel('Desvio padrão angular (Y)')
						plt.grid(axis='y', alpha=0.75)
						#plt.show()
						plt.savefig('runs/dp_y.png')
						plt.clf()

						intervalo = np.amax(me_ori_x)-np.amin(me_ori_x)
						r_width = intervalo/num_inter
						me_ori_x.plot.hist(grid=True, bins=10, rwidth=0.9, color='#607c8e')
						plt.title('Média angular do tronco (X)')
						plt.xlabel('Frequência')
						plt.ylabel('Média angular (X)')
						plt.grid(axis='y', alpha=0.75)
						#plt.show()
						plt.savefig('runs/mean_x.png')
						plt.clf()

						intervalo = np.amax(me_ori_y)-np.amin(me_ori_y)
						r_width = intervalo/num_inter
						me_ori_y.plot.hist(grid=True, bins=10, rwidth=0.9, color='#607c8e')
						plt.title('Média angular do tronco (Y)')
						plt.xlabel('Frequência')
						plt.ylabel('Média angular (Y)')
						plt.grid(axis='y', alpha=0.75)
						#plt.show()
						plt.savefig('runs/mean_y.png')
						plt.clf()

						intervalo = np.amax(distancias)-np.amin(distancias)
						r_width = intervalo/num_inter
						distancias.plot.hist(grid=True, bins=10, rwidth=0.9, color='#607c8e')
						plt.title('Distância percorrida em X')
						plt.xlabel('Frequência')
						plt.ylabel('Distância')
						plt.grid(axis='y', alpha=0.75)
						#plt.show()
						plt.savefig('runs/dist.png')
						plt.clf()

						self.stat_writer.close()
						exit()
					self.cont_samples += 1
					#if not self.interpolando:
					#	self.levanta()
				elif self.state == 'MARCH':
					if self.deslocamentoYpelves != self.deslocamentoYpelvesMAX:
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
					elif self.deslocamentoXpes != 0:
						self.freia_frente()
					elif self.deslocamentoYpelves != 0:
						self.recuar()
					else:
						if self.activate:
							novo_estado = self.classifica_estado()
							if novo_estado != -1:
								self.state = novo_estado
				elif self.state == 'WALK':
					if self.deslocamentoYpelves != self.deslocamentoYpelvesMAX:
						self.marchar()
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
				elif self.state == 'UP':
					if self.rota_dir != 0 or self.rota_esq != 0:
						self.para_de_virar()
					elif self.deslocamentoXpes != 0:
						self.freia_frente()
					elif self.deslocamentoYpelves != 0:
						self.recuar()
					else:
						#robo pronto para levantar
						pass
				elif self.state == 'PENALIZED':
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
				#self.posiciona_gimbal()
				self.posiciona_robo()
				timer_main_loop += self.deltaTime
				time.sleep(self.simTransRate)


			except KeyboardInterrupt as e:
				rclpy.shutdown()
				self.sim_t.join()
				self.spin_t.join()
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
				#anda de lado para a esquerda
				if self.perna:
					self.anda_de_lado_esquerda()
				else:
					self.desanda_de_lado_esquerda()
			elif self.robo_yall < 90:
				#anda de lado para a direita
				if not self.perna:
					self.anda_de_lado_direita()
				else:
					self.desanda_de_lado_direita()
		if self.timer_reposiciona > self.tempoPasso*6:
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
			if (self.robo_pitch >0):
				t = threading.Thread(target=self.interpola_estados, args=[self.estados_levanta_frente, self.tempos_levanta_frente])
				t.daemon = True
				t.start()
			else:
				t = threading.Thread(target=self.interpola_estados, args=[self.estados_levanta_costas, self.tempos_levanta_costas])
				t.daemon = True
				t.start()

	def anda_de_lado_esquerda(self):
		if (not self.ladeando or self.desladeando)and self.pos_inicial_pelves[1] != self.deslocamentoYpelvesMAX/8:
			self.ladeando = True
			self.desladeando = False
			self.timer_movimentacao = 0
		if self.pos_inicial_pelves[1] != self.deslocamentoYpelvesMAX/8:
			self.timer_movimentacao += self.deltaTime
			self.pos_inicial_pelves[1] = sigmoid_deslocada(self.timer_movimentacao, self.tempoPasso)*self.deslocamentoYpelvesMAX/8
		if abs (self.pos_inicial_pelves[1] - self.deslocamentoYpelvesMAX/8) <= 0.01:
			self.pos_inicial_pelves[1] = self.deslocamentoYpelvesMAX/8
			self.ladeando = False

	def desanda_de_lado_esquerda(self):
		if (not self.desladeando or self.ladeando) and self.pos_inicial_pelves[1] != 0.:
			self.desladeando = True
			self.ladeando = False
			self.timer_movimentacao = 0
		if self.pos_inicial_pelves[1] != 0.:
			self.timer_movimentacao += self.deltaTime
			self.pos_inicial_pelves[1] = (1-sigmoid_deslocada(self.timer_movimentacao, self.tempoPasso))*self.deslocamentoYpelvesMAX/8
		if self.pos_inicial_pelves[1] <= 0.01:
			self.pos_inicial_pelves[1] = 0.
			self.desladeando = False

	def anda_de_lado_direita(self):
		if (not self.ladeando or self.desladeando)and self.pos_inicial_pelves[1] != self.deslocamentoYpelvesMAX/8:
			self.ladeando = True
			self.desladeando = False
			self.timer_movimentacao = 0
		if self.pos_inicial_pelves[1] != self.deslocamentoYpelvesMAX/8:
			self.timer_movimentacao += self.deltaTime
			self.pos_inicial_pelves[1] = -sigmoid_deslocada(self.timer_movimentacao, self.tempoPasso)*self.deslocamentoYpelvesMAX/8
		if abs (self.pos_inicial_pelves[1] - self.deslocamentoYpelvesMAX/8) <= 0.01:
			self.pos_inicial_pelves[1] = self.deslocamentoYpelvesMAX/8
			self.ladeando = False

	def desanda_de_lado_direita(self):
		if (not self.desladeando or self.ladeando) and self.pos_inicial_pelves[1] != 0.:
			self.desladeando = True
			self.ladeando = False
			self.timer_movimentacao = 0
		if self.pos_inicial_pelves[1] != 0.:
			self.timer_movimentacao += self.deltaTime
			self.pos_inicial_pelves[1] = (-1 +sigmoid_deslocada(self.timer_movimentacao, self.tempoPasso))*self.deslocamentoYpelvesMAX/8
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
		if (not self.marchando) and self.deslocamentoYpelves != self.deslocamentoYpelvesMAX:
			self.marchando = True
			self.timer_movimentacao = 0
		if self.deslocamentoYpelves != self.deslocamentoYpelvesMAX:
			self.timer_movimentacao += self.deltaTime
			self.deslocamentoZpes = sigmoid_deslocada(self.timer_movimentacao, self.tempo_marchando)*self.deslocamentoZpesMAX
			self.deslocamentoYpelves = sigmoid_deslocada(self.timer_movimentacao, self.tempo_marchando)*self.deslocamentoYpelvesMAX
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
			self.deslocamentoZpes = (1. - sigmoid_deslocada(self.timer_movimentacao, self.tempo_marchando))*self.deslocamentoZpesMAX
			self.deslocamentoYpelves = (1. - sigmoid_deslocada(self.timer_movimentacao, self.tempo_marchando))*self.deslocamentoYpelvesMAX
		if self.deslocamentoYpelves <= 0.01:
			self.deslocamentoZpes = 0
			self.deslocamentoYpelves = 0
			self.recuando = False


	#Change state
	def chage_state(self):
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

		# nEstados * [-0.5,0.5]
		# aux_estados = (x-self.N_ESTADOS/2)

		# deslocamentoXpes/2 * tgh(x)

		dif_estado = (x-self.nEstados/2)

		aux = (2*dif_estado)/50
		aux2 = ((math.exp(aux) - math.exp(- aux))/(math.exp(aux)+math.exp(-aux)))

		p1 = (self.deslocamentoXpes/2)*aux2
		pos_pelves[0] = p1
		#pos_pelves[0] = x*(self.deslocamentoXpes/self.nEstados)
		pos_pelves[1] += -self.deslocamentoYpelves*math.sin(x*math.pi/self.nEstados)

		pos_foot = self.pos_inicial_pelves[:]
		p2 = (-self.deslocamentoXpes/2)*aux2
		pos_foot[0] = p2
		pos_foot[1] += self.deslocamentoYpelves*math.sin(x*math.pi/self.nEstados)
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
	control = Controlador(time_id = 17,
						robo_id = 0,
						simulador_enable=True,
						inertial_foot_enable=False,
						gravity_compensation_enable=True)
	control.run()
