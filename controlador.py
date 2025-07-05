# -*- coding:utf-8 -*-
import threading
import time
import numpy as np
from robot import Robo
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
from enum import Enum


class OrientationMode(Enum):
    VISAO = 1
    POSICAO_ALVO = 2

class Controlador():

	def __init__(self,
				simulador_enable=False,
				gravity_compensation_enable = False,
				orientation_mode=OrientationMode.POSICAO_ALVO):

		self.simulador = simulador_enable
		self.orientation_mode = orientation_mode
		self.state = 'IDDLE'
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

		self.robo = Robo(gravity_compensation_enable=gravity_compensation_enable)

		self.msg_to_micro = [0]*20

		self.ON_PIN = 25
		if RASPBERRY:
			GPIO.setup(self.ON_PIN, GPIO.IN)

		self.simTransRate = 0.008
		# self.robo.nEstados = (self.robo.tempoPasso / self.simTransRate)

		self.chegou_no_alvo = True
		self.turn90 = False
		self.max_yall = 20
		self.min_yall = 5

		self.robo_roll = 0
		self.robo_yall = 0
		self.robo_pitch = 0

		self.gimbal_yall = 0
		self.gimbal_pitch = -45

		self.robo_yall_lock = 0
		self.robo_pitch_lock = 0
		self.Lfoot_orientation = [0,0,0]
		self.Rfoot_orientation = [0,0,0]

		self.Lfoot_press = [0,0,0,0]
		self.Rfoot_press = [0,0,0,0]
		self.total_press = 0

		self.activate = True
		self.rst_imu_pin = 18

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

		self.rate = node.create_rate(1/(self.robo.tempoPasso/self.robo.tPasso))
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
			while rclpy.ok():
				data = (np.array(self.msg_to_micro[:19])*(1800/np.pi)).astype(np.int16).tolist()
				data[18] = self.state_encoder[self.state]
				mat.data = data

				mat.data[0] = -mat.data[0] #calcanhar direito ROLL

				mat.data[4] = -mat.data[4]

				mat.data[self.robo.RIGHT_HIP_PITCH] += 150
				mat.data[self.robo.LEFT_HIP_PITCH] += 150

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


	def is_fallen(self):
		return abs(self.robo_pitch) > 45 or abs(self.robo_roll) > 45

	# 	Leitura IMU - robo
	def robot_inertial_callback(self, msg):
		self.robo_yall = msg.data[2]
		self.robo_pitch = msg.data[1]
		self.robo_roll = msg.data[0]
		if self.is_fallen() and not self.robo.levantando:
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
			if self.chegou_no_alvo or abs(self.robo_yall_lock) <= self.min_yall:
				return 'MARCH'
			else:
				return -1
		elif self.state == 'MARCH':
			if self.chegou_no_alvo:
				return 'IDDLE'
			elif self.turn90:
				return 'TURN90'
			elif abs(self.robo_yall_lock) > self.max_yall:
				return 'TURN'
			elif self.robo_pitch_lock > -45:
				return 'WALK'
			else:
				return -1
		elif self.state == 'WALK':
			if self.chegou_no_alvo or self.robo_pitch_lock <= -45:
				return 'MARCH'
			elif abs(self.robo_yall_lock) > self.max_yall:
				return 'TURN'
			else:
				return -1
		elif self.state == 'TURN':
			if self.chegou_no_alvo or abs(self.robo_yall_lock) < self.min_yall:
				return 'MARCH'
			else:
				return -1
		elif self.state == 'FALLEN':
			if not self.is_fallen():
				return 'IDDLE'
			else:
				return -1
		else:
			print("ERRO: Estado invalido!!")

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

		self.distancia_ponto_alvo = math.sqrt((self.posicao_alvo[0] - self.posicao_robo[0])**2 + (self.posicao_alvo[1] - self.posicao_robo[1])**2)
		if (self.distancia_ponto_alvo > 0.1):
			self.chegou_no_alvo = False
			self.robo_pitch_lock = 0
		else:
			self.chegou_no_alvo = True
			self.robo_pitch_lock = -45

	def run(self):
		#update function
		timer_main_loop = 0

		def atualiza_estado():
			novo_estado = self.classifica_estado()
			if novo_estado != -1:
				self.state = novo_estado

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
					self.robo.levanta(caido_de_frente=self.robo_pitch > 0, callback=atualiza_estado)
				elif self.state == 'IDDLE':
					if self.robo.esta_virando():
						self.robo.para_de_virar()
					if self.robo.esta_andando():
						self.robo.freia_frente()
					elif self.robo.esta_marchando():
						self.robo.para_de_machar()
					else:
						if self.activate:
							atualiza_estado()
				elif self.state == 'MARCH':
					if not self.robo.esta_levantando_o_pe():
						self.robo.marchar()
					elif self.robo.esta_andando():
						self.robo.freia_frente()
					else:
						atualiza_estado()
				elif self.state == 'WALK':
					if not self.robo.esta_andando_velocidade_max():
						self.robo.acelera_frente()
					else:
						atualiza_estado()
				elif self.state == 'TURN':
					if not self.chegou_no_alvo and abs(self.robo_yall_lock) > self.min_yall:
						self.robo.vira(esquerda=self.robo_yall_lock < 0)
					elif self.robo.esta_virando():
						self.robo.para_de_virar()
					else:
						atualiza_estado()
				elif self.state == 'TURN90':
					if not self.robo.esta_levantando_o_pe():
						self.robo.marchar()
					elif abs(self.robo_yall_lock) > self.min_yall:
						self.robo.vira(esquerda=self.robo_yall_lock < 0)
					elif self.robo.esta_virando():
						self.robo.para_de_virar()
					else:
						atualiza_estado()
						if self.state != 'TURN90':
							self.turn90 = False
				elif self.state == 'UP' or self.state == 'PENALIZED':
					if self.robo.esta_virando():
						self.robo.para_de_virar()
					elif self.robo.esta_andando():
						self.robo.freia_frente()
					elif self.robo.esta_marchando():
						self.robo.para_de_machar()
					else:
						#robo pronto para novo estado
						pass

				self.msg_to_micro = self.robo.computa_angulos(current_state=self.state)

				if (self.orientation_mode == OrientationMode.VISAO):
					self.compura_direcao_pelo_gimbal()
				elif (self.orientation_mode) == OrientationMode.POSICAO_ALVO:
					self.computa_direcao_pela_posicao_alvo()
				
				timer_main_loop += self.robo.deltaTime
				time.sleep(self.simTransRate)

				# print(self.robo.fps_count)


			except KeyboardInterrupt as e:
				print("Main loop finalizado!!")
				rclpy.shutdown()
				self.sim_t.join()
				break
			except Exception as e:
				raise e



if __name__ == '__main__':
	control = Controlador(simulador_enable=True,
						gravity_compensation_enable=True)
	control.run()
