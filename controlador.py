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
	from std_msgs.msg import Float32MultiArray
except Exception as e:
	pass
import receiver
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




def sigmoid_deslocada(x, periodo):
	return 1./(1.+math.exp(-(12./periodo)*(x-(periodo/2.))))

class Controlador():

	def __init__(self,
				simulador=False,
				time_id=17,
				robo_id=0,
				altura_inicial=17.,
				tempoPasso = 3.125,
				deslocamentoYpelves = 4.,
				deslocamentoZpes = 2.,
				deslocamentoXpes= 2.,
				deslocamentoZpelves = 30.,
				ip_rasp_visao='',
				porta_rasp_visao=24702):
		if (robo_id == -1):
			print("ERRO: ID do robo inválido")
			exit()

		self.ip_rasp_visao = ip_rasp_visao
		self.porta_rasp_visao = porta_rasp_visao

		self.simulador = simulador
		self.state = 'IDDLE'
		self.time_id = time_id
		self.robo_id = robo_id
		self.altura = altura_inicial
		self.pos_inicial_pelves = [0., 0.6, altura_inicial]
		self.pos_inicial_foot = [0., -0.6, -altura_inicial]
		self.deslocamentoXpes = 0.
		self.deslocamentoYpelves = 0
		self.deslocamentoZpes = 0
		self.deslocamentoZpelves = 0
		self.deslocamentoXpesMAX = deslocamentoXpes
		self.deslocamentoZpesMAX = deslocamentoZpes
		self.deslocamentoYpelvesMAX = deslocamentoYpelves
		self.deslocamentoZpelvesMAX = deslocamentoZpelves

		self.nEstados = 125
		self.tempoPasso = tempoPasso
		self.a = 10.5
		self.c = 10.2
		t = threading.Thread(target=self.inicia_modulos)
		t.daemon = True
		t.start()
		self.visao_msg = b''
		self.msg_from_micro = []
		self.msg_to_micro = [0]*20
		self.micro_msg_head = -3599
		self.micro_msg_footer = -3342
		self.fps_count = 0
		self.last_time = 0
		self.count_frames = 0
		self.timer_fps = 0
		self.deltaTime = 0
		self.ON_PIN = 25
		if RASPBERRY:
			GPIO.setup(self.ON_PIN, GPIO.IN)

		self.visao_ativada = False
		self.micro_ativado = False
		self.incercial_ativado = False
		self.simulador_ativado = False

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

		self.dist_bola = 0
		self.altura_atual = 53.5-(self.a + self.c - altura_inicial)

		self.limiar_erro_inercial = 20 # quantos graus de diferença é permitida entre entre duas leituras consecutivas

		self.tabela_transicoes = {
		1 : 'TURN90',
		2 : 'IDDLE',
		3 : 'IDDLE',
		4 : 'MARCH',
		5 : 'MARCH',
		6 : 'TURN',
		7 : 'MARCH',
		8 : 'WALK'
		}

		self.gimbal_yall_lock = 0
		self.gimbal_pitch_lock = 0
		self.robo_yall_lock = 0
		self.robo_pitch_lock = 0

		self.frequencia_envio_micro = 100 #Hz
		self.timer_micro = 0
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

		self.activate = True
		self.caiu = False

		self.rst_imu_pin = 18

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

		if self.simulador:
			self.inicia_modulo_simulador()


	def inicia_modulos(self):
		self.inicia_modulo_visao()
		if RASPBERRY:
			self.inicia_modulo_inercial()
			#self.inicia_modulo_juiz()
			self.inicia_modulo_micro()


	def inicia_modulo_juiz (self):
		"Iniciando moodulo juiz.."
		self.rec = receiver.GameStateReceiver(team=self.time_id, player=self.robo_id, control=self)
		t = threading.Thread(target=self.escuta_juiz)
		t.daemon = True
		t.start()


	def inicia_modulo_inercial(self):
		print("Iniciando leitura do sensor inercial..")
		while 1:
			try:
				self.bno = BNO055.BNO055(rst=self.rst_imu_pin, address=BNO055.BNO055_ADDRESS_B)
				if not self.bno.begin(BNO055.OPERATION_MODE_IMUPLUS):
					raise Exception("ERRO:Nao foi possivel inicializar BNO055!")
				status, self_test, error = self.bno.get_system_status()
				if status == 0x01:
					raise Exception('System error')
				t = threading.Thread(target=self.escuta_inercial)
				t.daemon = True
				t.start()
				break
			except Exception as e:
				print(str(e))
				#print("ERRO:Não foi possível inicializar BNO055!")
				time.sleep(1)



	def inicia_modulo_simulador(self):
		print("Iniciando ROS node para execcao do simulador..")
		self.pub = rospy.Publisher('Bioloid/cmd_vel', Float32MultiArray, queue_size=1)
		rospy.init_node('publisher', anonymous=True)
		self.rate = rospy.Rate(self.tempoPasso/self.nEstados)
		t = threading.Thread(target=self.escuta_simulador)
		t.daemon = True
		t.start()


	def inicia_modulo_micro(self):
		print("Abrindo Conexao com o Micro-Controlador..")
		i = 0
		while True:
			try:
				usb_path = '/dev/ttyACM'+str(i)
				self.micro_serial = serial.Serial(usb_path, 256000)
				t = threading.Thread(target=self.escuta_micro)
				t.daemon = True
				t.start()
				break
			except Exception as e:
				print("ERRO: Nao foi possivel se conectar ao micro-controlador:   %s" %(usb_path))
				i = (i+1)%5
				time.sleep(0.4)

	def inicia_modulo_visao(self):
		print("Abrindo Conexao com a Visao..")
		port_shift = -1
		while True:
			port_shift = (port_shift + 1) % 1
			try:
				self.visao_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
				self.visao_socket.connect((self.ip_rasp_visao, self.porta_rasp_visao+port_shift))
				t = threading.Thread(target=self.escuta_visao)
				t.daemon = True
				t.start()
				break
			except Exception as e:
				#print("ERRO: Nao foi possivel se conectar a rasp da visao (ip/port):   %s:%i" %(self.ip_rasp_visao, self.porta_rasp_visao))
				time.sleep(0.1)


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

	def escuta_juiz(self):
		print("Juiz OK!")
		try:
			self.rec.receive_forever()
		except Exception as e:
			print(str(e))
			self.inicia_modulo_juiz()

	def escuta_inercial(self):
		print("Inercial OK!")
		self.incercial_ativado = True
		contagem_nulo = 0
		while 1:
			try:
				yall, pitch, roll = self.bno.read_euler()
				sys, gyro, accel, mag = self.bno.get_calibration_status()
				if (sys != 2 and math.sqrt((self.robo_roll-roll)**2+(self.robo_pitch-pitch)**2+(self.robo_yall-yall)**2) > self.limiar_erro_inercial): #calibrando
					continue
				if self.robo_yall + yall + self.robo_roll + roll + self.robo_pitch + pitch == 0:
					contagem_nulo += 1
				else:
					contagem_nulo = 0
					self.robo_yall = yall
					self.robo_pitch = pitch
					self.robo_roll = roll

					if (abs(self.robo_pitch) > 45 or abs(self.robo_roll) > 45) and not self.interpolando:
						self.state = 'FALLEN'

					'''
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
					'''

					if self.visao_ativada:
						self.visao_socket.send(("['"+self.state+"',"+str(self.robo_pitch)+','+str(self.robo_yall)+']').encode())
				if contagem_nulo > 20:
					raise Exception("Bugou")

			except Exception as e:
				print("ERRO: Comunicao com o IMU foi perdida!!")
				self.incercial_ativado = True
				self.inicia_modulo_inercial()
				break

	def escuta_simulador(self):
		try:
			print("Simulador OK!")
			mat = Float32MultiArray()
			self.simulador_ativado = True
			while not rospy.is_shutdown():

				'''
				ponto1, ponto2 = getTragectoryPoint(self.timer_tragetoria)
				if flag is True:
					mat.data = footToHip(ponto1) + footToHip(ponto2) + [0]*6
					#print(np.array(mat.data[:5])*180/math.pi)
				else:
					#print("f ",ponto2[0])
					mat.data = footToHip(ponto2) + footToHip(ponto1) + [0]*6

				
				'''
				
				mat.data = self.msg_to_micro[:18]

				mat.data[10] = -mat.data[10] # quadril esquerdo ROLL
				mat.data[0] = -mat.data[0] #calcanhar direito ROLL

				mat.data[4] = -mat.data[4]
				mat.data[10] = -mat.data[10]

				#print ("%s %d" % (self.state , self.robo_yall_lock))
				self.pub.publish(mat)
				rospy.sleep(self.simTransRate)
		except Exception as e:
			pass
		

	def escuta_micro(self):
		print("Micro OK!")
		self.micro_serial.flush()
		self.micro_ativado = True
		while True:
			try:
				while True: 
					#print (self.micro_serial.read(45))
					head, checksum = struct.unpack('>hB', self.micro_serial.read(3))
					if head == self.micro_msg_head:
						Gpitch, Gyall, servo_p, servo_y, bat, footer = struct.unpack('>4h1Bh', self.micro_serial.read(11))
						cs = reduce(lambda x,y: x^y, struct.unpack('>9B', struct.pack('>4hB', *([Gpitch, Gyall, servo_p, servo_y, bat]))))
						data = [head, checksum, Gpitch, Gyall, servo_p, servo_y, bat, footer]
						self.micro_serial.flush()
						#print("cabeca certa!!", cs, checksum==cs, data)
						if cs == checksum and footer == self.micro_msg_footer:
							break
					else:
						self.micro_serial.flush()


				'''self.gimbal_pitch = Gpitch
				self.gimbal_yall = Gyall
				self.VBat = bat
				'''
				'''f = self.atualiza_fps()
				if f is not None:
					print (f)
				'''
				#
				#print("Recebi : ", [Gpitch, Gyall])
			except Exception as e:
				print("ERRO: Conexao com o micro-controlador encerrada!")
				print(str(e))
				#raise e
				self.micro_ativado = False
				self.inicia_modulo_micro()
				break

	def escuta_visao(self):
		print("Visao OK!")
		self.visao_ativada = True
		while (True):
			try:
				#print("Esperando receber dados..")
				data = self.visao_socket.recv(1000)
				if data == b'':
					raise Exception("Eu quis fechar sa porra")
				
				#Checa se o dado está correto
				cont = 0
				data_str = str(data)
				for i in data_str:
					if i == '[':
						cont += 1

				if cont > 1 or cont == 0:
					continue

				#Armazena dado em variavel do objeto para que o laço principal possa usa-la
				#tupla recebida pela visão tem o formato [angulo_vertial, angulo_horizontal, estáComABola, procurandoBola, vira90Graus]
				self.visao_msg = eval(data)
				if self.robo_yall + self.visao_msg[1] < 0:
					self.gimbal_yall = self.robo_yall + self.visao_msg[1] + 360
				elif self.robo_yall + self.visao_msg[1] > 360:
					self.gimbal_yall = (self.robo_yall + self.visao_msg[1])% 360
				else:
					self.gimbal_yall = self.robo_yall + self.visao_msg[1]
				self.gimbal_pitch = self.visao_msg[0]
				self.visao_bola = self.visao_msg[5] != 0

			except Exception as e:
				raise e
				print("ERRO: Conexao com a visao encerrada!")
				self.visao_ativada = False
				if self.visao_socket is not None:
					self.visao_socket.close()
					self.visao_socket = None
				self.inicia_modulo_visao()
				break

	def classifica_estado(self):
		if self.state is 'IDDLE':
			if self.turn90:
				return 1
			elif self.visao_bola:
				return 4
			else:
				return -1
		elif self.state is 'TURN90':
			if abs(self.robo_yall - self.robo_yall_lock) <= self.min_yall:
				return 2
			else:
				return -1
		elif self.state is 'MARCH':
			if not self.visao_bola:
				return 3
			elif self.visao_bola and abs(self.robo_yall_lock - self.robo_yall) > self.max_yall and self.visao_ativada:
				return 6
			elif self.visao_bola and (self.robo_yall >= 270 or self.robo_yall <= 90) and self.robo_pitch_lock > -45:
				return 8
			else:
				return -1
		elif self.state is 'WALK':
			if not self.visao_bola or abs(self.robo_yall_lock - self.robo_yall) > self.max_yall or self.robo_pitch_lock <= -45:
				return 7
			else:
				return -1
		elif self.state is 'TURN':
			if not self.visao_bola or abs(self.robo_yall_lock - self.robo_yall) < self.min_yall:
				return 5
			else:
				return -1
		else:
			print("ERRO: Estado invalido!!")

	'''
	def posiciona_gimbal(self):
		if len(self.visao_msg) > 0:
			self.visao_bola = self.visao_msg[5] == 1
			self.visao_search = self.visao_msg[4] == 1
			self.turn90 = self.visao_msg[6]
			self.gimbal_yall_lock = (self.visao_msg[3]+self.visao_msg[1])
			self.gimbal_pitch_lock = (self.visao_msg[2]+self.visao_msg[0])
			self.msg_to_micro[18] = self.gimbal_pitch_lock
			self.msg_to_micro[19] = self.gimbal_yall_lock
			if self.msg_to_micro[19] < 0:
				self.msg_to_micro[19] += 360
			self.visao_msg = []
			#print(self.gimbal_pitch_lock, self.gimbal_yall_lock)

			
			#excluir
			self.gimbal_yall = self.gimbal_yall_lock
			if self.gimbal_yall < 0:
				self.gimbal_yall += 360
			self.gimbal_pitch = self.gimbal_pitch_lock
			
	'''

			

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

		'''
		if self.gimbal_pitch < 0: 
			dist_virtual = self.altura_atual/math.tan(abs(self.gimbal_pitch)*math.pi/180.)
			p1 = [0., 0., self.altura_atual]
			p2 = [dist_virtual, 0., 0.]
			alfa = 30
			mat_rot_y = np.array([
				[math.cos(alfa), 0., math.sin(alfa)],
				[0., 1., 0.],
				[-math.sin(alfa), 0., math.cos(alfa)]])
			p1_ = [sum(p1*mat_rot_y[0:,0]), sum(p1*mat_rot_y[0:,1]), sum(p1*mat_rot_y[0:,2])]
			p2_ = [sum(p2*mat_rot_y[0:,0]), sum(p2*mat_rot_y[0:,1]), sum(p2*mat_rot_y[0:,2])]

			#print (p1_, p2_)
			m_ = (p1_[0]-p2_[0])/(p1_[2] - p2_[2])
			self.dist_bola = m_*(0-p2_[2])+p2_[0]

			#print (self.dist_bola, dist_virtual)
		else:
			self.dist_bola = self.dist_ideal+1
		'''

	def send_micro(self):
		try:
			if self.micro_ativado:
				self.timer_micro += self.deltaTime
				if self.timer_micro >= 1./self.frequencia_envio_micro:
					self.timer_micro = 0
					dados = [int(i*10) for i in self.msg_to_micro]
					dados[:18] = [(i/10*180./math.pi)*10 for i in dados[:18]]
					#print("Estou enviando :", dados[19])
					dados = struct.pack('>20h', *(dados))
					#print(dados)
					cs = reduce(lambda x, y: x^y, struct.unpack('>40B', dados))
					to_send = struct.pack('>hB21h', *([self.micro_msg_head, cs]+[int(i*10) for i in self.msg_to_micro]+[self.micro_msg_footer]))
					self.micro_serial.write(to_send)
					'''
					para_ler = b''
					for it in to_send:
						self.micro_serial.write(bytes([it]))
						para_ler += bytes([it])
						time.sleep(5./1000.)
					'''
					#print("Estou enviando : ", to_send)

		except Exception as e:
			self.micro_ativado = False
			raise e
		

	def run(self):
		#update function
		timer_main_loop = 0
		self.perna = 0
		self.rot_desvio = 0
		while (True):
			try:
				#print(self.activate, self.robo_yall)
				print (self.state, self.gimbal_yall, flush=True)
				if self.visao_ativada:
						self.visao_socket.send(("['"+self.state+"',"+str(50)+','+str(100)+']').encode())
				if RASPBERRY:
					#só executa oque se o dispositivo que estier rodando for a raspberry
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
						#print(self.pos_inicial_pelves)
						'''if self.perna:
							self.anda_de_lado()
						else:
							self.desanda_de_lado()
						self.robo_yall_lock = 90
						self.vira()
						'''
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.state = self.tabela_transicoes[novo_estado]
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
								self.state = self.tabela_transicoes[novo_estado]
				elif self.state is 'WALK':
					if self.deslocamentoXpes < self.deslocamentoXpesMAX:
						self.acelera_frente()
					else:
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.state = self.tabela_transicoes[novo_estado]
				elif self.state is 'TURN':
					if abs(self.robo_yall-self.robo_yall_lock) > self.min_yall:
						self.vira()
					elif self.rota_dir != 0 or self.rota_esq != 0:
						self.para_de_virar()
					else:
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.state = self.tabela_transicoes[novo_estado]
				elif self.state is 'TURN90':
					if self.deslocamentoYpelves < self.deslocamentoYpelvesMAX:
						self.marchar()
					elif abs(self.robo_yall-self.robo_yall_lock) > self.min_yall:
						self.vira()
						#print("Por favor vire o robo  %.2f no eixo yall" %(self.robo_yall-self.robo_yall_lock))
						#self.robo_yall = self.robo_yall_lock
					elif self.rota_dir != 0 or self.rota_esq != 0:
						seff.para_de_virar()
					else:
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.state = self.tabela_transicoes[novo_estado]
				elif self.state is 'UP':
					if self.rota_dir != 0 or self.rota_esq != 0:
						seff.para_de_virar()
					elif self.deslocamentoXpes != 0:
						self.freia_frente()
					elif self.deslocamentoYpelves != 0:
						self.recuar()
					else:
						#robo pronto para levantar
						pass
				elif self.state is 'PENALIZED':
					if self.rota_dir != 0 or self.rota_esq != 0:
						seff.para_de_virar()
					elif self.deslocamentoXpes != 0:
						self.freia_frente()
					elif self.deslocamentoYpelves != 0:
						self.recuar()



				self.atualiza_fps()
				self.chage_state()
				self.atualiza_cinematica()
				#self.posiciona_gimbal()
				self.posiciona_robo()
				self.send_micro()
				#print ("%s Xpes=%.2f, Zpes=%.2f, Ypelves=%.2f, bola=%s" % (self.state, self.deslocamentoXpes, self.deslocamentoYpelves, self.deslocamentoYpelves, str(self.visao_bola)))
				#print(self.msg_to_micro)
				timer_main_loop += self.deltaTime
				time.sleep(self.simTransRate)


			except Exception as e:
				self.visao_socket.close()
				print("Main loop finalizado!!")
				raise e
				break

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
			

	'''
		- Começa a virar
	'''
	def vira(self):
		if self.robo_yall_lock < 0:
			self.rot_desvio = 1
		else:
			self.rot_desvio = -1

	'''
		- Vai parando de virar pelo tempo definido no construtor
	'''
	def para_de_virar(self):
		self.rot_desvio = 0


	'''
		- Interpola distância de deslocamento dos pés, da atual até o max setado no contrutor
	'''
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


	'''
		- Interpola distância de deslocamento dos pés, diminuindo este valor até que se torne 0
	'''
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

	'''
		- Interpola deslocamento lateral da pelves e o deslocamento para cima dos pés, da atual até o max
	'''
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

	'''
		- Interpola deslocamento lateral da pelves e o deslocamento para cima dos pés, diminuindo estes valores até chegar em 0
	'''
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
		self.t_state += self.deltaTime
		if self.t_state >= self.tempoPasso:
			self.t_state = 0
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

	'''
		- Retorna os 6 angulos de da perna, calculando a cinematica inversa. Considerando o pé como base e o quadril como ponto variável
	'''
	def footToHip(self, pointHip):
		angulos = []
		x,y,z = pointHip

		#ankle roll
		theta = math.atan(y/z)
		angulos.append(theta)

		#ankle pitch
		b = math.sqrt(x**2+y**2+z**2)
		anguloA = math.acos((self.a**2-(b**2+self.c**2))/(-2*b*self.c))
		betha = math.atan(x/z)
		anguloA = betha + anguloA
		angulos.append(anguloA)

		#knee
		anguloB = math.acos((b**2-(self.a**2+self.c**2))/(-2*self.a*self.c))
		anguloB = anguloB - math.pi
		angulos.append(anguloB)

		#hip pitch
		anguloC = math.acos((self.c**2-(self.a**2+b**2))/(-2*self.a*b))
		anguloC = anguloC - betha
		angulos.append(anguloC)

		#hip roll
		angulos.append(theta)

		#hip yall
		angulos.append(0)

		return angulos


	'''
		- Pega o proximo "estado" da função de tragetória, a função de tragetória muda de acordo com as variaveis que definem o deslocamento e rotação do robô
	'''
	def getTragectoryPoint(self, x):
		pos_pelves = self.pos_inicial_pelves[:]
		p1 = (self.deslocamentoXpes/2)*((math.exp((2*(x-self.nEstados/2))/50) - math.exp((2*(x-self.nEstados/2))/-50))/(math.exp((2*(x-self.nEstados/2))/50)+math.exp((2*(x-self.nEstados/2))/-50)))
		pos_pelves[0] = p1
		pos_pelves[1] += -self.deslocamentoYpelves*math.sin(x*math.pi/self.nEstados)

		pos_foot = self.pos_inicial_pelves[:]
		#pos_foot[1] *= -1
		p2 = (-self.deslocamentoXpes/2)*((math.exp((2*(x-self.nEstados/2))/50) - math.exp((2*(x-self.nEstados/2))/-50))/(math.exp((2*(x-self.nEstados/2))/50)+math.exp((2*(x-self.nEstados/2))/-50)))
		pos_foot[0] = p2
		pos_foot[1] += self.deslocamentoYpelves*math.sin(x*math.pi/self.nEstados)
		pos_foot[2] = self.altura - self.deslocamentoZpes*math.exp(-((x-self.nEstados/2)**2)/600)
		return pos_pelves, pos_foot


	def interpola_estados(self, estados, tempos):
		if len(estados) > 0:
			p_ant = estados[0]
		else:
			return
		for i in range(1, len(estados)):
			p_atual = s[i]
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
		ponto1, ponto2 = self.getTragectoryPoint(x)
		if self.perna:
			data_pelv = self.footToHip(ponto1)
			data_foot = self.footToHip(ponto2)
			if self.rota_dir == 1:
				data_pelv[5] = self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			elif self.rota_dir == -1:
				data_pelv[5] = -self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			if self.rota_esq == 2:
				data_foot[5] = self.angulo_vira - (self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			elif self.rota_esq == -2:
				data_foot[5] = -self.angulo_vira - (-self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			else:
				data_foot[5] = 0

			data = data_pelv + data_foot + [0]*6
		else:
			data_pelv = self.footToHip(ponto1)
			data_foot = self.footToHip(ponto2)
			if self.rota_esq == 1:
				data_pelv[5] =  self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			elif self.rota_esq == -1:
				data_pelv[5] =  -self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			if self.rota_dir == 2:
				data_foot[5] =  self.angulo_vira - (self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			elif self.rota_dir == -2:
				data_foot[5] =  -self.angulo_vira - (-self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))		
				data_foot[5] = data_foot[5] * math.pi/180.
			else:
				data_foot[5] = 0

			data = data_foot + data_pelv + [0]*6

		self.msg_to_micro[:18] = data




if __name__ == '__main__':
	control = Controlador(time_id = 17,robo_id = 0,ip_rasp_visao='localhost', simulador=True)
	control.run()