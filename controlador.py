import socket
import threading
import time
import os
import serial
import numpy as np
from functools import reduce
import struct
from std_msgs.msg import Float32MultiArray
try:
	import rospy
except Exception as e:
	print("Falha ao importar a bibliotera 'rospy'!")
import math
try:
	from Adafruit_BNO055 import BNO055
except Exception as e:
	print("Falha ao importar a bibliotera 'Adafruit_BNO055'!")




def sigmoid_deslocada(x, periodo):
	return 1./(1.+math.exp(-(12./periodo)*(x-(periodo/2.))))

class Controlador():

	def __init__(self, 
				simulador=False,
				time_id=13,
				robo_id=-1,
				altura_inicial=17.,
				tempoPasso = 0.7,
				deslocamentoYpelves = 2.2,
				deslocamentoZpes = 2.,
				deslocamentoXpes= 2.,
				deslocamentoZpelves = 30.,
				ip_rasp_visao='',
				porta_rasp_visao=24700):
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
		self.pos_inicial_pelves = [0., -0.5, altura_inicial]
		self.pos_inicial_foot = [0., 0.5, -altura_inicial]
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
		self.fps_count = 0
		self.last_time = 0
		self.count_frames = 0
		self.timer_fps = 0
		self.deltaTime = 0

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
		self.max_yall = 30
		self.min_yall = 5
		self.dist_ideal = 10

		self.robo_roll = 0
		self.robo_yall = 0
		self.robo_pitch = 0

		self.gimbal_yall = 0
		self.gimbal_pitch = 0

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
		self.angulo_vira = 4

		self.marchando = False
		self.recuando = False
		self.acelerando = False
		self.freando = False

		if self.simulador:
			self.inicia_modulo_simulador()


	def inicia_modulos(self):
		if not self.simulador:
			self.inicia_modulo_micro()

		time.sleep(3)
		self.visao_msg = [0, 0, 0, 0, 0, 1, 0]

		#self.inicia_modulo_inercial()
		self.inicia_modulo_visao()


	def inicia_modulo_inercial(self):
		print("Iniciando leitura do sensor inercial..")
		while 1:
			try:
				self.bno = BNO055.BNO055(rst=18, address=BNO055.BNO055_ADDRESS_B)
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
				i = (i+1)%10
				time.sleep(1)

	def inicia_modulo_visao(self):
		print("Abrindo Conexao com a Visao..")
		while True:
			try:
				self.visao_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
				self.visao_socket.connect((self.ip_rasp_visao, self.porta_rasp_visao))
				t = threading.Thread(target=self.escuta_visao)
				t.daemon = True
				t.start()
				break
			except Exception as e:
				print("ERRO: Nao foi possivel se conectar a rasp da visao (ip/port):   %s:%i" %(self.ip_rasp_visao, self.porta_rasp_visao))
				time.sleep(1)


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


	def escuta_inercial(self):
		print("Inercial OK!")
		self.incercial_ativado = True
		contagem_nulo = 0
		while 1:
			try:
				yall, roll, pitch = self.bno.read_euler()
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
				if contagem_nulo > 20:
					raise Exception("Bugou")

			except Exception as e:
				print("ERRO: Comunicao com o IMU foi perdida!!")
				self.incercial_ativado = True
				self.inicia_modulo_inercial()
				break

	def escuta_simulador(self):
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

	def escuta_micro(self):
		print("Micro OK!")
		self.micro_serial.flush()
		self.micro_ativado = True
		while True:
			try:
				while True:
					head, cmd, qt, checksum = struct.unpack('>h3B', self.micro_serial.read(5))
					#print(self.micro_serial.read(1))
					
					if head == self.micro_msg_head and (cmd == 1 or cmd == 4):
						if self.micro_ativado:
							cs = reduce(lambda x, y: x^y, struct.unpack('>42B', struct.pack('>2B20h', *([3, 45]+[int(i*10) for i in self.msg_to_micro]))))
							self.micro_serial.write(struct.pack('>h3B20h', *([self.micro_msg_head, 3, 45, cs]+[int(i*10) for i in self.msg_to_micro])))
						self.micro_serial.flush()
					elif head != self.micro_msg_head or qt != 46 or cmd != 2:
						cs = reduce(lambda x, y: x^y, [1, 5])
						self.micro_serial.write(struct.pack('>h3B', *([self.micro_msg_head, 1, 5, cs])))
						self.micro_serial.flush()
					else:
						data = struct.unpack('>20hB', self.micro_serial.read(41))
						cs = reduce(lambda x,y: x^y, struct.unpack('>43B', struct.pack('>2B20hB', *([cmd, qt]+[i for i in data]))))
						self.micro_serial.flush()
						if cs == checksum:
							break


				self.msg_from_micro = [i/10. for i in data]
				#print(self.msg_from_micro[0], self.msg_from_micro[1]," -> ", self.msg_from_micro[2],self.msg_from_micro[3], "( ", self.msg_from_micro[18], self.msg_from_micro[19], self.msg_from_micro[18]-self.msg_from_micro[0],self.msg_from_micro[19]-self.msg_from_micro[1]," )")
				if self.visao_ativada:
					self.visao_socket.send(("['"+self.state+"',"+str(self.msg_from_micro[18])+','+str(self.msg_from_micro[19])+']').encode())
			except Exception as e:
				print("ERRO: Conexao com o micro-controlador encerrada!")
				print(str(e))
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
				#print("Recebi um dado!")
				self.visao_msg = eval(data)
			except Exception as e:
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
			if abs(self.robo_yall - self.robo_yall_lock) < self.min_yall:
				return 2
			else:
				return -1
		elif self.state is 'MARCH':
			if self.visao_search:
				return 3
			elif self.visao_bola and abs(self.robo_yall_lock - self.robo_yall) > self.max_yall:
				return 6
			elif self.visao_bola and self.dist_bola > self.dist_ideal:
				return 8
			else:
				return -1
		elif self.state is 'WALK':
			if not self.visao_bola or self.dist_bola < self.dist_ideal or abs(self.robo_yall_lock - self.robo_yall) > self.max_yall:
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


	def posiciona_gimbal(self):
		if len(self.visao_msg) > 0:
			self.visao_bola = self.visao_msg[5] == 1
			self.visao_search = self.visao_msg[4] == 1
			self.turn90 = self.visao_msg[6]
			self.gimbal_yall_lock = (self.visao_msg[3]+self.visao_msg[1])
			self.gimbal_pitch_lock = (self.visao_msg[2]+self.visao_msg[0])
			self.msg_to_micro[18] = self.gimbal_pitch_lock
			self.msg_to_micro[19] = self.gimbal_yall_lock
			self.visao_msg = []


			#excluir
			self.gimbal_yall = self.gimbal_yall_lock
			if self.gimbal_yall < 0:
				self.gimbal_yall += 360
			self.gimbal_pitch = self.gimbal_pitch_lock


			

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

	def send_micro(self):
		if self.micro_ativado:
			self.timer_micro += self.deltaTime
			if self.timer_micro >= 1./self.frequencia_envio_micro:
				self.timer_micro = 0
				cs = reduce(lambda x, y: x^y, struct.unpack('>42B', struct.pack('>2B20h', *([3, 45]+[int(i*10) for i in self.msg_to_micro]))))
				self.micro_serial.write(struct.pack('>h3B20h', *([self.micro_msg_head,3, 45, cs]+[int(i*10) for i in self.msg_to_micro])))


	def run(self):
		#update function
		timer_main_loop = 0
		self.perna = 0
		self.rot_desvio = 0
		while (True):
			try:
				if self.state is 'MARCH':
					if self.deslocamentoZpes != self.deslocamentoZpesMAX:
						self.marchar()
					elif self.deslocamentoXpes != 0:
						self.freia_frente()
					else:
						novo_estado = self.classifica_estado()
						if novo_estado != -1:
							self.state = self.tabela_transicoes[novo_estado]
				elif self.state is 'IDDLE':
					if self.deslocamentoZpes != 0:
						self.recuar()
					else:
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
					if self.deslocamentoZpes < self.deslocamentoZpesMAX:
						self.marchar()
						self.turning90 = True
						self.gimbal_yall = self.robo_yall+90.
					elif abs(self.robo_yall-self.robo_yall_lock) > self.min_yall:
						self.vira()
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
					elif self.deslocamentoZpes != 0:
						self.recuar()
					else:
						#robo pronto para levantar
						pass



				self.atualiza_fps()
				self.chage_state()
				self.atualiza_cinematica()
				self.posiciona_gimbal()
				self.posiciona_robo()
				self.send_micro()
				#print ("%s Xpes=%.2f, Zpes=%.2f, Ypelves=%.2f, bola=%s" % (self.state, self.deslocamentoXpes, self.deslocamentoZpes, self.deslocamentoYpelves, str(self.visao_bola)))
				#print(self.msg_to_micro)
				timer_main_loop += self.deltaTime
				time.sleep(self.simTransRate)


			except KeyboardInterrupt as e:
				self.visao_socket.close()
				print("Main loop finalizado!!")
				break
			except Exception as e:
				raise e


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
		if (not self.marchando) and self.deslocamentoZpes != self.deslocamentoZpesMAX:
			self.marchando = True
			self.timer_movimentacao = 0
		if self.deslocamentoZpes != self.deslocamentoZpesMAX:
			self.timer_movimentacao += self.deltaTime
			self.deslocamentoZpes = sigmoid_deslocada(self.timer_movimentacao, self.tempo_marchando)*self.deslocamentoZpesMAX
			self.deslocamentoYpelves = sigmoid_deslocada(self.timer_movimentacao, self.tempo_marchando)*self.deslocamentoYpelvesMAX
		if abs(self.deslocamentoZpes - self.deslocamentoZpesMAX) <= 0.01:
			self.deslocamentoZpes = self.deslocamentoZpesMAX
			self.deslocamentoYpelves = self.deslocamentoYpelvesMAX
			self.marchando = False

	'''
		- Interpola deslocamento lateral da pelves e o deslocamento para cima dos pés, diminuindo estes valores até chegar em 0
	'''
	def recuar(self):
		if not self.recuando and self.deslocamentoZpes != 0:
			self.recuando = True
			self.timer_movimentacao = 0
		if self.deslocamentoZpes != 0:
			self.timer_movimentacao += self.deltaTime
			self.deslocamentoZpes = (1. - sigmoid_deslocada(self.timer_movimentacao, self.tempo_marchando))*self.deslocamentoZpesMAX
			self.deslocamentoYpelves = (1. - sigmoid_deslocada(self.timer_movimentacao, self.tempo_marchando))*self.deslocamentoYpelvesMAX
		if self.deslocamentoZpes <= 0.01:
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
		pos_pelves[1] = -self.deslocamentoYpelves*math.sin(x*math.pi/self.nEstados)

		pos_foot = self.pos_inicial_pelves[:]
		p2 = (-self.deslocamentoXpes/2)*((math.exp((2*(x-self.nEstados/2))/50) - math.exp((2*(x-self.nEstados/2))/-50))/(math.exp((2*(x-self.nEstados/2))/50)+math.exp((2*(x-self.nEstados/2))/-50)))
		pos_foot[0] = p2
		pos_foot[1] = self.deslocamentoYpelves*math.sin(x*math.pi/self.nEstados)
		pos_foot[2] = self.altura - self.deslocamentoZpes*math.exp(-((x-self.nEstados/2)**2)/600)
		return pos_pelves, pos_foot


	def atualiza_cinematica(self):
		x = (self.t_state*125)/self.tempoPasso
		ponto1, ponto2 = self.getTragectoryPoint(x)
		if self.perna:
			data_pelv = self.footToHip(ponto1)
			data_foot = self.footToHip(ponto2)
			if self.rota_dir == 1:
				data_pelv[5] = self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
				#data_pelv[4] += data_pelv[5]
				#data_pelv[0] += data_pelv[5]
			elif self.rota_dir == -1:
				data_pelv[5] = -self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			if self.rota_esq == 2:
				data_foot[5] = self.angulo_vira - (self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
				#data_foot[4] += data_pelv[5]
				#data_foot[0] += data_pelv[5]
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
				#data_pelv[4] += data_pelv[5]
				#data_pelv[0] += data_pelv[5]
			elif self.rota_esq == -1:
				data_pelv[5] =  -self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			if self.rota_dir == 2:
				data_foot[5] =  self.angulo_vira - (self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
				#data_foot[4] += data_pelv[5]
				#data_foot[0] += data_pelv[5]
			elif self.rota_dir == -2:
				data_foot[5] =  -self.angulo_vira - (-self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))		
				data_foot[5] = data_foot[5] * math.pi/180.
			else:
				data_foot[5] = 0

			data = data_foot + data_pelv + [0]*6

		self.msg_to_micro[:18] = data




if __name__ == '__main__':
	control = Controlador(robo_id = 0,ip_rasp_visao='localhost', simulador=True)
	control.run()
