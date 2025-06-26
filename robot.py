# -*- coding:utf-8 -*-
import numpy as np
from body_solver import Body
import math
import threading
import csv
import time


RAD_TO_DEG = 180 / np.pi
DEG_TO_RAD = np.pi / 180.
KP_CONST = 0.3


def sigmoid_deslocada(x, periodo, inc=12.):
	return 1./(1.+math.exp(-(inc/periodo)*(x-(periodo/2.))))

class Robo():

	def __init__(self,
				altura_inicial=17.,
				tempo_passo = 0.35, # 0.35
				deslocamento_ypelves = 3.8, # 3.8
				deslocamento_zpes = 3., # 3.
				deslocamento_xpes= 2.5, # 2.5
				anguloViraMAX = 15., # 15.
				inertial_foot_enable = False,
				gravity_compensation_enable = False):
			
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
		self.anguloViraMAX = anguloViraMAX
		self.influencia_gravity_compensation = 10 # quanto maior o valor menos influencia o GC terá
		self.fator_deslocamento_lateral_pe_balanco = 1.1 # 1 representa nenhuma influencia, ou nenhum deslocamento lateral adicional do pé de balanço

		self.nEstados = 125
		self.tPasso = self.nEstados/2
		self.tempoPasso = tempo_passo
		self.a = 10.5
		self.c = 10.2
		
		self.incTanh = 0.3 # utilizado para controlar o deslocamento frontal, e rotação. Quando menor este valor, mais tarde o bipede realizará o movimento
		self.incSino = 0.4 # utilizado para controlar o deslocamento em z do pé de balanço. Quando menor este valor, mais tarde o bipede realizará o movimento

		self.angulos = [0]*20

		self.fps_count = 0
		self.last_time = 0
		self.count_frames = 0
		self.timer_fps = 0
		self.deltaTime = 0
		
		self.time_ignore_GC = 0.1 # entre 0 e 1 - porcentagem de tempo para ignorar o gravity compensation

		self.tempo_acelerando = 4.
		self.tempo_marchando = 4.
		self.tempo_virando = 3.

		self.Lfoot_orientation = [0,0,0]
		self.Rfoot_orientation = [0,0,0]

		self.inertial_foot_enable = inertial_foot_enable
		self.gravity_compensation_enable = gravity_compensation_enable
		
		self.Lfoot_press = [0,0,0,0]
		self.Rfoot_press = [0,0,0,0]
		self.total_press = 0

		self.t_state = 0
		self.rot_desvio = 0
		self.rota_dir = 0
		self.rota_esq = 0

		self.marchando = False
		self.recuando = False
		self.acelerando = False
		self.freando = False
		self.levantando = False

		# Perna no chão: 1 = direita; 0 = esquerda
		self.perna = 0

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

	def gravity_compensation(self, current_state):
		#return
		if not self.gravity_compensation_enable or (self.t_state < self.tempoPasso/2 and self.t_state < self.tempoPasso*self.time_ignore_GC) or (self.t_state >= self.tempoPasso/2 and self.t_state > self.tempoPasso*(1-self.time_ignore_GC)) or (current_state == "IDDLE" and not self.recuando and not self.freando):
			return

		torques = self.body.get_torque_in_joint(self.perna,[2,5])

		dQ = (np.array(torques)/KP_CONST)/self.influencia_gravity_compensation
		dQ *= math.sin(self.t_state*math.pi/self.tempoPasso)
		dQ *= (self.deslocamentoZpes / self.deslocamentoZpesMAX)

		if self.perna:
			self.angulos[self.RIGHT_ANKLE_PITCH] += dQ[0]
			self.angulos[self.RIGHT_HIP_ROLL] += (dQ[1]*-1)
		else:
			self.angulos[self.LEFT_ANKLE_PITCH] += dQ[0]
			self.angulos[self.LEFT_HIP_ROLL] += dQ[1]
		
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

	def levanta(self, caido_de_frente=True, callback=None):
		if not self.levantando:
			self.levantando = True
			if (caido_de_frente):
				t = threading.Thread(target=self.interpola_estados, args=[self.estados_levanta_frente, self.tempos_levanta_frente, callback])
				t.daemon = True
				t.start()
			else:
				t = threading.Thread(target=self.interpola_estados, args=[self.estados_levanta_costas, self.tempos_levanta_costas, callback])
				t.daemon = True
				t.start()

	# 	'''
	# 		- Define para qual lado o robô deve virar com base no yall lock
	# 	'''
	def vira(self, esquerda):
		if esquerda:
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
				if self.rot_desvio > 0: # vira para a esquerda
					if self.perna: # perna direita no chão
						self.rota_dir = -1
						self.rota_esq *= 2
					else: # perna esquerda no chão
						self.rota_esq = -1
						self.rota_dir *= 2
				else: # vira para a direita
					if self.perna: # perna direita no chão
						self.rota_dir = 1
						self.rota_esq *= 2
					else: # perna esquerda no chão
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

		dif_estado = (x-self.tPasso)

		aux = dif_estado/(self.nEstados*self.incTanh)
		aux2 = (math.exp(aux) - math.exp(-aux))/(math.exp(aux)+math.exp(-aux))

		pHx = (self.deslocamentoXpes/2)*aux2
		pHy = -self.deslocamentoYpelves*math.sin(x*math.pi/self.nEstados)
		pos_pelves[0] = pHx
		pos_pelves[1] += pHy

		pos_foot = self.pos_inicial_pelves[:]
		pFx = (-self.deslocamentoXpes/2)*aux2
		pos_foot[0] = pFx
		pos_foot[1] += -pHy*self.fator_deslocamento_lateral_pe_balanco
		pos_foot[2] = self.altura - self.deslocamentoZpes*math.exp(-(dif_estado**2)/(self.tPasso*self.incSino)**2)
		return pos_pelves, pos_foot

	# interpolação simples entre estados
	def interpola_estados(self, estados, tempos, callback):
		if len(estados) > 0:
			p_ant = estados[0]
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
						self.angulos[j] = m[j]*timer + p_ant[j]
		
		if callback != None:
			callback()
		self.levantando = False


	def atualiza_cinematica(self):
		x = (self.t_state*self.nEstados)/self.tempoPasso
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
	
			xTanH = (x-self.nEstados/2)/(self.nEstados*self.incTanh)
			tanH = (np.exp(xTanH) - np.exp(-xTanH)) / (np.exp(xTanH) + np.exp(-xTanH))
			#ROTINHA PARA PRODUZIR MOVIMENTO DE ROTAÇÃO NA PERNA DIREITA (VIRANDO PARA A ESQUERDA)
			if self.rota_dir == -1:
				data_pelv[5] = -self.anguloViraMAX/2. - self.anguloViraMAX/2.* tanH
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			#ROTINHA PARA RESETAR PERNA ESQUERDA (CASO ESTEJA VIRANDO PARA A DIREITA)
			if self.rota_esq == 2:
				data_foot[5] = self.anguloViraMAX - (self.anguloViraMAX/2. + self.anguloViraMAX/2. * tanH)
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

			#ROTINHA PARA PRODUZIR MOVIMENTO DE ROTAÇÃO NA PERNA ESQUERDA (VIRANDO PARA A DIREITA)
			if self.rota_esq == 1:
				data_pelv[5] =  self.anguloViraMAX/2. + self.anguloViraMAX/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			#ROTINHA PARA RESETAR PERNA DIREITA (CASO ESTEJA VIRANDO PARA A ESQUERDA)
			if self.rota_dir == -2:
				data_foot[5] =  -self.anguloViraMAX - (-self.anguloViraMAX/2. - self.anguloViraMAX/2.*((np.exp((2*(x-self.nEstados/2))/50) - np.exp((2*(x-self.nEstados/2))/-50))/(np.exp((2*(x-self.nEstados/2))/50)+np.exp((2*(x-self.nEstados/2))/-50))))		
				data_foot[5] = data_foot[5] * math.pi/180.
			else:
				data_foot[5] = 0

			#PÉ ESQUERDO ESTÁ EM CONTATO COM O CHÃO E PÉ DIREITO ESTÁ SE MOVENDO.
			data = data_foot + data_pelv + [0]*6

			#CONFIGURA BODY SOLVER PARA INVOCAR FUNÇÕES DO MODELO DINÂMICO DO ROBÔ
			self.body.set_angles(self.perna, data_foot, data_pelv)

		self.angulos[:18] = data

	def esta_andando(self):
		return self.deslocamentoXpes != 0
	
	def esta_andando_velocidade_max(self):
		return self.deslocamentoXpes >= self.deslocamentoXpesMAX
	
	def esta_marchando(self):
		return self.deslocamentoYpelves != 0
	
	def esta_levantando_o_pe(self):
		return self.deslocamentoZpes == self.deslocamentoZpesMAX
	
	def esta_virando(self):
		return self.rota_dir != 0 or self.rota_esq != 0
	

	def computa_angulos(self, current_state):
		self.atualiza_fps()
		self.atualiza_tempo_marcha()
		self.atualiza_cinematica()
		self.gravity_compensation(current_state)
		self.calcula_centro_pressao()

		return self.angulos
