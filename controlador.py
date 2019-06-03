import time
import numpy as np
try:
	import rospy
	from std_msgs.msg import Float32MultiArray
except Exception as e:
	print("Falha ao importar módulos ROS!")
import math
from body_solver import Body
from parameters import *


def sigmoid_deslocada(x, periodo):
	return 1./(1.+math.exp(-(12./periodo)*(x-(periodo/2.))))

class Controlador():

	def __init__(self,
				pub,
				pub_rate,
				gravity_compensation_enable = False):
		
		self.reset()
		self.a = UPPER_LEG_LENGHT
		self.c = LOWER_LEG_LENGHT
		self.body_angles = [0]*18
		self.fps_count = 0
		self.last_time = 0
		self.count_frames = 0
		self.timer_fps = 0
		self.time_ignore_GC = TIME_TO_IGNORE_GC #entre 0 e 1 - gravity compensation rodará no intervalo (tempoPasso*time_ignore_GC, tempoPasso*(1-time_ignore_GC))
		self.pos_pub = pub
		self.pub_rate = pub_rate
		self.gravity_compensation_enable = gravity_compensation_enable
		self.body = Body()
		self.inicia_modulo_simulador()


	def reset(self):
		#variaveis do controlador marcos
		self.altura = ALTURA_INICIAL
		self.pos_inicial_pelves = [0., DISTANCIA_PES_INICIAL/2, ALTURA_INICIAL]
		self.pos_inicial_foot = [0., DISTANCIA_PES_INICIAL/2, ALTURA_INICIAL]
		self.deslocamentoXpes = SHIFT_X_FOOT_INIT
		self.deslocamentoYpelves = SHIFT_Y_HIP_INIT
		self.deslocamentoZpes = SHIFT_Z_FOOT_INIT
		self.deslocamentoZpelves = ANGLE_Z_HIP_INIT
		self.deslocamentoXpesMAX = SHIFT_X_FOOT_MAX
		self.deslocamentoZpesMAX = SHIFT_Z_FOOT_MAX
		self.deslocamentoYpelvesMAX = SHIFT_Y_HIP_MAX
		self.angulo_vira = ANGLE_Z_HIP_MAX
		self.tempoPasso = TIME_STEP_INIT
		self.foot_point_last = np.array(pos_inicial_foot)
		self.pelv_point_last = np.array(pos_inicial_pelves)
		self.torso_point_last = np.array([0., 0., 1.])

		self.t_state = 0
		self.rota_dir = 0
		self.rota_esq = 0
		self.deltaTime = 0

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


	def gravity_compensation(self):
		#return
		if not self.gravity_compensation_enable or (self.t_state < self.tempoPasso/2 and self.t_state < self.tempoPasso*self.time_ignore_GC) or (self.t_state >= self.tempoPasso/2 and self.t_state > self.tempoPasso*(1-self.time_ignore_GC)) or self.deslocamentoYpelves != self.deslocamentoYpelvesMAX or self.deslocamentoZpes == 0:
			return

		torques = self.body.get_torque_in_joint(self.perna,[3,5])

		dQ = (np.array(torques)/KP_CONST)/15
		#print(dQ[1], self.perna)

		dQ *= math.sin(self.t_state*math.pi/self.tempoPasso)
		if self.perna:
			self.body_angles[2] += dQ[0]
			self.body_angles[4] += (dQ[1]*-1)
		else:
			self.body_angles[6+2] += dQ[0]
			self.body_angles[6+4] += dQ[1]

	def run(self):
		#update function
		timer_main_loop = 0
		self.perna = 0
		self.rot_desvio = 0
		while (True):
			try:
				self.atualiza_fps()
				self.chage_state()
				self.atualiza_cinematica()
				self.gravity_compensation()

				timer_main_loop += self.deltaTime
				time.sleep(self.simTransRate)
			except KeyboardInterrupt as e:
				print("Main loop finalizado!!")
				break
			except Exception as e:
				raise e


	def step(self, action):
		action[:9] = np.array(action)[:9]*VELOCITY_POINTS_MAX

		foot_v = np.array(action[0:3])
		pelv_v = np.array(action[3:6])
		torso_v = np.array(action[6:9])

		foot_p = self.foot_point_last
		pelv_p = self.pelv_point_last
		torso_p = self.torso_point_last

		ant_t = time.time()
		timer = 0.
		while(timer < TIME_STEP_ACTION):
			atual_t = time.time()
			dt = (atual_t - ant_t)
			timer += dt
			
			att_foot_p = foot_v*dt + foot_p
			att_pelv_p = pelv_v*dt + pelv_p
			att_torso_p = torso_v*dt + torso_p

			##### se dist entre att_foot_p e (0,0,0) for maior que a+c : normalize para que a dist seja = a+c #######
			try:
				angles = self.cinematica_inversa(att_pelv_p, att_foot_p, att_torso_p)
			except Exception as e:
				pass
			else:
				foot_p = att_foot_p
				pelv_p = att_pelv_p
				att_torso_p = torso_p
				self.body_angles = angles

			mat = Float32MultiArray()
			mat.data = self.body_angles
			self.pos_pub.publish(mat)
			ant_t = atual_t
			self.pub_rate.sleep()

		self.foot_point_last = foot_p
		self.pelv_point_last = pelv_p
		self.torso_point_last = torso_p
		return self.get_state()
			

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
		Entrada: tempo float/int t
		Saída: 2 vetores de 3 posições (x,y,z). O primeiro indica a posição da pelves considerando o pé em contato com o chão como base, 
			   o segundo vetor indica a posição do pé de balanço considerando a pelves do pé de balanço como base.
	'''
	def getTragectoryPoint(self, x):
		pos_pelves = self.pos_inicial_pelves[:]
		p1 = (self.deslocamentoXpes/2)*((math.exp((2*(x-125/2))/50) - math.exp((2*(x-125/2))/-50))/(math.exp((2*(x-125/2))/50)+math.exp((2*(x-125/2))/-50)))
		pos_pelves[0] = p1
		pos_pelves[1] += -self.deslocamentoYpelves*math.sin(x*math.pi/125)

		pos_foot = self.pos_inicial_pelves[:]
		p2 = (-self.deslocamentoXpes/2)*((math.exp((2*(x-125/2))/50) - math.exp((2*(x-125/2))/-50))/(math.exp((2*(x-125/2))/50)+math.exp((2*(x-125/2))/-50)))
		pos_foot[0] = p2
		pos_foot[1] += self.deslocamentoYpelves*math.sin(x*math.pi/125)
		pos_foot[2] = self.altura - self.deslocamentoZpes*math.exp(-((x-125/2)**2)/600)
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
					self.body_angles[j] = m[j]*timer + p_ant[j]


	def get_state(self):
		return np.array([0]*10)

	def cinematica_inversa(self, pelv_point, foot_point, torso_point):
		return np.array([0]*18)


	def atualiza_cinematica(self):
		x = (self.t_state*125)/self.tempoPasso
		pelv_point, foot_point = self.getTragectoryPoint(x)
		if self.perna:
			#CINEMÁTICA INVERSA
			data_pelv = self.footToHip(pelv_point)
			data_foot = self.footToHip(foot_point)
		
			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A ESQUERDA
			if self.rota_dir == 1:
				data_pelv[5] = self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			elif self.rota_dir == -1:
				data_pelv[5] = -self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A DIREITA
			if self.rota_esq == 2:
				data_foot[5] = self.angulo_vira - (self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			elif self.rota_esq == -2:
				data_foot[5] = -self.angulo_vira - (-self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			else:
				data_foot[5] = 0

			#PÉ DIREITO ESTÁ EM CONTATO COM O CHÃO E PÉ ESQUERDO ESTÁ SE MOVENDO.
			data_pelv[0] *= -1
			data_pelv[4] *= -1
			data = data_pelv + data_foot + [0]*6

			#CONFIGURA BODY SOLVER PARA INVOCAR FUNÇÕES DO MODELO DINÂMICO DO ROBÔ
			self.body.set_angles(self.perna, data_pelv, data_foot)
		else:
			#CINEMÁTICA INVERSA
			data_pelv = self.footToHip(pelv_point)
			data_foot = self.footToHip(foot_point)

			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A ESQUERDA
			if self.rota_esq == 1:
				data_pelv[5] =  self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			elif self.rota_esq == -1:
				data_pelv[5] =  -self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A DIREITA
			if self.rota_dir == 2:
				data_foot[5] =  self.angulo_vira - (self.angulo_vira/2. + self.angulo_vira/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			elif self.rota_dir == -2:
				data_foot[5] =  -self.angulo_vira - (-self.angulo_vira/2. - self.angulo_vira/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50))))		
				data_foot[5] = data_foot[5] * math.pi/180.
			else:
				data_foot[5] = 0

			#PÉ ESQUERDO ESTÁ EM CONTATO COM O CHÃO E PÉ DIREITO ESTÁ SE MOVENDO.
			data_foot[0] *= -1
			data_foot[4] *= -1
			data = data_foot + data_pelv + [0]*6

			#CONFIGURA BODY SOLVER PARA INVOCAR FUNÇÕES DO MODELO DINÂMICO DO ROBÔ
			self.body.set_angles(self.perna, data_foot, data_pelv)

		self.body_angles[:18] = data


if __name__ == '__main__':
	control = Controlador(gravity_compensation_enable=True)
	control.run()