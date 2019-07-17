# coding: utf-8
import time
import numpy as np
try:
	import rospy
	from std_msgs.msg import Float32MultiArray
	from geometry_msgs.msg import Vector3
except Exception as e:
	print("Falha ao importar módulos ROS!")
import math
from body_solver import Body
from parameters import *
import threading as mt
import os


def sigmoid_deslocada(x, periodo):
	return 1./(1.+math.exp(-(12./periodo)*(x-(periodo/2.))))

class Controlador():

	def __init__(self,
				idx,
				pub,
				pub_rate,
				t_ori,
				t_acc,
				t_pos,
				t_joint,
				t_force,
				gravity_compensation_enable = False):
		
		simu_name_id = 'w%i' % idx
		self.w_id = idx
		self.a = UPPER_LEG_LENGHT
		self.c = LOWER_LEG_LENGHT
		self.deslocamentoXpesMAX = SHIFT_X_FOOT_MAX
		self.deslocamentoZpesMAX = SHIFT_Z_FOOT_MAX
		self.deslocamentoYpelvesMAX = SHIFT_Y_HIP_MAX
		self.body_angles = [0]*19
		self.fps_count = 0
		self.last_time = 0
		self.count_frames = 0
		self.timer_fps = 0
		self.cg = 15
		self.time_ignore_GC = TIME_TO_IGNORE_GC #entre 0 e 1 - gravity compensation rodará no intervalo (tempoPasso*time_ignore_GC, tempoPasso*(1-time_ignore_GC))
		self.gravity_compensation_enable = gravity_compensation_enable
		self.body = Body()
		self.pub_queue = pub
		self.pub_rate = pub_rate

		self.t_acc_shd = t_acc 	# accelerometer
		self.t_ori_shd = t_ori 	# IMU
		self.t_pos_shd = t_pos	    # position (X Y), odometry
		self.t_joint_shd = t_joint
		self.t_force_shd = t_force
		
		self.reset()
		
		time.sleep(TIME_WAIT_INIT_PUBS)
		#define subscribers para os dados do state
		if not TESTING:
			rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+"/t_acc_last", Vector3, self.t_acc_last_callback)
			rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+"/t_ori_last", Vector3, self.t_ori_last_callback)
			rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+"/t_pos_last", Vector3, self.t_pos_last_callback)
			rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+"/t_joint_last", Float32MultiArray, self.t_joint_last_callback)
			rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+"/t_force_last", Float32MultiArray, self.t_force_last_callback)
		else:
			rospy.Subscriber("vrep_ros_interface/"+simu_name_id+"/t_acc_last", Vector3, self.t_acc_last_callback)
			rospy.Subscriber("vrep_ros_interface/"+simu_name_id+"/t_ori_last", Vector3, self.t_ori_last_callback)
			rospy.Subscriber("vrep_ros_interface/"+simu_name_id+"/t_pos_last", Vector3, self.t_pos_last_callback)
			rospy.Subscriber("vrep_ros_interface/"+simu_name_id+"/t_joint_last", Float32MultiArray, self.t_joint_last_callback)
			rospy.Subscriber("vrep_ros_interface/"+simu_name_id+"/t_force_last", Float32MultiArray, self.t_force_last_callback)

		'''
		t = mt.Thread(target=run_marcos_controller)
		t.start()
		'''

	def reset(self):
		#dados que vem do simulador
		self.t_acc_last = np.array([0.]*3)
		self.t_ori_last = np.array([0.]*3)
		self.t_pos_last = np.array([0.]*2)
		self.t_force_last = np.array([0.125]*8)

		#variaveis do controlador marcos
		self.altura = HEIGHT_INIT
		self.pos_inicial_pelves = [0., DISTANCE_FOOT_INIT/2, HEIGHT_INIT]
		self.pos_inicial_foot = [0., DISTANCE_FOOT_INIT/2, HEIGHT_INIT]
		self.deslocamentoXpes = SHIFT_X_FOOT_INIT
		self.deslocamentoYpelves = SHIFT_Y_HIP_INIT
		self.deslocamentoZpes = SHIFT_Z_FOOT_INIT
		self.deslocamentoZpelves = ANGLE_Z_HIP_INIT
		self.tempoPasso = TIME_STEP_INIT

		#variaveis que a rede usa para executar a ação
		self.r_point_last = np.array(self.pos_inicial_foot)
		self.l_point_last = np.array(self.pos_inicial_pelves)
		self.t_angles_last = np.array([0., 0.])
		self.lz_angles_last = np.array([0., 0.])
		#self.pos_target = (np.random.rand(2)*2-np.array([1, 1]))*TARGET_BOUND_RANGE  # posição alvo definida neste episódio
		#self.pos_target = (self.t_pos_last-self.pos_target)/np.linalg.norm(self.t_pos_last-self.pos_target)
		self.pos_target = np.array([1.,0.])

		self.action_last = np.array([0.]*N_A)
		self.done = False
		self.perna = 0
		self.t_state = 0
		self.rot_desvio = 0
		self.rota_dir = 0
		self.rota_esq = 0
		self.deltaTime = 0
		self.cmd = False

		#pega estado inicial
		self.last_time = time.time()
		self.atualiza_fps()
		self.chage_state()
		self.atualiza_cinematica()

		state_a = []
		state = []
		#state = self.pos_target.tolist()
		#state += [np.linalg.norm(self.t_pos_last-self.pos_target)/TARGET_BOUND_RANGE]
		if COM_IN_STATE:
			self.body.set_angles(self.body_angles[:6], self.body_angles[6:12])
			com_relative_dir = self.body.get_com(1) # right leg are support leg
			com_relative_esq = self.body.get_com(0) # left leg are support leg
			state_a += (com_relative_dir.tolist()+com_relative_esq.tolist())
		if USING_MARCOS_CONTROLLER:
			state_a += [self.perna, self.t_state/self.tempoPasso]
		if TORSO_ACCELERATION_IN_STATE:
			state_a += (self.t_acc_last/11.).tolist()
		if TORSO_ORIENTATION_IN_STATE:
			state_a += (self.t_ori_last/math.pi).tolist()
		if LAST_ACTION_IN_STATE:
			state_a += self.action_last.tolist()
		if PRESSURE_FEET_IN_STATE:
			state_a += self.t_force_last.tolist()
		if LEG_JOINT_POSITION_IN_STATE:
			state += (np.array(self.body_angles[:12])/math.pi).tolist()

		state = state+([0.]*N_PS*(S_P*2-1))
		state += state_a
		return np.array(state)


	def step(self, action, cmd):
		#bounded
		action[0] *= (self.a+self.c-HEIGHT_INIT)
		action[1] = (action[1]+1)*SHIFT_X_FOOT_MAX
		action[2] = (action[2]+1)*SHIFT_Y_HIP_MAX
		action[3] = (action[3]+1)*SHIFT_Z_FOOT_MAX
		action[4] = (action[4]+1)*DISTANCE_FOOT_MAX
		#self.rot_desvio = 1 if action[4] > 0 else -1
		#action[4] = math.fabs(action[4])*ANGLE_Z_HIP_MAX
		#action[5] = (action[5]+1) * TIME_STEP_MAX + TIME_STEP_MIN

		
		self.altura = HEIGHT_INIT+action[0]
		self.pos_inicial_pelves = [0., action[4],self.altura]
		self.pos_inicial_foot = [0., action[4],self.altura]
		self.deslocamentoXpes = action[1]
		self.deslocamentoYpelves = action[2]
		self.deslocamentoZpes = action[3]
		#self.deslocamentoZpelves = action[4]
		#self.tempoPasso = action[5]
		

		'''
		self.altura = 17.
		self.deslocamentoXpes = 2.
		self.deslocamentoYpelves = 3.5
		self.deslocamentoZpes = 1.5
		self.deslocamentoZpelves = 5.
		self.tempoPasso = 1.
		'''

		self.cmd = cmd
		self.last_time = time.time()
		self.atualiza_fps()
		self.t_step = 0
		while(self.t_step < TIME_STEP_ACTION):
			self.atualiza_cinematica()
			self.atualiza_fps()
			self.chage_state()
			self.t_step += self.deltaTime

			self.pub_queue.put([False, self.w_id, self.body_angles])
			self.pub_rate.sleep()
		cmd_to_float = 1. if not self.cmd else 0.
		self.pub_queue.put([False, self.w_id, [0.]*18+[cmd_to_float]]) #command to pause simulation

		return self.get_state(action)

	def run_marcos_controller(self):
		self.last_time = time.time()
		self.atualiza_fps()
		while(True):
			self.chage_state()
			self.atualiza_cinematica()
			self.atualiza_fps()
			
			print(self.altura,
				self.deslocamentoXpes,
				self.deslocamentoYpelves,
				self.deslocamentoZpes,
				self.deslocamentoZpelves)

			self.pub_queue.put([False, self.w_id, self.body_angles])
			self.pub_rate.sleep()


	def get_state(self, action=None):
		vetor_mov = np.array(self.t_pos_shd)-self.t_pos_last
		self.t_pos_last = np.array(self.t_pos_shd)
		self.t_ori_last = np.array(self.t_ori_shd)
		self.t_acc_last = np.array(self.t_acc_shd)
		self.t_joint_last = np.array(self.t_joint_shd)

		state_a = []
		state = []
		#state = self.pos_target.tolist()
		#state = ((self.t_pos_last-self.pos_target)/np.linalg.norm(self.t_pos_last-self.pos_target)).tolist()
		if COM_IN_STATE:
			self.body.set_angles(self.t_joint_last[:6], self.t_joint_last[6:12])
			com_relative_dir = self.body.get_com(1) # right leg are support leg
			com_relative_esq = self.body.get_com(0) # left leg are support leg
			state_a += (com_relative_dir.tolist()+com_relative_esq.tolist())
		if USING_MARCOS_CONTROLLER:
			state_a += [self.perna, self.t_state/self.tempoPasso]
		if TORSO_ACCELERATION_IN_STATE:
			state_a += (self.t_acc_last/11.).tolist()
		if TORSO_ORIENTATION_IN_STATE:
			state_a += (self.t_ori_last/math.pi).tolist()
		if LAST_ACTION_IN_STATE:
			state_a += self.action_last.tolist()
			if (action is not None):
				self.action_last = np.array(action)
		if PRESSURE_FEET_IN_STATE:
			state_a += self.t_force_last.tolist()
		if LEG_JOINT_POSITION_IN_STATE:
			state += (self.t_joint_last/math.pi).tolist()

		i = int((self.t_state/self.tempoPasso)*S_P)
		if not self.perna:
			if i == 0:
				state = state+([0.]*N_PS*(S_P*2-1))
			else:
				state = ([0.]*N_PS*i)+state+([0.]*N_PS*(S_P-(i+1)))+([0.]*N_PS*S_P)
		else:
			if i == 0:
				state = ([0.]*N_PS*S_P)+state+([0.]*N_PS*(S_P-1))
			else:
				state = ([0.]*N_PS*S_P)+([0.]*N_PS*i)+state+([0.]*N_PS*(S_P-(i+1)))

		state += state_a

		#print(self.t_ori_last)
		#check if done
		reward = 0.
		progress = vetor_mov[0]
		if math.fabs(self.t_ori_last[0]) > ANGLE_FALLEN_THRESHOLD or math.fabs(self.t_ori_last[1]) > ANGLE_FALLEN_THRESHOLD:
			self.done = True
			reward = W_ALIVE*-1
		else:
			'''
			#orientação
			to_target = self.pos_target
			erro_ori = (to_target[0]/(np.sum(to_target)))*math.cos(self.t_ori_last[2])-(to_target[1]/(np.sum(to_target)))*math.sin(self.t_ori_last[2])

			#localização
			aux = np.sum(self.pos_target*vetor_mov)
			angle_bet = 0 if aux < 1e-6 else math.acos(aux/(np.linalg.norm(self.pos_target)*np.linalg.norm(vetor_mov)))
			dist_no_rumo = 0.
			if angle_bet > 90:
				angle_bet = 180. - angle_bet
				dist_no_rumo = -math.cos(angle_bet)*np.linalg.norm(vetor_mov)
			else:
				dist_no_rumo = math.cos(angle_bet)*np.linalg.norm(vetor_mov)
			
			reward = W_ORI*math.exp(-((1-erro_ori)**2))+W_INC/2.*math.exp(-(math.fabs(self.t_ori_last[0])))+W_INC/2.*math.exp(-(math.fabs(self.t_ori_last[1])))+W_DIST*dist_no_rumo
			'''
			progress = vetor_mov[0]
			bonus_alive = 1.
			rewards =  [W_INC/2.*math.exp(-(math.fabs(self.t_ori_last[0]))),
						W_INC/2.*math.exp(-(math.fabs(self.t_ori_last[1]))),
						W_DIST*progress,
						W_ALIVE*bonus_alive]
			reward = np.sum(rewards)



		#print(self.done)
		info = {'progress': progress}
		return np.array(state), self.done, reward, info

#############################	CALLBACKS	##############################################

	def t_acc_last_callback (self, vetor):
		self.t_acc_shd[0] = vetor.x
		self.t_acc_shd[1] = vetor.y
		self.t_acc_shd[2] = vetor.z

	def t_ori_last_callback (self, vetor):
		self.t_ori_shd[0] = vetor.x*DEG_2_RAD
		self.t_ori_shd[1] = vetor.y*DEG_2_RAD
		self.t_ori_shd[2] = vetor.z*DEG_2_RAD

	def t_pos_last_callback (self, vetor):
		self.t_pos_shd[0] = vetor.x
		self.t_pos_shd[1] = vetor.y

	def t_joint_last_callback (self, msg):
		data = msg.data
		for i in range(len(data)):
			self.t_joint_shd[i] = data[i]

	def t_force_last_callback (self, msg):
		data = msg.data
		for i in range(len(data)):
			self.t_force_shd[i] = data[i]/30.

	###################################################################################

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
			return 1
		else:
			return 0


	# Retorna os 6 angulos de da perna, calculando a cinematica inversa. Considerando o pé como base e o quadril como ponto variável
	def footToHip(self, pointHip):
		angulos = []
		b = np.linalg.norm(np.array(pointHip))
		if b > self.a+self.c:
			b = self.a+self.c - 0.01
			aux = np.array(pointHip)
			pointHip = ((aux/np.linalg.norm(aux))*b).tolist()
		elif b < 1.:
			b = 1.
			aux = np.array(pointHip)
			pointHip = (aux/np.linalg.norm(aux)).tolist()

		x,y,z = pointHip


		#ankle roll
		theta = math.atan(y/z)
		angulos.append(theta)

		#ankle pitch
		'''if ((self.a**2-(b**2+self.c**2))/(-2*b*self.c) < -1 or (self.a**2-(b**2+self.c**2))/(-2*b*self.c) > 1):
			print (self.a, b, self.c)'''
		anguloA = math.acos((self.a**2-(b**2+self.c**2))/(-2*b*self.c))
		betha = math.atan(x/z)
		anguloA = betha + anguloA
		angulos.append(anguloA)

		#knee
		anguloB = math.acos((b**2-(self.a**2+self.c**2))/(-2*self.a*self.c))
		anguloB = anguloB - math.pi
		angulos.append(anguloB)

		#hip pitch
		'''if (self.c**2-(self.a**2+b**2))/(-2*self.a*b) > 1 or (self.c**2-(self.a**2+b**2))/(-2*self.a*b) < -1:
			print (self.a, b, self.c)'''
		anguloC = math.acos((self.c**2-(self.a**2+b**2))/(-2*self.a*b))
		anguloC = anguloC - betha
		angulos.append(anguloC)

		#hip roll
		angulos.append(theta)

		#hip yall
		angulos.append(0)

		return angulos


	#	- Pega o proximo "estado" da função de tragetória, a função de tragetória muda de acordo com as variaveis que definem o deslocamento e rotação do robô
	#	Entrada: tempo float/int t
	#	Saída: 2 vetores de 3 posições (x,y,z). O primeiro indica a posição da pelves considerando o pé em contato com o chão como base, 
	#		   o segundo vetor indica a posição do pé de balanço considerando a pelves do pé de balanço como base.
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


	def atualiza_cinematica(self):
		x = (self.t_state*125)/self.tempoPasso
		pelv_point, foot_point = self.getTragectoryPoint(x)
		if self.perna:
			#CINEMÁTICA INVERSA
			data_pelv = self.footToHip(pelv_point)
			data_foot = self.footToHip(foot_point)
		
			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A ESQUERDA
			if self.rota_dir == 1:
				data_pelv[5] = self.deslocamentoZpelves/2. + self.deslocamentoZpelves/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			elif self.rota_dir == -1:
				data_pelv[5] = -self.deslocamentoZpelves/2. - self.deslocamentoZpelves/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A DIREITA
			if self.rota_esq == 2:
				data_foot[5] = self.deslocamentoZpelves - (self.deslocamentoZpelves/2. + self.deslocamentoZpelves/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			elif self.rota_esq == -2:
				data_foot[5] = -self.deslocamentoZpelves - (-self.deslocamentoZpelves/2. - self.deslocamentoZpelves/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			else:
				data_foot[5] = 0

			#PÉ DIREITO ESTÁ EM CONTATO COM O CHÃO E PÉ ESQUERDO ESTÁ SE MOVENDO.
			data_pelv[0] *= -1
			data_pelv[4] *= -1
			data = data_pelv + data_foot + [0]*6

			#CONFIGURA BODY SOLVER PARA INVOCAR FUNÇÕES DO MODELO DINÂMICO DO ROBÔ
			self.body.set_angles(data_pelv, data_foot)
		else:
			#CINEMÁTICA INVERSA
			data_pelv = self.footToHip(pelv_point)
			data_foot = self.footToHip(foot_point)

			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A ESQUERDA
			if self.rota_esq == 1:
				data_pelv[5] =  self.deslocamentoZpelves/2. + self.deslocamentoZpelves/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			elif self.rota_esq == -1:
				data_pelv[5] =  -self.deslocamentoZpelves/2. - self.deslocamentoZpelves/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50)))
				data_pelv[5] = data_pelv[5] * math.pi/180.
			else:
				data_pelv[5] = 0

			#ROTINHA PARA VIRAR/PARAR DE VIRAR PARA A DIREITA
			if self.rota_dir == 2:
				data_foot[5] =  self.deslocamentoZpelves - (self.deslocamentoZpelves/2. + self.deslocamentoZpelves/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50))))
				data_foot[5] = data_foot[5] * math.pi/180.
			elif self.rota_dir == -2:
				data_foot[5] =  -self.deslocamentoZpelves - (-self.deslocamentoZpelves/2. - self.deslocamentoZpelves/2.*((np.exp((2*(x-125/2))/50) - np.exp((2*(x-125/2))/-50))/(np.exp((2*(x-125/2))/50)+np.exp((2*(x-125/2))/-50))))		
				data_foot[5] = data_foot[5] * math.pi/180.
			else:
				data_foot[5] = 0

			#PÉ ESQUERDO ESTÁ EM CONTATO COM O CHÃO E PÉ DIREITO ESTÁ SE MOVENDO.
			data_foot[0] *= -1
			data_foot[4] *= -1
			data = data_foot + data_pelv + [0]*6

			#CONFIGURA BODY SOLVER PARA INVOCAR FUNÇÕES DO MODELO DINÂMICO DO ROBÔ
			self.body.set_angles(data_foot, data_pelv)

		cmd_to_float = 1. if self.cmd else 0.
		self.body_angles = data+[cmd_to_float]
		#self.gravity_compensation()


	def gravity_compensation(self):
		#return
		if (self.t_state < self.tempoPasso/2 and self.t_state < self.tempoPasso*self.time_ignore_GC) or (self.t_state >= self.tempoPasso/2 and self.t_state > self.tempoPasso*(1-self.time_ignore_GC)) or self.deslocamentoYpelves != self.deslocamentoYpelvesMAX or self.deslocamentoZpes == 0:
			return

		torques = self.body.get_torque_in_joint(self.perna,[3,5])

		dQ = (np.array(torques)/KP_CONST)/self.cg
		#print(dQ[1], self.perna)

		dQ *= math.sin(self.t_state*math.pi/self.tempoPasso)
		if self.perna:
			self.body_angles[2] += dQ[0]
			self.body_angles[4] += (dQ[1]*-1)
		else:
			self.body_angles[6+2] += dQ[0]
			self.body_angles[6+4] += dQ[1]


	def atualiza_fps(self):
		self.deltaTime = time.time() - self.last_time
		self.last_time = time.time()
		self.count_frames += 1
		self.timer_fps += self.deltaTime
		return None

	'''
	def cinematica_inversa(self, r_point, l_point, t_angles, lz_angles, cmd):
		data_r = self.footToHip(r_point)
		data_l = self.footToHip(l_point)
		data_r[0] *= -1
		data_r[4] *= -1

		#torso angles gain
		data_r[3] += t_angles[0]
		data_r[4] += t_angles[1]
		data_l[3] += t_angles[0]
		data_l[4] += t_angles[1]

		#rotate leg at z axis
		data_r[5] += lz_angles[0]
		data_l[5] += lz_angles[1]

		cmd_to_float = 1. if cmd else 0.
		return np.array(data_r + data_l + [0]*6 + [cmd_to_float])


	# Retorna os 6 angulos de da perna, calculando a cinematica inversa. Considerando o pé como base e o quadril como ponto variável
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

	def step(self, action, cmd):
		#print(cmd)
		r_v = np.array(action[0:3])
		l_v = np.array(action[3:6])
		t_v = np.array(action[6:8])*10
		lz_v = np.array(action[8:10])*10

		r_p = self.r_point_last
		l_p = self.l_point_last
		t_a = self.t_angles_last
		lz_a = self.lz_angles_last

		ant_t = time.time()
		timer = 0.
		while(timer < TIME_STEP_ACTION):
			atual_t = time.time()
			dt = (atual_t - ant_t)
			timer += dt
			
			att_r_p = r_v*dt + r_p
			att_l_p = l_v*dt + l_p
			att_t_a = t_v*dt + t_a
			att_lz_a = lz_v*dt + lz_a

			##### se dist entre att_r_p e (0,0,0) for maior que a+c : normalize para que a dist seja = a+c #######
			try:
				angles = self.cinematica_inversa(att_r_p, att_l_p, att_t_a, att_lz_a, cmd)
			except Exception as e:
				pass
			else:
				r_p = att_r_p
				l_p = att_l_p
				t_a = att_t_a
				lz_a = att_lz_a
				self.body_angles = angles

			mat = Float32MultiArray()
			mat.data = self.body_angles
			self.pos_pub.publish(mat)
			#print(mat.data)
			ant_t = atual_t
			self.pub_rate.sleep()

		self.r_point_last = r_p
		self.l_point_last = l_p
		self.t_angles_last = t_a
		self.lz_angles_last = lz_a
		self.action_last = np.array(action[:10])


		return self.get_state()
	

	def get_state(self):
		state = []
		if COM_IN_STATE:
			self.body.set_angles(self.body_angles[:6], self.body_angles[6:12])
			com_relative_dir = self.body.get_com(1) # right leg are support leg
			com_relative_esq = self.body.get_com(0) # left leg are support leg
			state += (com_relative_dir.tolist()+com_relative_esq.tolist())
		if TARGETS_POS_IN_STATE:
			state += (self.r_point_last.tolist() + self.l_point_last.tolist() + self.t_angles_last.tolist() + self.lz_angles_last.tolist())
		if TORSO_ACCELERATION_IN_STATE:
			state += self.t_acc_last.tolist()
		if TORSO_ORIENTATION_IN_STATE:
			state += self.t_ori_last.tolist()
		if LAST_ACTION_IN_STATE:
			state += self.action_last.tolist()

		#check if done
		if math.fabs(self.t_ori_last[0]) > ANGLE_FALLEN_THRESHOLD or math.fabs(self.t_ori_last[1]) > ANGLE_FALLEN_THRESHOLD:
			self.done = True
			reward = 0
		else:
			to_target = self.pos_target - self.t_pos_last
			erro_ori = (to_target[0]/(np.sum(to_target)))*math.cos(self.t_ori_last[2])-(to_target[1]/(np.sum(to_target)))*math.sin(self.t_ori_last[2])
			reward = W_ORI*math.exp(-((1-erro_ori)**2))+W_INC/2*math.exp(-((self.t_ori_last[0])**2))+W_INC/2*math.exp(-((self.t_ori_last[1])**2))+W_DIST*math.exp(-(np.linalg.norm(self.t_pos_last-self.pos_target)))-(W_DIST+W_INC+W_ORI)

		#print(self.done)
		return np.array(state), self.done, reward
	'''