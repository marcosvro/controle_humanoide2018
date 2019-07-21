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
		self.mass = self.body.perna_dir_para_esq.total_mass
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

	#################################### FUNÇÕES DO EVIROMENT #############################

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
		#self.lz_angles_last = np.array([0., 0.])
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
		self.body_angles = self.cinematica_inversa(self.r_point_last, self.l_point_last, self.t_angles_last, self.cmd)

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

		state = state+([0.]*N_PS*(S_P-1))
		state += state_a
		return np.array(state)


	def step(self, action, cmd):
		#print(cmd)
		r_v = np.array(action[0:3])*10
		l_v = np.array(action[3:6])*10
		t_v = np.array(action[6:8])
		#lz_v = np.array(action[8:10])*10

		r_p = self.r_point_last
		l_p = self.l_point_last
		t_a = self.t_angles_last
		#lz_a = self.lz_angles_last

		ant_t = time.time()
		timer = 0.
		while(timer < TIME_STEP_ACTION):
			atual_t = time.time()
			dt = (atual_t - ant_t)
			timer += dt
			ant_t = atual_t
			
			att_r_p = r_v*dt + r_p
			att_l_p = l_v*dt + l_p
			att_t_a = t_v*dt + t_a
			#att_lz_a = lz_v*dt + lz_a

			#constrants
			modulo_att_r_p = np.linalg.norm(att_r_p)
			modulo_att_l_p = np.linalg.norm(att_l_p)
			limite = self.a+self.c
			if modulo_att_r_p >= limite:
				att_r_p = (att_r_p/modulo_att_r_p)*(limite-0.01)
			if modulo_att_l_p >= limite:
				att_l_p = (att_l_p/modulo_att_l_p)*(limite-0.01)

			if att_t_a[0] > TORSO_COMPENSATION_MAX:
				att_t_a[0] = TORSO_COMPENSATION_MAX
			elif att_t_a[0] < -TORSO_COMPENSATION_MAX:
				att_t_a[0] = -TORSO_COMPENSATION_MAX

			if att_t_a[1] > TORSO_COMPENSATION_MAX:
				att_t_a[1] = TORSO_COMPENSATION_MAX
			elif att_t_a[1] < -TORSO_COMPENSATION_MAX:
				att_t_a[1] = -TORSO_COMPENSATION_MAX

			#print(att_r_p, att_l_p)
			#inverse kinematics
			try:
				angles = self.cinematica_inversa(att_r_p, att_l_p, att_t_a, cmd)
			except Exception as e:
				pass
			else:
				r_p = att_r_p
				l_p = att_l_p
				t_a = att_t_a
				#lz_a = att_lz_a
				self.body_angles = angles
			self.pub_queue.put([False, self.w_id, self.body_angles])
			self.pub_rate.sleep()

		self.t_state += timer
		if (self.t_state > self.tempoPasso):
			self.t_state = 0.
		cmd_to_float = 1. if not self.cmd else 0.
		self.pub_queue.put([False, self.w_id, [0.]*18+[cmd_to_float]]) #command to pause simulation

		self.r_point_last = r_p
		self.l_point_last = l_p
		self.t_angles_last = t_a
		#self.lz_angles_last = lz_a

		return self.get_state(action)


	def get_state(self, action=None):
		vetor_mov = np.array(self.t_pos_shd)-self.t_pos_last
		self.t_pos_last = np.array(self.t_pos_shd)
		self.t_ori_last = np.array(self.t_ori_shd)
		self.t_acc_last = np.array(self.t_acc_shd)
		self.t_joint_last = np.array(self.t_joint_shd)
		self.t_force_last = np.array(self.t_force_shd)

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
		if i == 0:
			state = state+([0.]*N_PS*(S_P-1))
		else:
			state = ([0.]*N_PS*i)+state+([0.]*N_PS*(S_P-(i+1)))
	
		state += state_a

		#print(self.t_ori_last)
		#check if done
		reward = 0.
		progress = vetor_mov[0]
		bad_support = 0.
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
			
			pessao_pe_esq = np.sum(self.t_force_last[:4])
			pessao_pe_dir = np.sum(self.t_force_last[4:])
			if i == 1: # pé direito no chão
				pessao_ideal_pe_esq = 0
				pessao_ideal_pe_dir = self.mass*9.8 # peso
			elif i == 3: #pé esquerdo no chão
				pessao_ideal_pe_esq = self.mass*9.8
				pessao_ideal_pe_dir = 0

			if i == 3 or i == 1:
				erro_press_esq = math.fabs(pessao_ideal_pe_esq-pessao_pe_esq)
				erro_press_dir = math.fabs(pessao_ideal_pe_dir-pessao_pe_dir)
				bad_support -= (1 - math.exp(-erro_press_esq))
				bad_support -= (1 - math.exp(-erro_press_dir))

			progress = vetor_mov[0]
			bonus_alive = 1.
			rewards =  [W_INC/2.*math.exp(-(math.fabs(self.t_ori_last[0]))),
						W_INC/2.*math.exp(-(math.fabs(self.t_ori_last[1]))),
						W_DIST*progress,
						W_ALIVE*bonus_alive,
						W_APOIO*bad_support]
			reward = np.sum(rewards)


		#print(self.done)
		info = {
			'progress': progress,
			'bad_support': bad_support
		}
		return np.array(state), self.done, reward, info

#######################################################################################

#############################	CALLBACKS	###########################################

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
			self.t_force_shd[i] = data[i]/(self.mass*9.8)

	###################################################################################

	############################ FUNÇÕES DE CINEMÁTICA ################################

	def cinematica_inversa(self, r_point, l_point, t_angles, cmd):
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
		#data_r[5] += lz_angles[0]
		#data_l[5] += lz_angles[1]

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

	###################################################################################
