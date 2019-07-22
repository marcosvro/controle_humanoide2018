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
from scipy.interpolate import interp1d


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
				t_pos_feet,
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
		self.t_pos_feet_shd = t_pos_feet
		
		self.reset()
		
		time.sleep(TIME_WAIT_INIT_PUBS)
		#define subscribers para os dados do state
		if not TESTING:
			rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+"/t_acc_last", Vector3, self.t_acc_last_callback)
			rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+"/t_ori_last", Vector3, self.t_ori_last_callback)
			rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+"/t_pos_last", Vector3, self.t_pos_last_callback)
			rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+"/t_joint_last", Float32MultiArray, self.t_joint_last_callback)
			rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+"/t_force_last", Float32MultiArray, self.t_force_last_callback)
			rospy.Subscriber("/"+simu_name_id+"/"+simu_name_id+"/t_feet_pos_last", Float32MultiArray, self.t_pos_feet_last_callback)
		else:
			rospy.Subscriber("vrep_ros_interface/"+simu_name_id+"/t_acc_last", Vector3, self.t_acc_last_callback)
			rospy.Subscriber("vrep_ros_interface/"+simu_name_id+"/t_ori_last", Vector3, self.t_ori_last_callback)
			rospy.Subscriber("vrep_ros_interface/"+simu_name_id+"/t_pos_last", Vector3, self.t_pos_last_callback)
			rospy.Subscriber("vrep_ros_interface/"+simu_name_id+"/t_joint_last", Float32MultiArray, self.t_joint_last_callback)
			rospy.Subscriber("vrep_ros_interface/"+simu_name_id+"/t_force_last", Float32MultiArray, self.t_force_last_callback)
			rospy.Subscriber("vrep_ros_interface/"+simu_name_id+"/t_feet_pos_last", Float32MultiArray, self.t_pos_feet_last_callback)

	#################################### FUNÇÕES DO EVIROMENT #############################

	def reset(self):
		#dados que vem do simulador
		self.t_acc_last = np.array([0.]*3)
		self.t_ori_last = np.array([0.]*3)
		self.t_pos_last = np.array([0.]*2)
		self.t_force_last = np.array([0.125]*8)
		self.t_pos_feet_last = np.array([0., DISTANCE_FOOT_INIT/2, 0., -DISTANCE_FOOT_INIT/2])

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
		self.body_angles = self.cinematica_inversa(0. ,self.r_point_last, self.l_point_last, self.cmd)

		state_a = []
		state = []
		#state = self.pos_target.tolist()
		#state += [np.linalg.norm(self.t_pos_last-self.pos_target)/TARGET_BOUND_RANGE]
		if COM_IN_STATE:
			self.body.set_angles(self.body_angles[:6], self.body_angles[6:12])
			com_relative_dir = self.body.get_com(1) # right leg are support leg
			com_relative_esq = self.body.get_com(0) # left leg are support leg
			state += (com_relative_dir.tolist()+com_relative_esq.tolist())
		if USING_MARCOS_CONTROLLER:
			state += [self.perna, self.t_state/self.tempoPasso]
		if TORSO_ACCELERATION_IN_STATE:
			state += (self.t_acc_last/11.).tolist()
		if TORSO_ORIENTATION_IN_STATE:
			state += (self.t_ori_last/math.pi).tolist()
		if LAST_ACTION_IN_STATE:
			state += self.action_last.tolist()
		if PRESSURE_FEET_IN_STATE:
			state += self.t_force_last.tolist()
		if LEG_JOINT_POSITION_IN_STATE:
			state += (np.array(self.body_angles[:12])/math.pi).tolist()

		state = state+([0.]*N_PS*(S_P-1))
		#state += state_a
		return np.array(state)


	def step(self, action, cmd):
		#print(cmd)
		r_v = np.array(action[0:3]) # hip
		l_v = np.array(action[3:6]) # foot

		# X axis - forward
		r_v[0] *= SHIFT_X_FOOT_MAX
		l_v[0] *= SHIFT_X_FOOT_MAX

		# Y axis - Side
		r_v[1] = ((r_v[1]+1)/2.) * (-SHIFT_Y_HIP_MAX)
		l_v[1] = ((l_v[1]+1)/2.) * (SHIFT_Y_HIP_MAX)
		
		# Z axis - height
		limite_height = self.a+self.c
		r_v[2] = r_v[2]*(limite_height-HEIGHT_INIT) + HEIGHT_INIT
		l_v[2] = l_v[2]*(limite_height-HEIGHT_INIT) + HEIGHT_INIT

		#r_v, l_v = self.get_reference_tragectory_point(self.t_state+TIME_STEP_ACTION) #test para ver se o controle euristico funciona
		#print (np.around(l_v, decimals=1), np.around(r_v, decimals=1), self.t_state)

		r_p = self.r_point_last
		l_p = self.l_point_last
		#t_a = self.t_angles_last
		#lz_a = self.lz_angles_last


		# create interpolators
		interp_lin_r = interp1d([0, TIME_STEP_ACTION], np.vstack([r_p, r_v]), axis=0)
		interp_lin_l = interp1d([0, TIME_STEP_ACTION], np.vstack([l_p, l_v]), axis=0)

		ant_t = time.time()
		timer = 0.
		while(timer < TIME_STEP_ACTION):

			att_r_p = interp_lin_r(timer)
			att_l_p = interp_lin_l(timer)

			#att_t_a = t_v*dt + t_a
			#att_lz_a = lz_v*dt + lz_a

			#constrants
			modulo_att_r_p = np.linalg.norm(att_r_p)
			modulo_att_l_p = np.linalg.norm(att_l_p)
			
			if modulo_att_r_p >= limite_height:
				att_r_p = (att_r_p/modulo_att_r_p)*(limite_height-0.01)
			if modulo_att_l_p >= limite_height:
				att_l_p = (att_l_p/modulo_att_l_p)*(limite_height-0.01)

			'''
			if att_t_a[0] > TORSO_COMPENSATION_MAX:
				att_t_a[0] = TORSO_COMPENSATION_MAX
			elif att_t_a[0] < -TORSO_COMPENSATION_MAX:
				att_t_a[0] = -TORSO_COMPENSATION_MAX

			if att_t_a[1] > TORSO_COMPENSATION_MAX:
				att_t_a[1] = TORSO_COMPENSATION_MAX
			elif att_t_a[1] < -TORSO_COMPENSATION_MAX:
				att_t_a[1] = -TORSO_COMPENSATION_MAX
			'''

			#print(att_r_p, att_l_p)
			#inverse kinematics
			try:
				angles = self.cinematica_inversa(timer, att_r_p, att_l_p, cmd)
				#print(np.around(angles[:12], decimals=1), self.t_state+timer)
			except Exception as e:
				pass
			else:
				r_p = att_r_p
				l_p = att_l_p
				#t_a = att_t_a
				#lz_a = att_lz_a
				self.body_angles = angles
			self.pub_queue.put([False, self.w_id, self.body_angles])
			self.pub_rate.sleep()
			atual_t = time.time()
			dt = (atual_t - ant_t)
			timer += dt
			ant_t = atual_t

		self.t_state += TIME_STEP_ACTION

		self.r_point_last = r_p
		self.l_point_last = l_p

		cmd_to_float = 1. if not cmd else 0.
		self.pub_queue.put([False, self.w_id, [0.]*18+[cmd_to_float]]) #command to pause simulation

		
		#self.t_angles_last = t_a
		#self.lz_angles_last = lz_a

		return self.get_state(action)


	def get_state(self, action=None):
		vetor_mov_feet = np.array(self.t_pos_feet_shd)-self.t_pos_feet_last
		self.t_pos_last = np.array(self.t_pos_shd)
		self.t_ori_last = np.array(self.t_ori_shd)
		self.t_acc_last = np.array(self.t_acc_shd)
		self.t_joint_last = np.array(self.t_joint_shd)
		self.t_force_last = np.array(self.t_force_shd)
		self.t_pos_feet_last = np.array(self.t_pos_feet_shd)

		state_a = []
		state = []
		#state = self.pos_target.tolist()
		#state = ((self.t_pos_last-self.pos_target)/np.linalg.norm(self.t_pos_last-self.pos_target)).tolist()
		if COM_IN_STATE:
			self.body.set_angles(self.t_joint_last[:6], self.t_joint_last[6:12])
			com_relative_dir = self.body.get_com(1) # right leg are support leg
			com_relative_esq = self.body.get_com(0) # left leg are support leg
			state += (com_relative_dir.tolist()+com_relative_esq.tolist())
		if USING_MARCOS_CONTROLLER:
			state += [self.perna, self.t_state/self.tempoPasso]
		if TORSO_ACCELERATION_IN_STATE:
			state += (self.t_acc_last/11.).tolist()
		if TORSO_ORIENTATION_IN_STATE:
			state += (self.t_ori_last/math.pi).tolist()
		if LAST_ACTION_IN_STATE:
			state += self.action_last.tolist()
			if (action is not None):
				self.action_last = np.array(action)
		if PRESSURE_FEET_IN_STATE:
			state += self.t_force_last.tolist()
		if LEG_JOINT_POSITION_IN_STATE:
			state += (self.t_joint_last/math.pi).tolist()

		i = int((self.t_state/self.tempoPasso)*S_P)
		if i == 0:
			state = state+([0.]*N_PS*(S_P-1))
		else:
			state = ([0.]*N_PS*i)+state+([0.]*N_PS*(S_P-(i+1)))
	
		#state += state_a

		#print(self.t_ori_last)
		#check if done
		reward = 0.
		progress = vetor_mov_feet[0]+vetor_mov_feet[2] #avanço do pé esq e do dir em relação ao estado anterior
		bad_support = 0.
		r_pose = 0.
		r_inc = 0.
		if math.fabs(self.t_ori_last[0]) > ANGLE_FALLEN_THRESHOLD or math.fabs(self.t_ori_last[1]) > ANGLE_FALLEN_THRESHOLD:
			self.done = True
			reward = W_ALIVE*-1
		else:
			'''
			#orientação
			to_target = self.pos_target
			erro_ori = (to_target[0]/(np.sum(to_target)))*math.cos(self.t_ori_last[2])-(to_target[1]/(np.sum(to_target)))*math.sin(self.t_ori_last[2])

			#localização
			aux = np.sum(self.pos_target*vetor_mov_feet)
			angle_bet = 0 if aux < 1e-6 else math.acos(aux/(np.linalg.norm(self.pos_target)*np.linalg.norm(vetor_mov_feet)))
			dist_no_rumo = 0.
			if angle_bet > 90:
				angle_bet = 180. - angle_bet
				dist_no_rumo = -math.cos(angle_bet)*np.linalg.norm(vetor_mov_feet)
			else:
				dist_no_rumo = math.cos(angle_bet)*np.linalg.norm(vetor_mov_feet)
			
			reward = W_ORI*math.exp(-((1-erro_ori)**2))+W_INC/2.*math.exp(-(math.fabs(self.t_ori_last[0])))+W_INC/2.*math.exp(-(math.fabs(self.t_ori_last[1])))+W_DIST*dist_no_rumo
			'''
			
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
			'''

			self.body.foot_to_hip.angles = self.t_joint_last[:5]
			pos_atual_dir = self.body.foot_to_hip.ee #posição real do quadril dir em relação ao pé dir
			
			angulos_perna_esq = self.t_joint_last[6:11]
			angulos_perna_esq[0] *= -1
			angulos_perna_esq[4] *= -1
			self.body.foot_to_hip.angles = angulos_perna_esq
			pos_atual_esq = self.body.foot_to_hip.ee #posição real do quadril esq em relação ao pé esq

			#pos_atual_dir = self.r_point_last
			#pos_atual_esq = self.l_point_last

			pos_ref_dir, pos_ref_esq  = self.get_reference_tragectory_point(self.t_state)
			#print (np.around(pos_atual_esq, decimals=1), np.around(pos_ref_esq, decimals=1), np.around(pos_ref_esq-pos_atual_esq, decimals=1), self.t_state)

			erro_pose_dir = np.linalg.norm(pos_ref_dir-pos_atual_dir)
			erro_pose_esq = np.linalg.norm(pos_ref_esq-pos_atual_esq)

			erro_pose = erro_pose_dir+erro_pose_esq	
			r_pose = math.exp(-(math.fabs(erro_pose)))
			#print (pos_atual_dir, erro_pose)

			r_inclinacao_x = math.exp(-(math.fabs(self.t_ori_last[0])))
			r_inclinacao_y = math.exp(-(math.fabs(self.t_ori_last[1])))
			r_inc = r_inclinacao_x+r_inclinacao_y

			bonus_alive = 1.
			rewards =  [W_INC*r_inc,
						W_DIST*progress,
						W_ALIVE*bonus_alive,
						W_APOIO*bad_support,
						W_POSE*r_pose]
			reward = np.sum(rewards)


		if self.t_state > self.tempoPasso/2 and self.perna == 0:
			self.perna = 1
		if self.t_state + 1e-3 >= self.tempoPasso:
			self.t_state = 0.
			self.perna = 0

		info = {
			'progress': W_DIST*progress,
			'bad_support': W_APOIO*bad_support,
			'pose_reward': W_POSE*r_pose,
			'inc_reward' : W_INC*r_inc
		}
		#print(np.around(state, decimals=1))
		#print(i, self.t_state, self.tempoPasso)
		#print(np.around([progress*W_DIST, bad_support*W_APOIO], decimals=1))
		#time.sleep(4)
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

	def t_pos_feet_last_callback (self, msg):
		data = msg.data
		self.t_pos_feet_shd[0] = data[0]
		self.t_pos_feet_shd[1] = data[1]
		self.t_pos_feet_shd[2] = data[3]
		self.t_pos_feet_shd[3] = data[4]

	###################################################################################

	############################ FUNÇÕES DE CINEMÁTICA ################################

	def cinematica_inversa(self, x, r_point, l_point, cmd):
		data_r = self.footToHip(r_point)
		data_l = self.footToHip(l_point)
		data_r[0] *= -1
		data_r[4] *= -1

		#torso angles gain
		'''
		data_r[3] += t_angles[0]
		data_r[4] += t_angles[1]
		data_l[3] += t_angles[0]
		data_l[4] += t_angles[1]
		'''

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

	def get_reference_tragectory_point(self, x):
		primeira_parte = True
		if x > self.tempoPasso/2.:
			x -= self.tempoPasso/2.
			primeira_parte = False

		pos_pelves = np.array(self.pos_inicial_pelves[:])

		dif_estado = (x-(self.tempoPasso/2.)/2.)

		aux = (2*dif_estado)/((self.tempoPasso/2.)*0.4)
		aux2 = ((math.exp(aux) - math.exp(-aux))/(math.exp(aux)+math.exp(-aux)))

		p1 = (self.deslocamentoXpes/2)*aux2
		pos_pelves[0] = p1
		pos_pelves[1] += -self.deslocamentoYpelves*math.sin(x*math.pi/(self.tempoPasso/2.))

		pos_foot = np.array(self.pos_inicial_foot[:])
		p2 = (-self.deslocamentoXpes/2)*aux2
		pos_foot[0] = p2
		pos_foot[1] += self.deslocamentoYpelves*math.sin(x*math.pi/(self.tempoPasso/2.))
		pos_foot[2] = self.altura - self.deslocamentoZpes*math.exp(-(dif_estado**2)/(0.04))

		#print (np.around(pos_pelves, decimals=1), np.around(pos_foot, decimals=1), x)
		if primeira_parte:
			return pos_pelves, pos_foot
		else:
			return pos_foot, pos_pelves


	###################################################################################
