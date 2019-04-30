import matplotlib.pyplot as plt
import matplotlib.animation as anim
import time
import math
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

R_press_vet = [0]*4
L_press_vet = [0]*4

fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
fig2 = plt.figure()
ax2 = fig2.add_subplot(111)

norma = 5


def get_r_vet():
	return R_press_vet

def get_l_vet():
	return L_press_vet

def foot_inertial_callback (msg):
	L_press_vet = msg.data[:4]
	R_press_vet = msg.data[4:]
	total_press_esq = np.sum(L_press_vet)
	total_press_dir = np.sum(R_press_vet)
	#norma = np.sum(R_press_vet)+np.sum(L_press_vet)+0.001
	print (np.array(L_press_vet).astype(np.int), np.array(R_press_vet).astype(np.int), total_press_esq, total_press_dir)

#inicia nó e se inscreve no tópico do sensor de pressão
rospy.init_node('Pressure_plotter', anonymous=True)
rospy.Subscriber("/vrep_ros_interface/Bioloid/foot_pressure_sensor", Float32MultiArray, foot_inertial_callback)

rospy.spin()
'''
def refrGraph_esq (i):
	ax1.clear()
	
	data = get_l_vet()

	e1 = data[1]
	e2 = data[3]
	e3 = data[2]
	e4 = data[0]

	print(data)

	ve1= np.array([-1, 1])*e1
	ve2= np.array([1, 1])*e2
	ve3= np.array([1, -1])*e3
	ve4= np.array([-1, -1])*e4
	
	ax1.quiver(0, 0, ve1[0], ve1[1], color='r', scale=3)
	ax1.quiver(0, 0, ve2[0], ve2[1], color='g', scale=3)	
	ax1.quiver(0, 0, ve3[0], ve3[1], color='b', scale=3)
	ax1.quiver(0, 0, ve4[0], ve4[1], scale=3)

def refrGraph_dir (i):
	ax2.clear()

	vd1= np.array([-1, 1])
	vd2= np.array([1, 1])
	vd3= np.array([1, -1])
	vd4= np.array([-1, -1])

	ax2.quiver(0, 0, vd1[0], vd1[1], color='r', scale=3)
	ax2.quiver(0, 0, vd2[0], vd2[1], color='g', scale=3)
	ax2.quiver(0, 0, vd3[0], vd3[1], color='b', scale=3)
	ax2.quiver(0, 0, vd4[0], vd4[1], scale=3)

ani1 = anim.FuncAnimation(fig1, refrGraph_esq,interval=0)
ani2 = anim.FuncAnimation(fig2, refrGraph_dir,interval=0)

plt.show()
'''