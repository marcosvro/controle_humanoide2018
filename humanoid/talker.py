#!/usr/bin/env python
 # license removed for brevity
import math
import rospy
from std_msgs.msg import Float32MultiArray
from leiDosCossenos import footToHip, hipToFoot, getTragectoryPoint,nEstados,tempoPasso
import numpy as np

def talker():
	pub = rospy.Publisher('Bioloid/cmd_vel', Float32MultiArray, queue_size=1)
	rospy.init_node('publisher', anonymous=True)
	rate = rospy.Rate(tempoPasso/nEstados) # 10hz
	mat = Float32MultiArray()
	time = 0
	flag = True

	while not rospy.is_shutdown():
		ponto1, ponto2 = getTragectoryPoint(time)
		if flag is True:
			mat.data = footToHip(ponto1) + footToHip(ponto2) + [0]*6
			#print(np.array(mat.data[:5])*180/math.pi)
		else:
			#print("f ",ponto2[0])
			mat.data = footToHip(ponto2) + footToHip(ponto1) + [0]*6

		mat.data[6] = -mat.data[6] #calcanhar direito
		#mat.data[2] = mat.data[2]*-1 - 1.57#joelho direito
		#mat.data[3] = mat.data[3]*-1
		mat.data[10] = -mat.data[10] #espacate
		#mat.data[6] = mat.data[6]*-1 #calcanhar esquerdo
		#mat.data[8] = mat.data[8]*-1 - 1.57 #joelho esquerdo



		pub.publish(mat)
		time = (time + 1)%nEstados
		if time == 0:
			flag = not flag
		rospy.sleep(tempoPasso/nEstados)


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
