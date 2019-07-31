# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import os
from threading import Lock

lock = Lock()

class Visao():
	def __init__(self):
		self.pub = rospy.Publisher('Bioloid/visao_cmd', Float32MultiArray, queue_size=1)
		rospy.init_node('visão', anonymous=True)

	def wait_for_cmd(self):
		while not rospy.is_shutdown():
			try:
				input("pressione enter para poder digitar as entradas para a visão")
				info = [0]*3

				lock.acquire()

				os.system("clear")
				angulo = input("Informe o angulo vertical (-180, 180)!")
				info[0] = angulo
				angulo = input("Informe o angulo horizontal (-180, 180)!")
				info[1] = angulo
				flag_msg = input("Esta com a bola?")
				info[2] = flag_msg

				lock.release()

				msg = Float32MultiArray()
				msg.data = np.array(info).astype(np.float)
				self.pub.publish(msg)
			except KeyboardInterrupt as e:
				break
			except Exception as e:
				continue

if "__main__" == __name__:
	visao = Visao()
	visao.wait_for_cmd()
