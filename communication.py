# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import os

class Visao():
	def __init__(self):
		rclpy.init()
		node = Node('visao')
		self.pub = node.create_publisher(Float32MultiArray, 'Bioloid/visao_cmd', 1)

	def wait_for_cmd(self):
		while (True):
			try:
				info = [0]*3

				os.system("clear")
				angulo = input("Informe o angulo vertical (-180, 180)!")
				info[0] = angulo
				angulo = input("Informe o angulo horizontal (-180, 180)!")
				info[1] = angulo
				flag_msg = input("Esta com a bola?")
				info[2] = flag_msg

				msg = Float32MultiArray()
				msg.data = np.array(info).astype(np.float32).tolist()
				self.pub.publish(msg)
			except KeyboardInterrupt as e:
				break
			except Exception as e:
				continue
		
if "__main__" == __name__:
	visao = Visao()
	visao.wait_for_cmd()
