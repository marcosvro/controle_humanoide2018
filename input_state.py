# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import numpy as np
import os

class State():
	def __init__(self):
		rclpy.init()
		node = Node('state')
		self.pub = node.create_publisher(Int8, 'Bioloid/state_cmd', 1)
		self.state_decoder = {
			1 : "IDDLE",
			2 : "MARCH",
			3 : "WALK",
			5 : "FALLEN",
			6 : "UP", 
			7 : "PENALIZED",
			8 : "TURN_R",
			9 : "TURN_L"
		}

	def wait_for_cmd(self):
		while (True):
			try:
				os.system("clear")
				state = input("Informe o próximo estado do Robô!\n" + 
				   "\n".join("%d - %s"%(key, value) for key, value in self.state_decoder.items()) + "\n\n")


				msg = Int8()
				msg.data = int(state)
				self.pub.publish(msg)
			except KeyboardInterrupt as e:
				break
			except Exception as e:
				continue
		
if "__main__" == __name__:
	visao = State()
	visao.wait_for_cmd()
