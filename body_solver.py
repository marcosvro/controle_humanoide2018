import numpy as np
import tinyik as ik

class Body():
	def __init__ (self):
		self.constrants = [[1, 0, 0],
						   [0, 1, 0],
						   [0, 1, 0],
						   [0, 1, 0],
						   [1, 0, 0],
						   [0, 0, 1],
						   [1, 0, 0],
						   [0, 1, 0],
						   [0, 1, 0],
						   [0, 1, 0],
						   [1, 0, 0],
						   [0, 0, 1],
						   [0, 1, 0],
						   [1, 0, 0],
						   [0, 1, 0],
						   [0, 1, 0],
						   [1, 0, 0],
						   [0, 1, 0]]

		self.CoM_parts = [[]
						  []]
		self.perna_dir = ik.Actuator()













'''
+5.711e+0
+3.495e+1
-6.876e+1
+3.381e+1
+5.711e+0
'''