import matplotlib.pyplot as plt
import numpy as np
import math
import time

x = time.time()
fig, ax1 = plt.subplots()
fig, ax2 = plt.subplots()

while 1:
	try:
		value = (math.sin(x)+2)/2
		x = time.time()
		
		e1 = 0.5
		e2 = 0.5
		e3 = 0.3
		e4 = value
		ve1= np.array([-1, 1])*e1
		ve2= np.array([1, 1])*e2
		ve3= np.array([1, -1])*e3
		ve4= np.array([-1, -1])*e4
		d1 = 0.3
		d2 = 0.3
		d3 = 0.3
		d4 = 0.3
		vd1= np.array([-1, 1])*d1
		vd2= np.array([1, 1])*d2
		vd3= np.array([1, -1])*d3
		vd4= np.array([-1, -1])*d4
		ax1.quiver(0, 0, ve1[0], ve1[1], color='r', scale=3)
		ax1.quiver(0, 0, ve2[0], ve2[1], color='g',scale=3)
		ax1.quiver(0, 0, ve3[0], ve3[1], color='b',scale=3)
		ax1.quiver(0, 0, ve4[0], ve4[1],scale=3)
		ax2.quiver(0, 0, vd1[0], vd1[1], color='r', scale=3)
		ax2.quiver(0, 0, vd2[0], vd2[1], color='g',scale=3)
		ax2.quiver(0, 0, vd3[0], vd3[1], color='b',scale=3)
		ax2.quiver(0, 0, vd4[0], vd4[1],scale=3)
		plt.show()

	except KeyboardInterrupt:
		break
	except Exception as e:
		print(e)