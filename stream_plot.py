import matplotlib.pyplot as plt
import matplotlib.animation as anim
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading

l_foot_y = 11
l_foot_x = 6.2

LD = [-l_foot_x/2., -l_foot_y/2.]
LT = [-l_foot_x/2., l_foot_y/2.]
RD = [l_foot_x/2., -l_foot_y/2.]
RT = [l_foot_x/2., l_foot_y/2.]

class Plotter():
	def __init__(self):
		self.R_press_vet = [0]*4
		self.L_press_vet = [0]*4

		# self.fig1 = plt.figure()
		# self.ax1 = self.fig1.add_subplot(111)
		self.fig2 = plt.figure()
		self.ax2 = self.fig2.add_subplot(111)

		#inicia nó e se inscreve no tópico do sensor de pressão
		rclpy.init()
		self.node = Node('Pressure_plotter')
		self.node.create_subscription(Float32MultiArray, "/Bioloid/foot_pressure_sensor", self.foot_inertial_callback, 1)

	def foot_inertial_callback (self, msg):
		self.L_press_vet = [(v if v != np.nan else 0.) for v in msg.data[:4]]
		self.R_press_vet = [(v if v != np.nan else 0.) for v in msg.data[4:]]
		self.total_press_esq = np.sum(self.L_press_vet)
		self.total_press_dir = np.sum(self.R_press_vet)
		#norma = np.sum(R_press_vet)+np.sum(L_press_vet)+0.001
		#print (np.array(L_press_vet).astype(np.int8), np.array(R_press_vet).astype(np.int8), total_press_esq, total_press_dir)

	def get_r_vet(self):
		r_press_weights_norm = (self.R_press_vet-np.min(self.R_press_vet))/(np.max(self.R_press_vet)-np.min(self.R_press_vet))
		return r_press_weights_norm

	def get_l_vet(self):
		l_press_weights_norm = (self.L_press_vet-np.min(self.L_press_vet))/(np.max(self.L_press_vet)-np.min(self.L_press_vet))
		return l_press_weights_norm

	def refrGraph_esq (self):
		self.ax1.clear()
		
		data = self.get_l_vet()

		l_press_vectors = np.array([LD, LT, RD, RT]) * np.array(data)[:, np.newaxis]
		self.l_center_of_press = np.sum(l_press_vectors, axis=0)

		self.ax1.set_xlim(-l_foot_x, l_foot_x)
		self.ax1.set_ylim(-l_foot_y, l_foot_y)
		self.ax1.set_aspect('equal', 'box')
		self.ax1.set_title(f"Pé esquerdo")
		
		# self.ax1.quiver(0, 0, l_press_vectors[0][0], l_press_vectors[0][1], angles='xy', scale_units='xy', scale=1)
		# self.ax1.quiver(0, 0, l_press_vectors[1][0], l_press_vectors[1][1], angles='xy', scale_units='xy', scale=1)
		# self.ax1.quiver(0, 0, l_press_vectors[2][0], l_press_vectors[2][1], angles='xy', scale_units='xy', scale=1)
		# self.ax1.quiver(0, 0, l_press_vectors[3][0], l_press_vectors[3][1], angles='xy', scale_units='xy', scale=1)
		self.ax2.quiver(0, 0, self.l_center_of_press[0], self.l_center_of_press[1], color='r', angles='xy', scale_units='xy', scale=1)


	def refrGraph_dir (self):
		self.ax2.clear()

		data = self.get_r_vet()

		r_press_vectors = np.array([LD, LT, RD, RT]) * np.array(data)[:, np.newaxis]
		self.r_center_of_press = np.sum(r_press_vectors, axis=0)

		self.ax2.set_xlim(-l_foot_x, l_foot_x)
		self.ax2.set_ylim(-l_foot_x, l_foot_x)
		self.ax2.set_aspect('equal', 'box')
		self.ax2.set_title(f"Pé direito")
		
		# self.ax2.quiver(0, 0, r_press_vectors[0][0], r_press_vectors[0][1], angles='xy', scale_units='xy', scale=1)
		# self.ax2.quiver(0, 0, r_press_vectors[1][0], r_press_vectors[1][1], angles='xy', scale_units='xy', scale=1)
		# self.ax2.quiver(0, 0, r_press_vectors[2][0], r_press_vectors[2][1], angles='xy', scale_units='xy', scale=1)
		# self.ax2.quiver(0, 0, r_press_vectors[3][0], r_press_vectors[3][1], angles='xy', scale_units='xy', scale=1)
		self.ax2.quiver(0, 0, self.r_center_of_press[0], self.r_center_of_press[1], color='r', angles='xy', scale_units='xy', scale=1)


	def setup_ros(self):
		rclpy.spin(self.node)
		self.node.destroy_node()
		rclpy.shutdown()

	def run(self):
		rate = self.node.create_rate(125)
		spin_t = threading.Thread(target=self.setup_ros, daemon=True)
		spin_t.start()

		while True:
			try:
				#self.refrGraph_esq()
				self.refrGraph_dir()
				
				plt.grid()
				#plt.draw()
				plt.pause(0.01)  # Pausa para permitir a atualização do gráfico
			except KeyboardInterrupt:
				break

		#plt.close(self.fig1)
		#plt.close(self.fig2)
		plt.show()
		spin_t.join()


if __name__ == '__main__':
	plotter = Plotter()
	plotter.run()