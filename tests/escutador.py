import rospy
from std_msgs.msg import String
import time

class Test():
	def __init__(self):
		self.t0 = time.time()
		self.next_t = self.t0 + 1
		self.fps = 0

		rospy.init_node('escutador')
		rospy.Subscriber('/Bioloid/resposta_sim', String, self.callback)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

	def callback(self, msg):
		self.fps += 1
		if time.time() > self.next_t:
			self.next_t = time.time() + 1
			print(self.fps)
			self.fps = 0

if __name__ == '__main__':
	a = Test()