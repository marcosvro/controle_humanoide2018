import threading
import rospy
from os import system
from time import sleep
from roslaunch.parent import ROSLaunchParent

parent = ROSLaunchParent('',[],is_core=True)
parent.start()

while not rospy.is_shutdown():
	sleep(0.5)
	if rospy.get_param('shutdown', False):
		parent.shutdown()
