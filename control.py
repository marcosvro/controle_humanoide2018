
import threading
import rospy
from controlador import Controlador
from communication import Visao
from os import system
from time import sleep
from roslaunch.parent import ROSLaunchParent

OBJECTS = {}

rosserial_node = "rosserial_node"
port = "ttyACM0"
baud = 500000



def startController():
	control = Controlador(time_id = 17,
						robo_id = 0,
						simulador_enable=False,
						inertial_foot_enable=False,
						gravity_compensation_enable=True)
	control.run()


parent = ROSLaunchParent('roscore',[],is_core=True)
parent.start()

while rospy.is_shutdown():
	sleep(0.5)
system(f"rosrun rosserial_python serial_node.py _port:=/dev/{port} _baud:={baud} &")
t = threading.Thread(target=startController)
t.daemon = True
t.start()

visao = Visao()
try:
	visao.wait_for_cmd()
except Exception as e:
	parent.shutdown()

	
