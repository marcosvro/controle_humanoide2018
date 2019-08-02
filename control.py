from subprocess import Popen
import threading
import rospy
from communication import Visao
from os import system, getpid
from time import sleep
import sys
from roslaunch.parent import ROSLaunchParent


rosserial_process = "rosserial_process"
port = "ttyACM0"
baud = 500000

py3_env = "humanoid"
py2_env = ""


parent = ROSLaunchParent('roscore',[],is_core=True)
parent.start()

while rospy.is_shutdown():
	sleep(0.5)
p = Popen("bash -i ./start.sh " + str(py3_env) + " " + str(port) + " " + str(baud), shell=True)

c = Popen("bash -i .start_controlador.sh " + str(py3_env))

visao = Visao()

try:
	visao.wait_for_cmd()
	from signal import SIGINT, SIGKILL
	c.send_signal(SIGKILL)
except Exception as e:
	pass
while not rospy.is_shutdown():
	sleep(0.5)

p.send_signal(SIGINT)
	
