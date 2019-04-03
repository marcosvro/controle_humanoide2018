#PARAMETROS
rst_imu_pin = 18
simTransRate = 1/125

#CONFIGURA NÓ ROS PARA PUBLICAÇÃO DE DADOS
print("Iniciando ROS node para execcao do simulador..")
pub = rospy.Publisher('/vrep_ros_interface/Bioloid/robot_inertial_sensor', Float32MultiArray, queue_size=1)
rospy.init_node('IMU', anonymous=True)
rate = rospy.Rate(1/125)

#CONFIGURA IMU PARA LEITURA DE DADOS DO SENSOR
while 1:
	try:
		bno = BNO055.BNO055(rst=rst_imu_pin, address=BNO055.BNO055_ADDRESS_B)
		if not bno.begin(BNO055.OPERATION_MODE_IMUPLUS):
			raise Exception("ERRO:Nao foi possivel inicializar BNO055!")
		status, self_test, error = bno.get_system_status()
		if status == 0x01:
			raise Exception('System error')
		break
	except Exception as e:
		print(str(e))
		time.sleep(1)


mat = Float32MultiArray()
while not rospy.is_shutdown():
	try:
		yall, pitch, roll = bno.read_euler()
		sys, gyro, accel, mag = bno.get_calibration_status()
		if sys != 2: #calibrando
			continue
		if yall+roll+pitch == 0:
			contagem_nulo += 1
		else:
			contagem_nulo = 0

			mat.data = [roll, pitch, yall]
			pub.publish(mat)
			rospy.sleep(simTransRate)
		if contagem_nulo > 20:
			raise Exception("Bugou")

	except Exception as e:
		print("ERRO: Comunicao com o IMU foi perdida!!")
		self.incercial_ativado = True
		self.inicia_modulo_inercial()
		break


	