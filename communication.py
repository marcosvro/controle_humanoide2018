import socket
import threading
from threading import Thread
import time
import os

class Communication(Thread):

	def __init__(self, gimbal, flags, ip='0.0.0.0', port=24700, sendDelay=0.1):
		Thread.__init__(self)
		self.clientsocket = None
		self.controlador_state = -1
		self.ip = ip
		self.port = port
		self.open_socket()
		self.wait_conection()
		self.fps_count = 0
		self.last_time = 0
		self.count_frames = 0
		self.gimbal = gimbal
		self.flags = flags
		self.sendDelay = sendDelay
		self.gimbal_tilt = 0
		self.gimbal_pan = 0


	def open_socket(self):
		self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		try:
			self.serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
			self.serversocket.bind((self.ip, self.port))
		except Exception:
			time.sleep(0.1)
			self.open_socket()
		self.serversocket.listen(5)
		print("Servidor da Visao aberto!")

	def wait_conection(self):
		print("Aguardando controlador se conectar..")
		t1 = threading.Thread(target=self.continous_listenning)
		t1.daemon = True
		t1.start()
		#(self.clientsocket, self.address_client) = self.serversocket.accept()
		t2 = threading.Thread(target=self.escuta_controlador)
		t2.daemon = True
		t2.start()

	def continous_listenning(self):
		while(True):
			(self.clientsocket, self.address_client) = self.serversocket.accept()

	def escuta_controlador (self):
		print("Escutando controlador!")
		while(True):
			if(self.clientsocket is not None):
				try:
					flag = self.clientsocket.recv(1000)
					if (flag == b''):
						raise Exception("Bugou")
					data = eval(flag)
					self.controlador_state = data[0]
					self.gimbal_tilt = data[1]
					self.gimbal_pan = data[2]
					self.gimbal.servoPan.real_angle = self.gimbal_pan
					self.gimbal.servoTilt.real_angle = self.gimbal_tilt
					print("Recebendo:", data)
				except Exception as e:
					#print(e)
					pass
					#print("Conexão encerrada pelo controlador!")
					#if self.clientsocket is not None:
					#	self.clientsocket.close()
					#continue

	def atualiza_fps(self):
		if (time.time() > self.last_time + 1.):
			self.fps_count = self.count_frames
			self.last_time = time.time() + 1.
			self.count_frames = 0
			return self.fps_count
		self.count_frames += 1
		return None

	def run(self):
		print("Visao pronta para detecção!!")
		while(True):
			try:
				#time.sleep(3)
				#execute aqui seu codigo
				#test_pitch = 0
				#test_yall = 0
				info = [self.gimbal.servoPan.target_angle_var, self.gimbal.servoTilt.target_angle_var, self.gimbal.servoPan.old_angle, self.gimbal.servoTilt.old_angle,
						self.flags['search'], self.flags['ball'], self.flags['turn90']]
				#info = [test_pitch, test_yall, self.gimbal_tilt, self.gimbal_pan, self.flags['search'], self.flags['ball'], self.flags['turn90']]
				time.sleep(self.sendDelay)

				'''if (self.atualiza_fps() is not None):
					os.system('cls' if os.name == 'nt' else 'clear')
					print(self.fps_count)
	'			'''
				info = [0]*7
				angulo = input("Informe o angulo para o robo virar (-180, 180)!")
				info[1] = angulo
				angulo = input("Informe o angulo para o robo andar (-180, 180)!")
				info[0] = angulo
				info[5] = 1
				if(self.clientsocket is not None):
					#publica direcao e distância
					try:
						lista_string = "["+",".join(str(x) for x in info)+"]"
						self.clientsocket.send(lista_string.encode('utf8'))
						print('Enviado:'+lista_string)
					except Exception as e:
						print(e)
						print('Falha no envio')
						if self.clientsocket is not None:
							self.clientsocket.close()
							self.clientsocket = None
			except KeyboardInterrupt as e:
				print("Finalizado pelo usuario!!")
				self.clientsocket.close()
				self.serversocket.close()
				break


if __name__ == '__main__':
	from utils.servo_gimbal import Servo, Gimbal
	pan = Servo()
	tilt = Servo()
	gimbal = Gimbal(pan,tilt)
	pan.loop()
	tilt.loop()

		
	flags = {
		"search": 1,
		"ball": 0,
		"turn90": 0,
	}

	a = Communication(gimbal,flags)
	#a.daemon = True
	#a.start()
	a.run()

	while True:
		time.sleep(500)
