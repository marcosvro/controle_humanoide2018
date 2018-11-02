import socket
import threading
import time
import os

class Visao ():

	def __init__(self, ip='localhost', port=24700):
		self.clientsocket = None
		self.controlador_state = -1
		self.ip = ip
		self.port = port
		self.open_socket()
		self.espera_conexao()
		self.fps_count = 0
		self.last_time = 0
		self.count_frames = 0



	def open_socket(self):
		self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.serversocket.bind((self.ip, self.port))
		self.serversocket.listen(5)
		print("Servidor da Visao aberto!")

	def espera_conexao(self):
		print("Aguardando controlador se conectar..")
		(self.clientsocket, self.address_client) = self.serversocket.accept()
		t = threading.Thread(target=self.escuta_controlador)
		t.daemon = True
		t.start()

	def escuta_controlador (self):
		print("Escutando controlador!")
		while(True):
			try:
				flag = self.clientsocket.recv(1)
			except Exception as e:
				print("Conexão encerrada pelo controlador!")
				if self.clientsocket is not None:
					self.clientsocket.close()
				self.espera_conexao()
				break
			
			self.controlador_state = int(flag)

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
				#execute aqui seu codigo
				time.sleep(1)

				'''if (self.atualiza_fps() is not None):
					os.system('cls' if os.name == 'nt' else 'clear')
					print(self.fps_count)
	'			'''


				########################### AQUI VEM O SEU CODIGO ##################################3




				if(self.clientsocket is not None):
					#publica direcao e distância
					try:
						self.clientsocket.send(b"VIRA PARA A DEIREITA!!")
					except Exception as e:
						if self.clientsocket is not None:
							self.clientsocket.close()
							self.clientsocket = None
			except KeyboardInterrupt as e:
				print("Finalizado pelo usuario!!")
				self.clientsocket.close()
				self.serversocket.close()
				break
			


if __name__ == '__main__':
	a = Visao()
	a.run()
