import time
import struct
from serial import Serial
from functools import reduce
from sys import stdout

class MockControll:
  def openSerial(self, delay = 0.5):
    i = 0
    while True:
      try:
        self.serial = Serial('/dev/ttyACM' + str(i), 512000)
        print("Opened port /dev/ttyACM" + str(i) + " successfully!")
        break
      except:
        self.serial = None
        print("Couldn't open port /dev/ttyACM" + str(i) + "!")
        time.sleep(delay)
        i += 1
        if (i >= 10): i = 0
        
  def waitSign(self):
    print("Esperando sinal...")
    self.serial.flush();
    c = b''
    while c != b'\x40':
      c = self.serial.read()
      print(c)
      
    print("Got sign...")
        
  def sendGoal(self):
    print("Sending goal...\n")
    data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    cs = reduce(lambda x,y: x^y, struct.unpack('>42b', struct.pack('>2b20h', *([3, 45]+data))))
    header = [-3599, 3, 45, cs]
    msg = struct.pack('>h3b20h', *(header+data))
    self.serial.write(msg)
    
  def readResponse(self):
    print("Waiting for response...")
    c = self.serial.read()
    while c != b'\x24':
      stdout.write(c.decode())
      c = self.serial.read()
    stdout.write('\n\n')
    
