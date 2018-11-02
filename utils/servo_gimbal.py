import time
import threading

import serial
import numpy as np
import struct

def _loop(servo):
        t = threading.currentThread()
        flag = False
        while getattr(t, "do_run", True):
            if(flag):
                if(not servo.moveplus()):
                    flag = False
                    servo.moveminus()
            else:
                if(not servo.moveminus()):
                    flag = True
                    servo.moveplus()
            time.sleep(servo.delay)

def _timer(servo):
    #serv.port.ChangeangleCycle(0)
    servo.TimerRunning = False

def _move(servo, target):
    while(not(servo.getAngle() >= target-servo.step*2 and servo.getAngle() <= target+servo.step*2)):
        servo.looping = True
        #print('moving_target thread running...')
        if(servo.getAngle() < target):
            servo.moveplus()
        else:
            servo.moveminus()
        time.sleep(servo.delay)
    servo.setAngle(target)
    servo.looping = False

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

class Servo:
    def __init__(self, step=1, delay = 0.01, angle_min_max=(-35,35)):

        self.step = step
        self.angle_min_max = angle_min_max
        self.angle = 0
        self.delay = delay

        self.target_angle_var = 0
        self.old_angle = 0
        self.real_angle = 0
        
        self.looping = False
        
        self.TimerRunning = False
        
        self.flag = False
        self.setTimer()

    def setTimer(self, time=0.5):
        if(self.TimerRunning):
            self.timer.cancel()       
        else:
            self.TimerRunning = True
        self.timer = threading.Timer(time, _timer, args=(self,))
        self.timer.start()

    def moveplus(self):
        if(self.angle < self.angle_min_max[1]):
            self.setTimer()
            return self.setAngle(self.getAngle() + self.step)
            self.target_angle_var = self.step
        return False

    def moveminus(self):
        if(self.angle > self.angle_min_max[0]):
            self.setTimer()
            return self.setAngle(self.getAngle() - self.step)
            self.target_angle_var = -self.step
        return False

    def loop(self):
        t = threading.Thread(target=_loop, args=(self,))
        t.daemon = True
        t.start()
        self.loop_thread = t
        self.looping = True
    
    def loopByStep(self):
        if(self.flag):
            if(not self.moveplus()):
                self.flag = False
                self.moveminus()
        else:
            if(not self.moveminus()):
                self.flag = True
                self.moveplus()
        #print('loopBySTep')

    def getAngle(self):
        return self.angle

    def setAngle(self, angle):
        self.old_angle = self.angle
        if(angle >= self.angle_min_max[0] and angle <= self.angle_min_max[1]):
                self.angle = angle
                return True
        else:
            if(angle < self.angle_min_max[0]):
                self.angle = self.angle_min_max[0]
            if(angle > self.angle_min_max[1]):
                self.angle = self.angle_min_max[1]
        return False

    def moveToAngle(self, angle):
        if(not self.looping):
            if(angle >= self.angle_min_max[0] and angle <= self.angle_min_max[1]):
                angle_target = angle
            else:
                if(angle < self.angle_min_max[0]):
                    angle_target = self.angle_min_max[0]
                if(angle > self.angle_min_max[1]):
                    angle_target = self.angle_min_max[1]
            #self.port.ChangeangleCycle(angle_target)
            self.stepToAngle(angle_target)
            #self.setTimer(time=0.1)

    def stepToAngle(self, angle_target):
        t = threading.Thread(target=_move, args=(self, angle_target,))
        t.daemon = True
        t.start()

    def stop(self):
        if(self.looping):
            self.loop_thread.do_run = False
            self.loop_thread.join()
            self.looping = False

class Gimbal():
    def __init__(self, servoPan, servoTilt, delay = 0.001, ports=(1,2)):
        self.servoTilt = servoTilt
        self.servoPan = servoPan
        self.delay = delay
        self.ports = ports
        #self.serial = serial.Serial('/dev/ttyUSB0', 230400, timeout=0)
        self.data_pelv = [0]*8

    def loopSerialSend(self):
        try:
            while 1:
                time.sleep(self.delay)
                self.data_pelv = [0 for i in self.data_pelv]
                self.data_pelv[self.ports[0]] = self.servoPan.getAngle()
                self.data_pelv[self.ports[1]] = self.servoTilt.getAngle()
                ##print(self.data_pelv)
                self.data_pelv = [90+i for i in self.data_pelv]
                send_pelv = np.array([255]+self.data_pelv+[254], dtype=np.uint8)
                #self.serial.write(struct.pack('>10B', *(send_pelv.tolist())))
        except Exception as e:
            print(e)

    def run(self):
        t = threading.Thread(target=self.loopSerialSend)
        t.daemon = True
        t.start()
        

if __name__ == "__main__":
    tilt = Servo(angle_min_max=(-35,35),delay = 0.0075)
    pan = Servo(angle_min_max=(-90,90),delay = 0.0025)
    gimbal = Gimbal(pan,tilt)
    #pan.loop()
    #tilt.loop()
    gimbal.run()
    pan.moveToAngle(60)
    
    while True:
        for i in range(-90,90,1):
            pan.moveToAngle(i)
            time.sleep(0.1)
        
    
