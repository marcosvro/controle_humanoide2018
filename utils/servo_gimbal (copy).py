import time
import RPi.GPIO as GPIO
import threading

import serial
import numpy as np
import struct

#oloko
def _loop(serv, delay):
        t = threading.currentThread()
        flag = False
        while getattr(t, "do_run", True):
            if(flag):
                if(not serv.moveplus()):
                    flag = False
            else:
                if(not serv.moveminus()):
                    flag = True
            time.sleep(delay)

def _timer(serv):
    #serv.port.ChangeDutyCycle(0)
    serv.TimerRunning = False

def _move(serv, target):
    while(not(serv.duty >= target-serv.step or serv.duty <= target+serv.step)):
        if(target < serv.duty):
            serv.moveplus()
        else:
            serv.moveminus()
        print('looping')
    serv.looping = False

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

class Servo:
    def __init__(self, gpio_port, step=1, frequency=100, duty_min_max=(-90,90), timerDelay = 0.5, angle_min_max=(-90,90)):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(gpio_port, GPIO.OUT)

        self.step = step
        self.duty_min_max = duty_min_max
        self.angle_min_max = angle_min_max
        self.duty = (duty_min_max[0]+duty_min_max[1])/2
        self.port = GPIO.PWM(gpio_port, frequency)  # up down
        self.port.start(self.duty)

        self.looping = False
        
        self.TimerRunning = False

        self.timerDelay = timerDelay
        
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
        if(self.duty < self.duty_min_max[1]):
            self.duty += self.step
            self.port.ChangeDutyCycle(self.duty)
            self.setTimer()
            return True
        return False

    def moveminus(self):
        if(self.duty > self.duty_min_max[0]):
            self.duty -= self.step
            self.port.ChangeDutyCycle(self.duty)
            self.setTimer()
            return True
        return False

    def loop(self, delay = 0.1):
        t = threading.Thread(target=_loop, args=(self, delay,))
        t.start()
        self.loop_thread = t
        self.looping = True
    
    def loopByStep(self):
        if(self.flag):
            if(not self.moveplus()):
                self.flag = False
        else:
            if(not self.moveminus()):
                self.flag = True
        print('loopBySTep')

    def getAngle(self):
        return translate(self.duty, self.duty_min_max[0], self.duty_min_max[1], self.angle_min_max[0], self.angle_min_max[1])

    def setAngle(self, angle):
        if(not self.looping):
            if(angle >= self.angle_min_max[0] and angle <= self.angle_min_max[1]):
                duty_target = translate(angle, self.angle_min_max[0], self.angle_min_max[1], self.duty_min_max[0], self.duty_min_max[1])
            else:
                if(angle < self.angle_min_max[0]):
                    duty_target = self.duty_min_max[0]
                if(angle > self.angle_min_max[1]):
                    duty_target = self.duty_min_max[1]
            #self.port.ChangeDutyCycle(duty_target)
            self.stepToDuty(duty_target)
            #self.setTimer(time=0.1)

    def stepToDuty(self, target):
        t = threading.Thread(target=_move, args=(self, target,))
        t.start()
        self.looping = True

    def stop(self):
        if(self.looping):
            self.loop_thread.do_run = False
            self.loop_thread.join()
            self.looping = False
            self.port.ChangeDutyCycle(0)

class Gimbal():
    def __init__(self, servoTilt, servoPan, ports=(1,2)):
        self.servoTilt = servoTilt
        self.servoPan = servoPan
        self.ports = ports
        ser = serial.Serial('/dev/ttyUSB0', 230400, timeout=0)
        self.data_pelv = [0]*8

    def loopSerialSend(self):
        while True:
            self.data_pelv[self.port[0]] = servoTilt.duty
            self.data_pelv[self.port[1]] = servoPan.duty
            data_pelv = [90+i for i in data_pelv]
            send_pelv = np.array([255]+data_pelv+[254], dtype=np.uint8)
            ser.write(struct.pack('>10B', *(send_pelv.tolist())))

    def run(self):
        t = threading.Thread(target=self.loopSerialSend, args=(self,))
        t.start()

if __name__ == "__main__":
    tilt = Servo(40)
    pan = Servo(38)
    gimbal = Gimbal(tilt,pan)
    pan.loop()
    while True:
        continue
