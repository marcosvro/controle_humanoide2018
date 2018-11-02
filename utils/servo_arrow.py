
import sys,tty,termios

from servo_gimbal import Servo, Gimbal
class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get():
    inkey = _Getch()
    while(1):
        k=inkey()
        if k!='':break
    if k=='\x1b[A':
        return "up"
    elif k=='\x1b[B':
        return "down"
    elif k=='\x1b[C':
        return "right"
    elif k=='\x1b[D':
        return "left"
    elif k=="qqq":
        return "quit"
    else:
        return "invalid"

import time

tilt = Servo(angle_min_max=(-35,35),delay = 0.01)
pan = Servo(angle_min_max=(-90,90),delay = 0.01)
gimbal = Gimbal(pan,tilt)
gimbal.run()

try:
    while(1):
        command = get()
        if (command == "up"):
            tilt.moveminus()
        elif (command == "down"):
            tilt.moveplus()
        elif (command == "right"):
            pan.moveplus()
        elif (command == "left"):
            pan.moveminus()
        elif (command == "quit"):
            break

        print("tilt: {}, pan: {}".format(tilt.getAngle(),pan.getAngle()))
        
except KeyboardInterrupt as e:
    pass