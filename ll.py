###################################################################################################################
#Imports
###################################################################################################################

import threading
import time
import serial
from threading import Thread
from sr.robot import *
from math import *

###################################################################################################################
#Constants
###################################################################################################################

R = Robot()
S = serial.Serial('/dev/DEVICE',9600)

motorPowerDrive = 25
motorPowerRotate = 20

wheelRadius = 0.0508#radius of the wheels attached to the motors
wheelDiameter = 2*wheelRadius#diameter of the wheels attached to the motors
wheelCircumference = wheelDiameter*pi

robotCircumference = "unknown"

ticksPerRevolution = 64
ticksPerMeter = 3200

robotCameraResolutionX = 1280#this is the robot cameras X resolution
robotCameraResolutionY = 1024#this is the robot cameras Y resolution

###################################################################################################################
#Hardware communication
###################################################################################################################

def getTicks(motor):
    if motor == 'L':
		return R.ruggeduinos[0].motorStatusL()
	elif motor == 'R':
	    return R.ruggeduinos[0].motorStatusR()
	else:
		raise ValueError()

def resetTicks():
    R.ruggeduinos[0].motorStatusL()
    R.ruggeduinos[0].motorStatusR()
    
def lowLevelServoState(state):
    if state=="open":
        R.servos[0][0]="needs to be tried out"
        R.servos[0][1]="need to be tried out"
        
    elif state=="close":
        R.servos[0][0]="needs to be tried out"
        R.servos[0][1]="need to be tried out"
        
    else:
        print("<-------------------------- Something is wrong | input: " + state +" -------------------------->
    
###################################################################################################################
#Class CustomisedRuggeduino
###################################################################################################################

class CustomisedRuggeduino(Ruggeduino):
    def readUS(self):
        with self.lock:
            resp=self.command("b")
        return int(resp)
        
    def motorReset(self):
    	with self.lock:
			resp=self.command("u")
		return int(resp)
		
    def motorStatusL(self):
        with self.lock:
            resp=self.command("w")
        return abs(int(resp))
		
    def motorStatusR(self):
        with self.lock:
            resp=self.command("x")
        return abs(int(resp))
        
R = Robot.setup()
R.ruggeduino_set_handler_by_fwver("SRcustom", CustomisedRuggeduino)
R.init()
R.wait_start()
