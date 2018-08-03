import car
import time
import logging
import RPi.GPIO as GPIO
import math

import sys
import tty
import termios
class WheelEncoder():
    def __init__(self):
        self.radius = 5.5
        self.circum = self.radius * math.pi
        self.ticks = 0
        self.startTime = time.time()
        self.lastTime = self.startTime
        self.elapsedTime = 0.0
        self.distance = 0.0
        self.acceleration = 0.0
        self.totalDistance = 0.0
        self.speed = 0.0
    
    def updateRevolutions(self):
        self.revolutions = int(self.ticks/36)
        self.ticks = 0

    def updateElapsedTime(self):
        thisTime = time.time()
        self.elapsedTime = thisTime - self.lastTime
        self.lastTime = thisTime

    def updateDistance(self):
        self.distance = self.circum * self.revolutions
        self.totalDistance += self.distance

    def updateSpeed(self,speed):
        self.speed = speed

    def getSpeed(self):
        speed = self.distance/self.elapsedTime
        return speed

    def updateAcceleration(self):
        self.updateRevolutions()
        self.updateElapsedTime()
        self.updateDistance()
        currentSpeed = self.getSpeed()
        self.acceleration = (currentSpeed-self.speed)/self.elapsedTime
        self.updateSpeed(currentSpeed)
    
    def updateState(self,ticks):
        self.ticks = ticks
        self.updateAcceleration()
        return self.getState()
    
    def getState(self):
        self.state = {"elapsedTime":self.elapsedTime, "distance (m)":self.distance/100, "totalDistance (m)": self.totalDistance/100, "speed (m/s)": self.speed/100, "acceleration (m/s^2)":self.acceleration/100, "revolutions": self.revolutions}
        return self.state
    
class Monitor():
    def init(self):
        #Setup wheel encoder
        self.wheelEncoderInterval = 1
        self.ENCODER1 = 16 #Wheel Encoder right
        self.ENCODER2 = 19 #Wheel Encoder left
        self.ticksR = 0
        self.ticksL = 0
        GPIO.setup(self.ENCODER1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.ENCODER2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.wheelEncoderR = WheelEncoder()
        self.wheelEncoderL = WheelEncoder()
        GPIO.add_event_detect(self.ENCODER1, GPIO.RISING, callback=self.encoderCallbackR,bouncetime=2)
        GPIO.add_event_detect(self.ENCODER2, GPIO.RISING, callback=self.encoderCallbackL,bouncetime=2)
        logging.basicConfig(filename='log.txt',level=logging.INFO)
        
    def encoderCallbackR(self,pin):
        self.ticksR += 1
        
    def encoderCallbackL(self,pin):
        self.ticksL += 1
        
    def runWheelMonitor(self):
        while True:
            time.sleep(self.wheelEncoderInterval)
            self.stateR = self.wheelEncoderR.updateState(self.ticksR)
            self.stateL = self.wheelEncoderL.updateState(self.ticksL)
            self.ticksR = 0
            self.ticksL = 0
            logging.info(self.stateR,self.stateL)
        
monitor = Monitor()
monitor.init()
monitor.runWheelMonitor()
        

