#!/usr/bin/python
#
# Python Module to externalise all Initio/PiRoCon specific hardware
#
# Created by Gareth Davies, Sep 2013
# Updated May 2014, Feb 2015
# Copyright 4tronix
#
# This code is in the public domain and may be freely copied and used
# No warranty is provided or implied
#
#======================================================================


#======================================================================
# General Functions
#
# init(). Initialises GPIO pins, switches motors off, etc
# cleanup(). Sets all motors off and sets GPIO to standard values
# version(). Returns 1. Invalid until after init() has been called
#======================================================================


#======================================================================
# Motor Functions
#
# stop(): Stops both motors
# forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
# reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
# turnreverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
#======================================================================


# Import all necessary libraries
import RPi.GPIO as GPIO, sys, threading, time, os, subprocess
import math

#use physical pin numbering
GPIO.setmode(GPIO.BOARD)
#======================================================================
# General Functions
#
# init(). Initialises GPIO pins, switches motors and LEDs Off, etc
#======================================================================
# Wheel Encoder Functions
#
class WheelEncoder():
    def __init__(self):
        self.radius = 55
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
        self.state = {"elapsedTime":self.elapsedTime, "distance (m)":self.distance/100, "totalDistance (m)": self.totalDistance/100, "speed (m/s)": self.speed/100, "acceleration (m/s^2)":self.acceleration/100}
        return self.state

class Initio():
    def init(self):
        #Pins
        # Pins 24, 26 Right Motor
        # Pins 19, 21 Left Motor
        self.R1 = 11
        self.R2 = 12
        self.L1 = 13
        self.L2 = 15

        #Pins Wheel encoder
        self.ENCODER1 = 16 #Wheel Encoder right
        self.ENCODER2 = 19 #Wheel Encoder left


        #Setup wheel encoder
        self.ticksR = 0
        self.ticksL = 0
        GPIO.setup(self.ENCODER1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.ENCODER2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.wheelEncoderR = WheelEncoder()
        self.wheelEncoderL = WheelEncoder()

        #Setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.R1, GPIO.OUT)
        GPIO.setup(self.R2, GPIO.OUT)
        GPIO.setup(self.L1, GPIO.OUT)
        GPIO.setup(self.L2, GPIO.OUT)


        #Setup PWM
        self.leftF = GPIO.PWM(self.R1,50)
        self.leftB = GPIO.PWM(self.R2,50)
        self.ightF = GPIO.PWM(self.L1,50)
        self.rightB = GPIO.PWM(self.L2,50)
        # Initialise the PWM device using the default address

        
        #print GPIO.RPI_REVISION
        
        #use pwm on inputs so motors don't go too fast
        GPIO.setup(self.L1, GPIO.OUT)
        self.p = GPIO.PWM(self.L1, 20)
        self.p.start(0)

        GPIO.setup(self.L2, GPIO.OUT)
        self.q = GPIO.PWM(self.L2, 20)
        self.q.start(0)

        GPIO.setup(self.R1, GPIO.OUT)
        self.a = GPIO.PWM(self.R1, 20)
        self.a.start(0)

        GPIO.setup(self.R2, GPIO.OUT)
        self.b = GPIO.PWM(self.R2, 20)
        self.b.start(0)
        
        GPIO.add_event_detect(self.ENCODER1, GPIO.RISING, callback=self.encoderCallbackR,bouncetime=2)
        GPIO.add_event_detect(self.ENCODER2, GPIO.RISING, callback=self.encoderCallbackL,bouncetime=2)

    # cleanup(). Sets all motors off and sets GPIO to standard values
    def cleanup(self):
        self.stop()
        GPIO.cleanup()

    # version(). Returns 1. Invalid until after init() has been called
    def version(self):
        return 1

    # End of General Functions
    #======================================================================


    #======================================================================
    # Motor Functions
    #
    # stop(): Stops both motors
    def stop(self):
        self.p.ChangeDutyCycle(0)
        self.q.ChangeDutyCycle(0)
        self.a.ChangeDutyCycle(0)
        self.b.ChangeDutyCycle(0)
        
    # forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
    def forward(self,speed):
        self.p.ChangeDutyCycle(speed)
        self.q.ChangeDutyCycle(0)
        self.a.ChangeDutyCycle(speed)
        self.b.ChangeDutyCycle(0)
        self.p.ChangeFrequency(speed + 5)
        self.a.ChangeFrequency(speed + 5)
        
    # reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
    def reverse(self,speed):
        self.p.ChangeDutyCycle(0)
        self.q.ChangeDutyCycle(speed)
        self.a.ChangeDutyCycle(0)
        self.b.ChangeDutyCycle(speed)
        self.q.ChangeFrequency(speed + 5)
        self.b.ChangeFrequency(speed + 5)

    # spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
    def spinLeft(self,speed):
        self.p.ChangeDutyCycle(0)
        self.q.ChangeDutyCycle(speed)
        self.a.ChangeDutyCycle(speed)
        self.b.ChangeDutyCycle(0)
        self.q.ChangeFrequency(speed + 5)
        self.a.ChangeFrequency(speed + 5)
        
    # spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
    def spinRight(self,speed):
        self.p.ChangeDutyCycle(speed)
        self.q.ChangeDutyCycle(0)
        self.a.ChangeDutyCycle(0)
        self.b.ChangeDutyCycle(speed)
        self.p.ChangeFrequency(speed + 5)
        self.b.ChangeFrequency(speed + 5)
        
    # turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
    def turnForward(self,leftSpeed, rightSpeed):
        self.p.ChangeDutyCycle(leftSpeed)
        self.q.ChangeDutyCycle(0)
        self.a.ChangeDutyCycle(rightSpeed)
        self.b.ChangeDutyCycle(0)
        self.p.ChangeFrequency(leftSpeed + 5)
        self.a.ChangeFrequency(rightSpeed + 5)
        
    # turnReverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
    def turnReverse(self,leftSpeed, rightSpeed):
        self.p.ChangeDutyCycle(0)
        self.q.ChangeDutyCycle(leftSpeed)
        self.a.ChangeDutyCycle(0)
        self.b.ChangeDutyCycle(rightSpeed)
        self.q.ChangeFrequency(leftSpeed + 5)
        self.b.ChangeFrequency(rightSpeed + 5)

# End of Motor Functions
#======================================================================

#======================================================================
# Wheel Encoder functions
#
# stop(): Stops both motors
    def encoderCallbackR(self,pin):
        self.ticksR += 1
        
    def encoderCallbackL(self,pin):
        self.ticksL += 1

    def getState(self):
        self.stateR = self.wheelEncoderR.updateState(self.ticksR)
        self.stateL = self.wheelEncoderL.updateState(self.ticksL)
        return self.stateR, self.stateL




