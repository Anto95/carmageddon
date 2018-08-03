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
        self.circum = radius * math.pi
        self.ticks = 0
        self.startTime = time.time()
        self.lastTime = self.startTime
        self.totalDistance = 0.0

    def updateRevolutions():
        self.revoultions = int(self.ticks/36)
        self.ticks = 0

    def updateElapsedTime():
        thisTime = time.time()
        self.elapsedTime = self.lastTime - thisTime
        self.lastTime = thisTime

    def updateDistance():
        self.distance = self.circum * self.revolutions
        self.totalDistance += self.distance

    def updateSpeed(speed):
        self.speed = speed

    def getSpeed():
        speed = self.distance/self.elapsedTime
        return speed

    def updateAcceleration():
        updateRevolutions()
        updateElapsedTime()
        updateDistance()
        currentSpeed = getSpeed()
        self.acceleration = (self.speed-currentSpeed)
        updateSpeed(currentSpeed)

    def getState(ticks):
        self.ticks = ticks
        updateRevolutions()
        updateElapsedTime()
        updateDistance()
        updateAcceleration()
        state = {"elapsedTime":self.elapsedTime, "distance":self.distance, "totalDistance": self.totalDistance, "speed": self.speed, "acceleration":self.acceleration}
        return state

class Initio():
    def init():

        #Pins
        # Pins 24, 26 Right Motor
        # Pins 19, 21 Left Motor
        R1 = 11
        R2 = 12
        L1 = 13
        L2 = 15

        #Pins Wheel encoder
        ENCODER1 = 16 #Wheel Encoder right
        ENCODER2 = 19 #Wheel Encoder left


        #Setup wheel encoder
        self.tickSR = 0
        self.ticksL = 0
        GPIO.setup(ENCODER1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(ENCODER2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        wheelEncoderR = WheelEncoder()
        wheelEncoderL = WheelEncoder()

        #Setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(R1, GPIO.OUT)
        GPIO.setup(R2, GPIO.OUT)
        GPIO.setup(L1, GPIO.OUT)
        GPIO.setup(L2, GPIO.OUT)


        #Setup PWM
        leftF = GPIO.PWM(IN1,50)
        leftB = GPIO.PWM(IN2,50)
        rightF = GPIO.PWM(IN3,50)
        rightB = GPIO.PWM(IN4,50)
        global p, q, a, b
        # Initialise the PWM device using the default address

        #use physical pin numbering
        GPIO.setmode(GPIO.BOARD)
        #print GPIO.RPI_REVISION

        #set up digital line detectors as inputs
        GPIO.setup(lineRight, GPIO.IN) # Right line sensor
        GPIO.setup(lineLeft, GPIO.IN) # Left line sensor

        #Set up IR obstacle sensors as inputs
        GPIO.setup(irFL, GPIO.IN) # Left obstacle sensor
        GPIO.setup(irFR, GPIO.IN) # Right obstacle sensor

        #use pwm on inputs so motors don't go too fast
        GPIO.setup(L1, GPIO.OUT)
        p = GPIO.PWM(L1, 20)
        p.start(0)

        GPIO.setup(L2, GPIO.OUT)
        q = GPIO.PWM(L2, 20)
        q.start(0)

        GPIO.setup(R1, GPIO.OUT)
        a = GPIO.PWM(R1, 20)
        a.start(0)

        GPIO.setup(R2, GPIO.OUT)
        b = GPIO.PWM(R2, 20)
        b.start(0)
        
        GPIO.add_event_detect(ENCODER1, GPIO.RISING, callback=encoderCallbackR,bouncetime=2)
        GPIO.add_event_detect(ENCODER2, GPIO.RISING, callback=encoderCallbackL,bouncetime=2)

    # cleanup(). Sets all motors off and sets GPIO to standard values
    def cleanup():
        stop()
        stopServos()
        GPIO.cleanup()

    # version(). Returns 1. Invalid until after init() has been called
    def version():
        return 1

    # End of General Functions
    #======================================================================


    #======================================================================
    # Motor Functions
    #
    # stop(): Stops both motors
    def stop():
        p.ChangeDutyCycle(0)
        q.ChangeDutyCycle(0)
        a.ChangeDutyCycle(0)
        b.ChangeDutyCycle(0)
        
    # forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
    def forward(speed):
        p.ChangeDutyCycle(speed)
        q.ChangeDutyCycle(0)
        a.ChangeDutyCycle(speed)
        b.ChangeDutyCycle(0)
        p.ChangeFrequency(speed + 5)
        a.ChangeFrequency(speed + 5)
        
    # reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
    def reverse(speed):
        p.ChangeDutyCycle(0)
        q.ChangeDutyCycle(speed)
        a.ChangeDutyCycle(0)
        b.ChangeDutyCycle(speed)
        q.ChangeFrequency(speed + 5)
        b.ChangeFrequency(speed + 5)

    # spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
    def spinLeft(speed):
        p.ChangeDutyCycle(0)
        q.ChangeDutyCycle(speed)
        a.ChangeDutyCycle(speed)
        b.ChangeDutyCycle(0)
        q.ChangeFrequency(speed + 5)
        a.ChangeFrequency(speed + 5)
        
    # spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
    def spinRight(speed):
        p.ChangeDutyCycle(speed)
        q.ChangeDutyCycle(0)
        a.ChangeDutyCycle(0)
        b.ChangeDutyCycle(speed)
        p.ChangeFrequency(speed + 5)
        b.ChangeFrequency(speed + 5)
        
    # turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
    def turnForward(leftSpeed, rightSpeed):
        p.ChangeDutyCycle(leftSpeed)
        q.ChangeDutyCycle(0)
        a.ChangeDutyCycle(rightSpeed)
        b.ChangeDutyCycle(0)
        p.ChangeFrequency(leftSpeed + 5)
        a.ChangeFrequency(rightSpeed + 5)
        
    # turnReverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
    def turnReverse(leftSpeed, rightSpeed):
        p.ChangeDutyCycle(0)
        q.ChangeDutyCycle(leftSpeed)
        a.ChangeDutyCycle(0)
        b.ChangeDutyCycle(rightSpeed)
        q.ChangeFrequency(leftSpeed + 5)
        b.ChangeFrequency(rightSpeed + 5)

# End of Motor Functions
#======================================================================

#======================================================================
# Wheel Encoder functions
#
# stop(): Stops both motors
    def encoderCallbackR(pin):
        ticksR += 1
        
    def encoderCallbackL(pin):
        ticksL += 1

    def getState():
        stateR = self.wheelEncoderR.getState()
        stateL = self.wheelEncoderL.getState()
        return stateR, stateL




