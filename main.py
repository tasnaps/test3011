#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor
from pybricks.nxtdevices import LightSensor
ev3 = EV3Brick()

# Wheels on b and c for correct lego calibration
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
base = DriveBase(left_motor, right_motor, 60, 170)
left_ir_sensor = Ev3devSensor(Port.S1)
right_ir_sensor = Ev3devSensor(Port.S2)
left_ir_sensor_value = left_ir_sensor.read("DC")
right_ir_sensor_value = right_ir_sensor.read("DC")

# when we set the robot in arena, must face forward we update this in degrees turned

heading = 0
locationToLine = 0
rotation = 0

def deduct(value):
    returnable = value
    if value>1:
        returnable =-1
        deduct(returnable)
    if value<-1:
        returnable =+1
        deduct(returnable)

# Problem might be in this function
def getRotation():
    #Cleaning data
    newHeading = heading/360
    deduct(newHeading)
    #newHeading = newHeading * 360
    print("heading is: " + str(newHeading))
    return abs(newHeading)



def updateLocation():
    tempHeading = getRotation()
    #Update on the location based on which side we have moved. 
    if tempHeading<0.5:
        locationToLine += 150
    if tempHeading>0.5:
        locationToLine -= 150
    

        
def moveToLine():
    tempHeading = getRotation() * 360

    #we are on the left side of the line
    rotation = -tempHeading + 90
    if locationToLine<0:
        base.turn(rotation)
        readAndMove()
        base.turn(-90)
        base.straight(10000)
    
    #we are on the right side of the line
    if locationToLine>0:
        base.turn(rotation+180)
        readAndMove()
        base.turn(90)
        base.straight(10000)

    #read the third sensor value and move forward until we hit the black line
    #once we hit the line we turn depending on approach and move forward.

#helper method for recursion
def readAndMove():
    #TODO Need to check what value we get on black line 
    color_sensor = ColorSensor(Port.S3)
    color_sensor_value = color_sensor.reflection()
    print(color_sensor_value)
    if(color_sensor_value > 3):
        base.straight(10)
        readAndMove()


# one option is to only use one sensor, if we install it on the middle. Then the values should be ok when ball is in the middle.
def movement(left_ir_sensor, right_ir_sensor):
    left_ir_sensor_value = left_ir_sensor.read("DC")
    right_ir_sensor_value = right_ir_sensor.read("DC")

    if(right_ir_sensor_value[0] == 0 and left_ir_sensor_value[0] == 0):
        print("ball behind")
        base.turn(180)
        heading =+180
        movement(left_ir_sensor, right_ir_sensor)

    if right_ir_sensor_value[0]>5:#6
        print("ball slightly to the right")
        base.turn(20)
        heading =+20
        movement(left_ir_sensor, right_ir_sensor)

    if left_ir_sensor_value[0] < 5:#4
        print("ball slightly to the left")
        base.turn(-25)
        heading =-25
        movement(left_ir_sensor, right_ir_sensor)
    
    if left_ir_sensor_value[0]>7 and right_ir_sensor_value[0]<4:
        print("ball between sensors")
        # Call move to line function
        moveToLine()
        
    else:
        base.straight(150)
        updateLocation()
        movement(left_ir_sensor, right_ir_sensor)

movement(left_ir_sensor, right_ir_sensor)