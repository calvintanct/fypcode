#!/usr/bin/env python

from lib_robotis import*
import math
import time

gripper = USB2Dynamixel_Device('/dev/ttyUSB0')
motor_link1= Robotis_Servo(gripper,1,series='MX')
motor_link2= Robotis_Servo(gripper,2,series='MX')
motor_suction= Robotis_Servo(gripper,3,series='MX')

def move_motor(id,tetha):
	if(id == 1):
		motor_link1.move_angle(tetha, blocking = False)
	elif(id == 2):
		motor_link2.move_angle(tetha, blocking = False)
	elif(id==3):
		motor_suction.move_angle(tetha, blocking = False)
	else:
		str ="Wrong Input ID"

    #ID1    (180(initial) - 270(max))
    #ID2    (230(initial))
    #       (10(open)-180(close) - fully out position)
    #ID 3   (180(initial)-70(final open))
    #ALL ANGLE WILL BE IN THE RANGE OF -180 to 180 (need to be substracted by 180)

angle3=math.radians(70-180)
print("open3")
motor_suction.move_angle(angle3, blocking = False)
time.sleep(2)

angle1=math.radians(180-180)
angle2=math.radians(230-180)
angle3=math.radians(180-180)
print("open1")
motor_link1.move_angle(angle1+math.radians(45), blocking = False)
print("open2")
motor_link2.move_angle(angle2-math.radians(100), blocking = False)
time.sleep(1)
print("open1")
motor_link1.move_angle(angle1, blocking = False)
print("open2")
motor_link2.move_angle(angle2, blocking = False)
time.sleep(1)
motor_suction.move_angle(angle3, blocking = False)
time.sleep(2)
