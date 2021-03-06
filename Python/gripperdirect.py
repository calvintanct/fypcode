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


angle3=math.radians(70-180)
move_motor(3, angle3)
time.sleep(2)

angle1=math.radians(270-180)
angle2=math.radians(10-180)
angle3=math.radians(70-180)
move_motor(3, angle3)
time.sleep(1)
move_motor(1, angle1-45)
move_motor(2, angle2+100)
time.sleep(1)
move_motor(1, angle1)
move_motor(2, angle2)
time.sleep(2)

time.sleep(7)

angle1=math.radians(180-180)
angle2=math.radians(230-180)
angle3=math.radians(180-180)
move_motor(1, angle1+45)
move_motor(2, angle2-100)
time.sleep(1)
move_motor(1, angle1)
move_motor(2, angle2)
time.sleep(2)
move_motor(3, angle3)
time.sleep(2)

    #ID1    (180(initial) - 270(max))
    #ID2    (230(initial))
    #       (10(open)-180(close) - fully out position)
    #ID 3   (180(initial)-70(final open))
    #ALL ANGLE WILL BE IN THE RANGE OF -180 to 180 (need to be substracted by 180)

def gripper_ready):
	str="Gripper ready for side"
	rospy.loginfo(str)
	angle1=math.radians(180)
	angle2=math.radians(230)
	angle3=math.radians(70)
	move_motor(3, angle3)
	time.sleep(2)

def gripper_close():
	str="Gripper close to initial"
	rospy.loginfo(str)
	angle1=math.radians(180)
	angle2=math.radians(230)
	angle3=math.radians(180)
	move_motor(1, angle1+45)
	move_motor(2, angle2-100)
	time.sleep(1)
	move_motor(1, angle1)
	move_motor(2, angle2)
	time.sleep(2)
	move_motor(3, angle3)
	time.sleep(2)

def gripper_open():
	str="Gripper open to ready for gripping"        
	rospy.loginfo(str)
	rospy.loginfo(str)
	angle1=math.radians(270)
	angle2=math.radians(10)
	angle3=math.radians(70)
	move_motor(3, angle3)
	time.sleep(1)
	move_motor(1, angle1-45)
	move_motor(2, angle2+100)
	time.sleep(1)
	move_motor(1, angle1)
	move_motor(2, angle2)
	time.sleep(2)

def gripper_standby():
	str="Gripper standby for front"
	rospy.loginfo(str)
	angle1=math.radians(180)
	angle2=math.radians(230)
	angle3=math.radians(180)
	move_motor(3, angle3)
	time.sleep(2)
