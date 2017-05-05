#!/usr/bin/env python

from lib_robotis import*
import rospy
import math
import time
from gripper_driver.msg import motor_state
from gripper_driver.msg import motor_command

"""
Comments:
1) Make a class with all the methods that you defined. Its better than working with functions from the script
2) When you import a message, you import it from the package once you have done a catkin make
    Eg: 
        if your message motor_command is stored in the package called gripper_driver under the folder msg, then the statement will be:
            from gripper_driver.msg import motor_command
3) Instead of printing out statements, use rospy logger (read up about it)

"""

class Gripper():
    
    def __init__(self):

        self.gripper = USB2Dynamixel_Device('/dev/ttyUSB0')
        self.motor_link1= Robotis_Servo(self.gripper,1,series='MX')
        self.motor_link2= Robotis_Servo(self.gripper,2,series='MX')
        self.motor_suction= Robotis_Servo(self.gripper,3,series='MX')

        self.pub = rospy.Publisher('/motor/state', motor_state)
        self.motor_statemsg = motor_state()

        rospy.init_node('gripper', anonymous=True)
        rospy.Subscriber("/motor/command", motor_command, self.Command)

    def Command(self, data):
        if(data.gripper_ready):
            self.gripper_ready()

        if(data.gripper_close):
            self.gripper_close()

        if(data.gripper_open):
            self.gripper_open()

        if(data.gripper_standby):
            self.gripper_standby()

        if(data.read_angle):
            self.motor_statemsg.angle=self.read_angle(data.read_angle)

        if(data.read_load):
            self.motor_statemsg.load=self.read_load(data.read_load)

        self.pub.publish(self.motor_statemsg)

        #I think I need to reset the motor/command topic, but I don't know how to do it

    def move_motor(self,id,tetha):
        if(id == 1):
            self.motor_link1.move_angle(tetha, blocking = False)
            rospy.loginfo("Motor {0} is moving to".format(id))
            rospy.loginfo(tetha)
        elif(id == 2):
            self.motor_link2.move_angle(tetha, blocking = False)
            rospy.loginfo("Motor {0} is moving to".format(id))
            rospy.loginfo(tetha)
        elif(id==3):
            self.motor_suction.move_angle(tetha, blocking = False)
            rospy.loginfo("Motor {0} is moving to".format(id))
            rospy.loginfo(tetha)
        else:
            str ="Wrong Input ID"
            rospy.loginfo(str)
    

    def read_angle(self,id):
        #angle is in radians
        if(id == 1):
            str="angle 1"
            rospy.loginfo(str)
            angle=math.degrees(self.motor_link1.read_angle())
        elif(id == 2):
            str="angle 2"
            rospy.loginfo(str)
            angle=math.degrees(self.motor_link2.read_angle())
        elif(id==3):
            str="angle 3"
            rospy.loginfo(str)
            angle=math.degrees(self.motor_suction.read_angle())
        else:
            str ="Wrong Input ID"
            rospy.loginfo(str)
        return angle

    def read_load(self,id):
        #the value is in range of 0-2047, mapped from 0 to 100% of maximum torque in CCW and CW
        #0-1023 CCW, 1024-2047 CW, tenth bit become load direction
        #if the value is 512, means 512/1023*100% of motor torque direction is CCW
        #maximum torque = 2.5 Nm
        if(id == 1):
            str="load 1"
            rospy.loginfo(str)
            load=self.motor_link1.read_load()
        elif(id == 2):
            str="load 2"
            rospy.loginfo(str)
            load=self.motor_link2.read_load()
        elif(id==3):
            str="load 3"
            rospy.loginfo(str)
            load=self.motor_suction.read_load()
        else:
            str ="Wrong Input ID"
            rospy.loginfo(str)
        return load

    #ID1    (180(initial) - 270(max))
    #ID2    (230(initial))
    #       (10(open)-180(close) - fully out position)
    #ID 3   (180(initial)-70(final open))
    #ALL ANGLE WILL BE IN THE RANGE OF -180 to 180 (need to be substracted by 180)
    #all value must be subtracted by 180
    def gripper_ready(self):
        str="Gripper ready for side"
        rospy.loginfo(str)
        angle1=math.radians(180-180)
        angle2=math.radians(230-180)
        angle3=math.radians(70-180)
        self.move_motor(3, angle3)
        time.sleep(2)

    def gripper_close(self):
        str="Gripper close to initial"
        rospy.loginfo(str)
        angle1=math.radians(180-180)
        angle2=math.radians(230-180)
        angle3=math.radians(180-180)
        self.move_motor(1, angle1+45)
        self.move_motor(2, angle2-100)
        time.sleep(1)
        self.move_motor(1, angle1)
        self.move_motor(2, angle2)
        time.sleep(2)
        self.move_motor(3, angle3)
        time.sleep(2)

    def gripper_open(self):
        str="Gripper open to ready for gripping"        
        rospy.loginfo(str)
        rospy.loginfo(str)
        angle1=math.radians(270-180)
        angle2=math.radians(10-180)
        angle3=math.radians(70-180)
        self.move_motor(3, angle3)
        time.sleep(1)
        self.move_motor(1, angle1-45)
        self.move_motor(2, angle2+100)
        time.sleep(1)
        self.move_motor(1, angle1)
        self.move_motor(2, angle2)
        time.sleep(2)

    def gripper_standby(self):
        str="Gripper standby for front"
        rospy.loginfo(str)
        angle1=math.radians(180-180)
        angle2=math.radians(230-180)
        angle3=math.radians(180-180)
        self.move_motor(3, angle3)
        time.sleep(2)

if __name__ == '__main__':
    try:
        Gripper()
        while(not rospy.is_shutdown()):
            pass
    except rospy.ROSInterruptException:
        pass

