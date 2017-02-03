#!/usr/bin/env python

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
            rospy.loginfo(id)
            rospy.loginfo(tetha)
        elif(id == 2):
            rospy.loginfo(id)
            rospy.loginfo(tetha)
        elif(id==3):
            rospy.loginfo(id)
            rospy.loginfo(tetha)
        else:
            str ="Wrong Input ID"
            rospy.loginfo(str)
    

    def read_angle(self,id):
        #angle is in radians
        if(id == 1):
            rospy.loginfo(id)
            str="angle 1"
            rospy.loginfo(str)
            angle=id
        elif(id == 2):
            rospy.loginfo(id)
            str="angle 2"
            rospy.loginfo(str)
            angle=id
        elif(id==3):
            rospy.loginfo(id)
            str="angle 3"
            rospy.loginfo(str)
            angle=id
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
            rospy.loginfo(id)
            str="load 1"
            rospy.loginfo(str)
            load=id
        elif(id == 2):
            rospy.loginfo(id)
            str="load 2"
            rospy.loginfo(str)
            load=id
        elif(id==3):
            rospy.loginfo(id)
            str="load 3"
            rospy.loginfo(str)
            load=id
        else:
            str ="Wrong Input ID"
            rospy.loginfo(str)
        return load


    #ALL ANGLE WILL BE IN THE RANGE OF -180 to 180 (need to be substracted by 180)
    def gripper_ready(self):
        str="Gripper ready"
        rospy.loginfo(str)
        time.sleep(4)

    def gripper_close(self):
        str="Gripper close"
        rospy.loginfo(str)
        time.sleep(3)

    def gripper_open(self):
        str="Gripper open"
        rospy.loginfo(str)
        time.sleep(3)

    def gripper_standby(self):
        str="Gripper standby"
        rospy.loginfo(str)
        time.sleep(4)

if __name__ == '__main__':
    try:
        obj 	= Gripper()

    	while(not rospy.is_shutdown()):
    	    obj.read_load()
        	obj.read_angle()
    except rospy.ROSInterruptException:
        pass

