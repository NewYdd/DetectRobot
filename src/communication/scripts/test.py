#!/usr/bin/env python

import rospy
import sys
import time
import math

#This module helps to receive values from serial port
from serial_port import SerialDataGateway

#Class to handle serial data from base and converted to ROS topics
class Base_Class(object):
	
	def __init__(self):
		print "Initializing Base Class"
#######################################################################################################################
		#Sensor variables
		self._Counter = 0

		self._left_encoder_value = 0
		self._right_encoder_value = 0

	

		self._left_wheel_speed_ = 0
		self._right_wheel_speed_ = 0

		self._LastUpdate_Microsec = 0
		self._Second_Since_Last_Update = 0

		self.robot_heading = 0
#######################################################################################################################
		#Get serial port and baud rate of Tiva C Launchpad
		#port = rospy.get_param("~port", "/dev/ttyACM0")
		#baudRate = int(rospy.get_param("~baudRate", 115200))
		self._SerialDataGateway = SerialDataGateway("/dev/ttyUSB0", 9600, self._HandleReceivedLine)
		rospy.loginfo("Started serial communication")
#######################################################################################################################
#Subscribers and Publishers

		#Publisher for left and right wheel encoder values
		self.Encoder
		self._Left_Encoder = rospy.Publisher('lwheel',Int64,queue_size = 10)		
		self._Right_Encoder = rospy.Publisher('rwheel',Int64,queue_size = 10)		
		#Publisher for entire serial data
		self._SerialPublisher = rospy.Publisher('serial', String,queue_size=10)
		#Speed subscriber
		self._left_motor_speed = rospy.Subscriber('left_wheel_speed',Float32,self._Update_Left_Speed)
		self._right_motor_speed = rospy.Subscriber('right_wheel_speed',Float32,self._Update_Right_Speed)
		#######################################################################################################################################################3
	def _Update_Right_Speed(self, right_speed):

		self._right_wheel_speed_ = right_speed.data

		rospy.loginfo(right_speed.data)

		speed_message = 's %d %d\r' %(int(self._left_wheel_speed_),int(self._right_wheel_speed_))

		self._WriteSerial(speed_message)


###############################################################################################
#######################################################################################################################
#Calculate orientation from accelerometer and gyrometer
	def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))
		

if __name__ =='__main__':
	base = Base_Class()
		
		
		
		
		
