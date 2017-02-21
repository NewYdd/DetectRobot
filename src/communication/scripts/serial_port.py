#!/usr/bin/env python
'''
Created on November 20, 2010

@author: Dr. Rainer Hessmer
'''
import threading
import serial
from cStringIO import StringIO
import time
import rospy

def _OnLineReceived(line):
	print(line)


class SerialDataGateway(object):
	'''
	Helper class for receiving lines from a serial port
	'''

	def __init__(self, port="/dev/ttyUSB0", baudrate=9600, lineHandler = _OnLineReceived):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		receivedLineHandler: The function to call when a line was received.
		'''
		self._Port = port
		self._Baudrate = baudrate
		self.ReceivedLineHandler = lineHandler
		self._KeepRunning = False

	def Start(self):
		try:
			self._Serial = serial.Serial(port = self._Port, baudrate = self._Baudrate, timeout = 1)
			print "open port sucess" 
			self._KeepRunning = True
			self._ReceiverThread = threading.Thread(target=self._Listen) #open a thread to listen 
			self._ReceiverThread.setDaemon(True)# set as back_thread,when main thread is over,this 														thread over
			self._ReceiverThread.start()
		except:
			print "open port failed"

	def Stop(self):
		if self._KeepRunning == True:
			rospy.loginfo("Stopping serial gateway")
			self._KeepRunning = False
			time.sleep(.1)
			self._Serial.close()
		else :
			print "NO open port"

	def _Listen(self):
		stringIO = StringIO()
		while self._KeepRunning:
			data = self._Serial.read()
			if data == '\r':
				pass
			if data == '\n':
				self.ReceivedLineHandler(stringIO.getvalue())
				stringIO.close()
				stringIO = StringIO()
			else:
				stringIO.write(data)

	def Write(self, data):
		info = "Writing to serial port: %s" %data
		rospy.loginfo(info)
		self._Serial.write(data)
		
if __name__ == '__main__':
	dataReceiver = SerialDataGateway("/dev/ttyUSB0",  115200)
	dataReceiver.Start()
	raw_input("Hit <Enter> to end.")
	dataReceiver.Stop()
