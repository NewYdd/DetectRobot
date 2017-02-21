#!/usr/bin/env python
# get the data from the base via serial port and 
# send the data from the ros via serial port
import serial
import sys

try: 
	ser=serial.Serial('/dev/ttyUSB0',9600)
	print "sucess"
except:
	print "cannot open"
	
while True:
	try:
		line = ser.readline()
		print line
	except:
		print "unable to read"
		sys.exit(0)
	
