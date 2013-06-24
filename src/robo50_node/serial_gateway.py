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


class SerialGateway(object):
	'''
	Helper class for receiving lines from a serial port
	'''

	def __init__(self, port="/dev/ttyACM0", baudrate=115200, lineHandler = _OnLineReceived):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		receivedLineHandler: The function to call when a line was received.
		'''
		self._Port = port
		self._Baudrate = baudrate
		self.ReceivedLineHandler = lineHandler
		self._KeepRunning = False

	def start(self):
		self._Serial = serial.Serial(port = self._Port, baudrate = self._Baudrate, timeout = 1)

		self._KeepRunning = True
		self._ReceiverThread = threading.Thread(target=self.listen)
		self._ReceiverThread.setDaemon(True)
		self._ReceiverThread.start()
		rospy.loginfo('Serial gateway started')

	def stop(self):
		rospy.loginfo("Stopping serial gateway")
		self._KeepRunning = False
		time.sleep(.1)
		self._Serial.close()

	def listen(self):
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

	def write(self, data):
		rospy.logdebug("Writing to serial port: %s" %data)
		self._Serial.write(data)

	if __name__ == '__main__':
		dataReceiver = SerialGateway("/dev/ttyACM0",  115200)
		dataReceiver.Start()

		raw_input("Hit <Enter> to end.")
		dataReceiver.Stop()
