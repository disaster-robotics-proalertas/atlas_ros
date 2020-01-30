#!/usr/bin/python

import serial
import sys
import time
import string 
from serial import SerialException
import RPi.GPIO as gpio

class SerialExpander:
	def __init__(self, port='/dev/ttyS0', baud=9600, timeout=0, **kwargs):
		self.__port = port
		self.__baud = baud
		self.ser = serial.Serial(self.__port, self.__baud, timeout=timeout)
		
		# Dictionary for expander addresses as accessed through GPIO
		self.expanderAddr = {'P1': [0,0,0],
							 'P2': [0,0,1],
							 'P3': [0,1,0],
							 'P4': [0,1,1],
							 'P5': [1,0,0],
							 'P6': [1,0,1],
							 'P7': [1,1,0],
							 'P8': [1,1,1]
							}
		
		# Configuration for Lutra Airboats
		self.config()

	def config(self):
		"""
		Configuration specific for Atlas sensors in LSA's Lutra Airboats.
		Namely sets GPIO ports, disable acknowledge (ACK, i.e., OK) messages for sensors 
		and turns off continuous sensor readings
		"""
		# Setup for GPIO
		gpio.setwarnings(False)
		gpio.setmode(gpio.BCM)
		self.gpioPins = [24, 23, 18]	# [S3, S2, S1]
		for pin in self.gpioPins:
			gpio.setup(pin, gpio.OUT)

		# Turn off ACK messages and continuous reading for all devices
		for port in self.expanderAddr:
			self.ser.flush()
			self.send_cmd("*OK,0", port)	# Disable OK messages
			time.sleep(0.01)				# Wait 10 ms before next instruction
			self.ser.flush()
			self.send_cmd("C,0", port)		# Disable continuous reading mode
			time.sleep(0.01)				# Wait 10 ms before next instruction

		# Return to default port "0,0,0" (or "P1")
		self.select_SE_port("P1")

	def select_SE_port(self, port):
		"""
		Selects Serial Expander port based on address table in self.expanderAddr dictionary
		"""
		for i, pin in enumerate(self.gpioPins):
			gpio.output(pin, self.expanderAddr[port][i])

	def change_port(self, port):
		self.__port = port
		self.connect()
		self.config()

	def change_baud(self, baud):
		self.__baud = baud
		self.connect()
		self.config()

	def connect(self):
		self.disconnect()
		while True:
			if self.ser.isOpen():
				return True
			self.ser.open()

	def disconnect(self):
		while True:
			if not self.ser.isOpen():
				return True
			self.ser.close()

	def read_line(self):
		"""
		taken from the ftdi library and modified to 
		use the ezo line separator "\r"
		"""
		lsl = len('\r')
		line_buffer = []
		while True:
			next_char = self.ser.read(1)
			if next_char == '':
				break
			line_buffer.append(next_char)
			if (len(line_buffer) >= lsl and
					line_buffer[-lsl:] == list('\r')):
				break
		return ''.join(line_buffer)
	
	def read_lines(self):
		"""
		also taken from ftdi lib to work with modified readline function
		"""
		lines = []
		try:
			while True:
				line = self.read_line()
				if not line:
					break
					self.ser.flush_input()
				lines.append(line)
			return lines
		except SerialException as e:
			print( "Error, ", e)
			return None	

	def send_cmd(self, cmd, port):
		"""
		Send command to the Atlas Sensor.
		Before sending, add Carriage Return at the end of the command.
		:param port:
		:param cmd:
		:return:
		"""
		self.select_SE_port(port)
		buf = cmd + "\r"     	# add carriage return
		try:
			self.ser.write(buf.encode('utf-8'))
			time.sleep(1)			# Wait 1s for the data to arrive
			return True
		except SerialException as e:
			print ("Error, ", e)
			return None

	def get_data(self, port):
		"""
		Gets a single reading from sensor in selected port
		"""
		# Clear previous data
		self.ser.flush()

		# Send request for data
		self.send_cmd("R", port)
		lines = self.read_lines()
		
		return lines[0]