#!/usr/bin/python

import smbus
import serial
import sys
import time

class I2Cbus:
	def __init__(self, address=1, timeout=1, **kwargs):
		self.__address = address
		self.__timeout = timeout

		self.bus = smbus.SMBus(1)

	def change_address(self, address):
		self.__address = int(address)

	def read_line(self):
		try:
        		response = self.bus.read_i2c_block_data(self.__address, 0x00)
        	except IOError as e:
			print "[Quality] An error", e, "ocurred while reading on address", self.__address
			return None
		response = [i for i in response if i != '\00']
        	char_list = list(map(lambda x: chr(x & ~0x80), list(response[1:])))
        	return ''.join(char_list).strip("\x00")

	def convert_string_to_bytes(self, cmd):
        	converted = []
        	for b in cmd:
        		converted.append(ord(b))
	        return converted

	def send_cmd(self, cmd):
        	start = ord(cmd[0])
	        end = cmd[1:] + "\00"
        	end = self.convert_string_to_bytes(end)
        	try:
        		self.bus.write_i2c_block_data(self.__address, start, end)
			return True
        	except IOError as e:
        		print "[Quality] An error", e, "ocurred while writing on address", self.__address
			return None

	def get_data(self, address):
		"""
		Gets a single reading from sensor in selected port
		"""
		self.change_address(address)

		# Send request for data
		self.send_cmd("R")
		time.sleep(self.__timeout)			# Wait 1s for the data to arrive
		line = self.read_line()
		
		return line
