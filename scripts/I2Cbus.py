#!/usr/bin/python

import smbus
import serial
import sys
import time

class I2Cbus:
	def __init__(self, timeout=1, **kwargs):
		self.__timeout = timeout
		self.bus = smbus.SMBus(1)

	def read_line(self, address):
		try:
			response = self.bus.read_i2c_block_data(address, 0x00)
		except IOError as e:
			print "[Quality] Error %s occurred while reading on address %d" % (e.strerror, address)
			return None
		response = [i for i in response if not i == '\00']
		char_list = list(map(lambda x: chr(x & ~0x80), list(response[1:])))
		char_list = ''.join(char_list).strip('\x00').split(',')
		return [float(x) for x in char_list]

	def convert_string_to_bytes(self, cmd):
        	converted = []
        	for b in cmd:
        		converted.append(ord(b))
	        return converted

	def send_cmd(self, cmd, address):
		start = ord(cmd[0])
		end = cmd[1:] + "\00"
		end = self.convert_string_to_bytes(end)
		try:
			self.bus.write_i2c_block_data(address, start, end)
			time.sleep(self.__timeout)			# Wait 1s for response to arrive
			return True
		except IOError as e:
			print "[Quality] Error %s occurred while writing on address %d" % (e.strerror, address)
			return None

	def get_data(self, address):
		"""
		Gets a single reading from sensor in selected port
		"""
		# Send request for data
		self.send_cmd("R", address)
		line = self.read_line(address)
		
		return line
