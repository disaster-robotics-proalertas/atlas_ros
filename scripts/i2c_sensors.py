#!/usr/bin/env python

from math import sqrt
import rospy
import rosparam
import signal
import socket
import sys

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from atlas_ros.msg import Ph, Conductivity, DissolvedOxygen, Temperature, OxiRedoxPotential
from atlas_ros.srv import CalibrateEC, CalibrateECResponse
from atlas_ros.srv import CalibratePH, CalibratePHResponse
from atlas_ros.srv import CalibrateDO, CalibrateDOResponse
from sensor_msgs.msg import TimeReference

from I2Cbus import I2Cbus

def handle_sigint(sig, frame):
	"""
	Handles SIGINT signal (CTRL+C interruption)
	"""
	rospy.loginfo("[atlas_node] Shutting down")
	rospy.signal_shutdown('SIGINT received (CTRL+C)')
	sys.exit(0)

class DiagStatus:
	def __init__(self, name, hardware_id, **kwargs):
		self.status = DiagnosticStatus()
		self.status.level = DiagnosticStatus.OK
		self.status.name = '%s/%s' % (hardware_id, name)
		self.status.message = 'OK'
		self.status.hardware_id = hardware_id
		self.status.values = [KeyValue(key='Update Status', value='OK'),
							  KeyValue(key='Time Since Update', value='N/A')]
		self.last_update = 0

	def check_stale(self, current_time, timeout):
		elapsed = current_time.to_sec() - self.last_update.to_sec()
		if elapsed > timeout:
			self.status.level = DiagnosticStatus.STALE
			self.status.message = 'Stale'
			self.status.values = [KeyValue(key='Update Status', value='Stale'),
							  	  KeyValue(key='Time Since Update', value=str(elapsed))]

class Node:
	def __init__(self, **kwargs):
		self.bus = I2Cbus()

		# Initialize ROS node
		self.system_name = socket.gethostname()
		rospy.init_node('%s_atlas_node' % self.system_name, anonymous=True)

		# Get parameters
		self.phTopic = self.get_param('/atlas/pH/topic', '/atlas/raw/pH')
		self.orpTopic = self.get_param('/atlas/RedoxPotential/topic', '/atlas/raw/RedoxPotential')
		self.doTopic = self.get_param('/atlas/DissolvedOxygen/topic', '/atlas/raw/DissolvedOxygen')
		self.ecTopic = self.get_param('/atlas/Conductivity/topic', '/atlas/raw/Conductivity')
		self.tempTopic = self.get_param('/atlas/Temperature/topic', '/atlas/raw/Temperature')
		
		# Addresses for sensors
		self.ec_address = int(self.get_param('/atlas/Conductivity/SEPort', '1'))
		self.orp_address = int(self.get_param('/atlas/RedoxPotential/SEPort', '2'))
		self.ph_address = int(self.get_param('/atlas/pH/SEPort', '3'))
		self.do_address = int(self.get_param('/atlas/DissolvedOxygem/SEPort', '4'))
		self.temp_address = int(self.get_param('/atlas/Temperature/SEPort', '5'))

		# Publishers
		self.phPub = rospy.Publisher(self.phTopic, Ph, queue_size=10)
		self.orpPub = rospy.Publisher(self.orpTopic, OxiRedoxPotential, queue_size=10)
		self.doPub = rospy.Publisher(self.doTopic, DissolvedOxygen, queue_size=10)
		self.ecPub = rospy.Publisher(self.ecTopic, Conductivity, queue_size=10)
		self.tempPub = rospy.Publisher(self.tempTopic, Temperature, queue_size=10)
		
		# Topic info
		rospy.loginfo("[atlas_node] Publishing pH information in %s topic" % self.phTopic)
		rospy.loginfo("[atlas_node] Publishing Redox Potential information in %s topic" % self.orpTopic)
		rospy.loginfo("[atlas_node] Publishing Dissolved Oxygen information in %s topic" % self.doTopic)
		rospy.loginfo("[atlas_node] Publishing Conductivity information in %s topic" % self.ecTopic)
		rospy.loginfo("[atlas_node] Publishing Temperature information in %s topic" % self.tempTopic)

		# ROS services
		self.ec_calib_service = rospy.Service('calibrate_EC_sensor', CalibrateEC, self.calibrate_EC_sensor)
		self.ph_calib_service = rospy.Service('calibrate_PH_sensor', CalibratePH, self.calibrate_PH_sensor)
		self.do_calib_service = rospy.Service('calibrate_DO_sensor', CalibrateDO, self.calibrate_DO_sensor)

		# Diagnostics
		self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
		self.pHStatus = DiagStatus('pH', self.system_name)
		self.ecStatus = DiagStatus('Conductivity', self.system_name)
		self.orpStatus = DiagStatus('Oxi-Redox Potential', self.system_name)
		self.doStatus = DiagStatus('Dissolved Oxygen', self.system_name)
		self.tempStatus = DiagStatus('Temperature', self.system_name)

		# Message instances for sensors
		self.pH_msg = Ph()
		self.pH_msg.header.frame_id = '%s' % self.system_name
		self.ec_msg = Conductivity()
		self.ec_msg.header.frame_id = '%s' % self.system_name
		self.do_msg = DissolvedOxygen()
		self.do_msg.header.frame_id = '%s' % self.system_name
		self.temp_msg = Temperature()
		self.temp_msg.header.frame_id = '%s' % self.system_name
		self.orp_msg = OxiRedoxPotential()
		self.orp_msg.header.frame_id = '%s' % self.system_name

		# Rate (defined by parameter)
		hz = self.get_param('/atlas/rate', "10")
		self.rate = rospy.Rate(int(hz))

		# Send command to enable percent saturation in DO sensor
		self.bus.send_cmd('O,%,1', self.do_address)

		# Flag to indicate an ongoing calibration procedure
		self.calib = False

	# Custom function to get parameters with default values
	# For some reason, this version of rosparam is behaving
	# weirdly about default values
	def get_param(self, name, default=''):
		if rospy.has_param(name):
			return rospy.get_param(name)
		else:
			return default

	# ROS service to calibrate EC sensor
	def calibrate_EC_sensor(self, req):
		# Wait a bit for other readings to finish
		rospy.loginfo("[atlas_node] Conductivity sensor calibration")
		try:
			self.calib = True
			rospy.sleep(10.0)
			if req.type.lower() == 'clear':
				rospy.loginfo("[atlas_node] Clearing calibration data")
				self.bus.send_cmd('Cal,clear', self.ec_address)
				self.calib = False
				return CalibrateECResponse("[atlas_node] Calibration completed successfully.")
			elif req.type.lower() == 'dry':
				rospy.loginfo("[atlas_node] Dry calibration: Remove the cap from your sensor probe.\nPlace your sensor probe in dry air, in a downward position.\nWait until the EC readings stabilize. The procedure will complete automatically.")
				self.bus.send_cmd('Cal,dry', self.ec_address)
				# Keep running calibration until values are stable within a certain tolerance from 0
				current, mean, t1 = 0, 99999999, rospy.Time.now().to_sec()
				while sqrt((mean)**2) > 0.05:
					mean, i = 0, 0
					while i < 10:
						try:
							data = self.bus.get_data(self.ec_address)
							current = data[0]
							mean += current
							rospy.loginfo("[atlas_node] Current measurement: %f" % current)
							i += 1
							if (rospy.Time.now().to_sec() - t1) > 300:
								raise TimeoutError
						except ValueError:
							pass
					mean /= i
				self.calib = False
				return CalibrateECResponse("[atlas_node] Calibration completed successfully.")
			elif req.type.lower() == 'low':
				rospy.loginfo("[atlas_node] Low point calibration: Remove the cap from your sensor probe.\n\
							Place your sensor probe in the low point solution, in a downward position.\n\
							Shake the probe to release any air bubbles from the sensing area.\n\
							Wait until the EC readings stabilize. The procedure will complete automatically.")
			elif req.type.lower() == 'high':
				rospy.loginfo("[atlas_node] High point calibration: Remove the cap from your sensor probe.\n\
							Place your sensor probe in the high point solution, in a downward position.\n\
							Shake the probe to release any air bubbles from the sensing area.\n\
							Wait until the EC readings stabilize. The procedure will complete automatically.")

			# Keep running calibration until values are stable within a certain tolerance (0.1% of mean)
			current, mean, t1 = 0, 99999999, rospy.Time.now().to_sec()
			while sqrt((current - mean)**2) > mean*0.001:
				mean, i = 0, 0
				while i < 10:
					try:
						data = self.bus.get_data(self.ec_address)
						current = data[0]
						mean += current
						rospy.loginfo("[atlas_node] Current measurement: %f" % current)
						i += 1
						if (rospy.Time.now().to_sec() - t1) > 300:
							raise TimeoutError
					except ValueError:
						pass
				mean /= i
			
			if not req.type.lower() == 'dry':
				self.bus.send_cmd('Cal,%s,%d' % (req.type, req.point), self.ec_address)

			self.calib = False
			return CalibrateECResponse("[atlas_node] Calibration completed successfully.")
		except:
			self.calib = False
			return CalibrateECResponse("[atlas_node] Calibration failed!")
	
	# ROS service to calibrate pH sensor
	def calibrate_PH_sensor(self, req):
		rospy.loginfo("[atlas_node] pH sensor calibration")
		try:
			self.calib = True
			rospy.sleep(10.0)
			if req.type.lower() == 'clear':
				rospy.loginfo("[atlas_node] Clearing calibration data")
				self.bus.send_cmd('Cal,clear', self.ph_address)
				self.calib = False
				return CalibratePHResponse("[atlas_node] Calibration completed successfully.")
			if req.type.lower() == 'mid':
				rospy.loginfo("[atlas_node] Mid point calibration: Remove the storage solution from the sensor probe and rinse the tip with clean water.\n\
							Place your sensor probe in the pH 7 solution, in a downward position.\n\
							Wait until the pH readings stabilize. The procedure will complete automatically.\n\
							!!!IMPORTANT!!!: This should always be the first step in the calibration process.")
			elif req.type.lower() == 'low':
				rospy.loginfo("[atlas_node] Low point calibration: Remove the storage solution from the sensor probe and rinse the tip with clean water.\n\
							Place your sensor probe in the pH 4 solution, in a downward position.\n\
							Wait until the pH readings stabilize. The procedure will complete automatically.\n\
							!!!IMPORTANT!!!: Mid point calibration must be performed first.")
			elif req.type.lower() == 'high':
				rospy.loginfo("[atlas_node] High point calibration: Remove the storage solution from the sensor probe and rinse the tip with clean water.\n\
							Place your sensor probe in the pH 10 solution, in a downward position.\n\
							Wait until the pH readings stabilize. The procedure will complete automatically.\n\
							!!!IMPORTANT!!!: Mid point calibration must be performed first.")

			# Keep running calibration until values are stable within a certain tolerance (0.1% of mean)
			current, mean, t1 = 0, 99999999, rospy.Time.now().to_sec()
			while sqrt((current - mean)**2) > mean*0.001:
				mean, i = 0, 0
				while i < 10:
					try:
						current = self.bus.get_data(self.ph_address)[0]
						mean += current
						rospy.loginfo("[atlas_node] Current: %f" % current)
						i += 1
						if (rospy.Time.now().to_sec() - t1) > 300:
							raise TimeoutError
					except ValueError:
						pass
				mean /= i
			
			self.bus.send_cmd('Cal,%s,%d' % (req.type, req.point), self.ph_address)
			
			self.calib = False
			return CalibratePHResponse("[atlas_node] Calibration completed successfully.")
		except:
			self.calib = False
			return CalibratePHResponse("[atlas_node] Calibration failed!")
	
	# ROS service to calibrate Dissolved Oxygen sensor
	def calibrate_DO_sensor(self, req):
		rospy.loginfo("[atlas_node] Dissolved Oxygen sensor calibration")
		try:
			self.calib = True
			rospy.sleep(10.0)
			if req.type.lower() == 'clear':
				rospy.loginfo("[atlas_node] Clearing calibration data")
				self.bus.send_cmd('Cal,clear', self.do_address)
				self.bus.send_cmd("O,%,1", self.do_address)
				self.calib = False
				return CalibrateDOResponse("[atlas_node] Calibration completed successfully.")
			elif req.type.lower() == 'dry':
				rospy.loginfo("[atlas_node] Dry calibration: Remove the cap from your sensor probe.\nPlace your sensor probe in dry air, in a downward position.\nWait until the DO readings stabilize. The procedure will complete automatically.")
			elif req.type.lower() == 'zero':
				rospy.loginfo("[atlas_node] Zero-Concentration calibration: Remove the cap from your sensor probe.\n\
								Place your sensor probe in the Zero-Concentration solution, in a downward position.\n\
								Shake the sensor to remove any air bubbles from the sensing area.\n\
								Wait until the DO readings stabilize. The procedure will complete automatically.")
			

			# Keep running calibration until values are stable within a certain tolerance (0.1% of mean)
			current, mean, t1 = 0, 99999999, rospy.Time.now().to_sec()
			while sqrt((current - mean)**2) > mean*0.001:
				mean, i = 0, 0
				while i < 10:
					try:
						current = self.bus.get_data(self.do_address)[0]
						mean += current
						rospy.loginfo("[atlas_node] Current: %f" % current)
						i += 1
						if (rospy.Time.now().to_sec() - t1) > 300:
							raise TimeoutError
					except ValueError:
						pass
				mean /= i
			
			if req.type.lower() == 'zero':
				self.bus.send_cmd('Cal,0', self.do_address)
			else:
				self.bus.send_cmd('Cal', self.do_address)
			
			self.calib = False
			return CalibrateDOResponse("[atlas_node] Calibration completed successfully.")
		except:
			self.calib = False
			return CalibrateDOResponse("[atlas_node] Calibration failed!")


	def run(self):
		if not self.calib:
			# Get EC sensor data and publish it
			self.ec_msg.header.stamp = rospy.Time.now()
			data = self.bus.get_data(self.ec_address)
			self.ec_msg.ec = data[0]
			self.ec_msg.ppm = int(data[1])
			self.ec_msg.salinity = float(data[2])
			self.ec_msg.specificGrav = float(data[3])
			self.ecPub.publish(self.ec_msg)
			self.ecStatus.status.level = DiagnosticStatus.OK
			self.ecStatus.status.message = 'OK'
			self.ecStatus.status.values = [KeyValue(key='Conductivity', value=str(self.ec_msg.ec))]
			self.ecStatus.last_update = rospy.Time.now()

			# Get Redox Potential sensor data and publish it
			self.orp_msg.header.stamp = rospy.Time.now()
			data = self.bus.get_data(self.orp_address)
			self.orp_msg.orp = data[0]
			self.orpPub.publish(self.orp_msg)
			self.orpStatus.status.level = DiagnosticStatus.OK
			self.orpStatus.status.message = 'OK'
			self.orpStatus.status.values = [KeyValue(key='Oxi-Redox Potential', value=str(self.orp_msg.orp))]
			self.orpStatus.last_update = rospy.Time.now()

			# Get ph sensor data and publish it
			self.pH_msg.header.stamp = rospy.Time.now()
			data = self.bus.get_data(self.ph_address)
			self.pH_msg.pH = data[0]
			self.phPub.publish(self.pH_msg)
			self.pHStatus.status.level = DiagnosticStatus.OK
			self.pHStatus.status.message = 'OK'
			self.pHStatus.status.values = [KeyValue(key='pH', value=str(self.pH_msg.pH))]
			self.pHStatus.last_update = rospy.Time.now()

			# Get dissolved oxygen sensor data and publish it
			self.do_msg.header.stamp = rospy.Time.now()
			data = self.bus.get_data(self.do_address)
			self.do_msg.do = data[0]
			self.do_msg.saturation = data[1]
			self.doPub.publish(self.do_msg)
			self.doStatus.status.level = DiagnosticStatus.OK
			self.doStatus.status.message = 'OK'
			self.doStatus.status.values = [KeyValue(key='Dissolved Oxygen Saturation', value=str(self.do_msg.saturation))]
			self.doStatus.last_update = rospy.Time.now()

			# Get RTD sensor data and publish it
			self.temp_msg.header.stamp = rospy.Time.now()
			data = self.bus.get_data(self.temp_address)
			self.temp_msg.celsius = data[0]
			self.temp_msg.fahrenheit = self.temp_msg.celsius*1.8 + 32.0	# Conversion to Fahrenheit
			self.tempPub.publish(self.temp_msg)
			self.tempStatus.status.level = DiagnosticStatus.OK
			self.tempStatus.status.message = 'OK'
			self.tempStatus.status.values = [KeyValue(key='Temperature (deg C)', value=str(self.temp_msg.celsius))]
			self.tempStatus.last_update = rospy.Time.now()

		# Check for stale status
		self.ecStatus.check_stale(rospy.Time.now(), 35)
		self.orpStatus.check_stale(rospy.Time.now(), 35)
		self.pHStatus.check_stale(rospy.Time.now(), 35)
		self.doStatus.check_stale(rospy.Time.now(), 35)
		self.tempStatus.check_stale(rospy.Time.now(), 35)

		# Publish diagnostics message
		diag_msg = DiagnosticArray()
		diag_msg.status.append(self.ecStatus.status)
		diag_msg.status.append(self.orpStatus.status)
		diag_msg.status.append(self.pHStatus.status)
		diag_msg.status.append(self.doStatus.status)
		diag_msg.status.append(self.tempStatus.status)
		self.diag_pub.publish(diag_msg)

		# Sleep
		self.rate.sleep()

if __name__ == "__main__":
	signal.signal(signal.SIGINT, handle_sigint)
	
	node = Node()
	while not rospy.is_shutdown():
		node.run()