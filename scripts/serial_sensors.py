#!/usr/bin/env python

from math import sqrt
import rospy
import rosparam
import signal
import sys

from atlas_ros.msg import Ph, Conductivity, DissolvedOxygen, Temperature, OxiRedoxPotential
from atlas_ros.srv import CalibrateEC, CalibrateECResponse
from atlas_ros.srv import CalibratePH, CalibratePHResponse
from atlas_ros.srv import CalibrateDO, CalibrateDOResponse
from sensor_msgs.msg import TimeReference

from SerialExpander import SerialExpander

class Node:
	def __init__(self, **kwargs):
		self.expander = SerialExpander()

		# Initialize ROS node
		rospy.init_node('atlas_node', anonymous=True)

		# Get parameters
		self.phTopic = self.get_param('/atlas/pH/topic', '/atlas/raw/pH')
		self.orpTopic = self.get_param('/atlas/RedoxPotential/topic', '/atlas/raw/RedoxPotential')
		self.doTopic = self.get_param('/atlas/DissolvedOxygen/topic', '/atlas/raw/DissolvedOxygen')
		self.ecTopic = self.get_param('/atlas/Conductivity/topic', '/atlas/raw/Conductivity')
		self.tempTopic = self.get_param('/atlas/Temperature/topic', '/atlas/raw/Temperature')
		
		# Addresses for sensors
		self.ec_address = self.get_param('/atlas/Conductivity/SEPort', 'P1')
		self.orp_address = self.get_param('/atlas/RedoxPotential/SEPort', 'P2')
		self.ph_address = self.get_param('/atlas/pH/SEPort', 'P3')
		self.do_address = self.get_param('/atlas/DissolvedOxygem/SEPort', 'P4')
		self.temp_address = self.get_param('/atlas/Temperature/SEPort', 'P5')

		# Publishers
		self.phPub = rospy.Publisher(self.phTopic, Ph, queue_size=10)
		self.orpPub = rospy.Publisher(self.orpTopic, OxiRedoxPotential, queue_size=10)
		self.doPub = rospy.Publisher(self.doTopic, DissolvedOxygen, queue_size=10)
		self.ecPub = rospy.Publisher(self.ecTopic, Conductivity, queue_size=10)
		self.tempPub = rospy.Publisher(self.tempTopic, Temperature, queue_size=10)
		
		# Connect to serial expander
		rospy.loginfo("[atlas_node] Connecting to Serial Expander...")
		expander.connect()
		rospy.loginfo("[atlas_node] Success")

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

		# Message instances for sensors
		self.pH_msg = Ph()
		self.pH_msg.header.frame_id = 'atlaspi'
		self.ec_msg = Conductivity()
		self.ec_msg.header.frame_id = 'atlaspi'
		self.do_msg = DissolvedOxygen()
		self.do_msg.header.frame_id = 'atlaspi'
		self.temp_msg = Temperature()
		self.temp_msg.header.frame_id = 'atlaspi'
		self.orp_msg = OxiRedoxPotential()
		self.orp_msg.header.frame_id = 'atlaspi'

		# Send command to enable percent saturation in DO sensor
		self.expander.send_cmd('O,%,1', self.do_address)

		# Rate (defined by parameter)
		hz = self.get_param('/atlas/rate', "10")
		self.rate = rospy.Rate(int(hz))

	def handle_sigint(self, sig, frame):
		"""
		Handles SIGINT signal (CTRL+C interruption)
		"""
		rospy.loginfo("[atlas_node] Shutting down")
		self.expander.disconnect()
		rospy.signal_shutdown('SIGINT received (CTRL+C)')
		sys.exit(0)

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
		rospy.loginfo("[atlas_node] Conductivity sensor calibration")
		try:
			self.expander.select_SE_port(self.ec_address)
			if req.type.lower() == 'dry':
				rospy.loginfo("[atlas_node] Dry calibration: Remove the cap from your sensor probe.\n\
							Place your sensor probe in dry air, in a downward position.\n\
							Wait until the EC readings stabilize. The procedure will complete automatically.")
				self.expander.send_cmd('Cal,dry')
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

			# Keep running calibration until values are stable within a certain tolerance (0.05)
			current, mean, t1 = 0, 99999999, rospy.Time.now().to_sec()
			while sqrt((current - mean)**2) > 0.05:
				mean = 0
				for _ in range(10):
					data = self.expander.get_data(self.ec_address).strip('\r').split(',')
					current = float(data[0])
					mean += current
					rospy.loginfo("[atlas_node] Current: %f" % current)
				mean /= 10
				if (rospy.Time.now().to_sec() - t1) > 300:
					raise TimeoutError
			
			if not req.type.lower() == 'dry':
				self.expander.send_cmd('Cal,%s,%d' % (req.type, req.point))

			return CalibrateECResponse("[atlas_node] Calibration completed successfully.")
		except:
			return CalibrateECResponse("[atlas_node] Calibration failed!")
	
	# ROS service to calibrate pH sensor
	def calibrate_PH_sensor(self, req):
		rospy.loginfo("[atlas_node] pH sensor calibration")
		try:
			self.expander.select_SE_port(self.ph_address)
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

			# Keep running calibration until values are stable within a certain tolerance (0.05)
			current, mean, t1 = 0, 99999999, rospy.Time.now().to_sec()
			while sqrt((current - mean)**2) > 0.05:
				mean = 0
				for _ in range(10):
					current = float(self.expander.get_data(self.ph_address).strip('\r'))
					mean += current
					rospy.loginfo("[atlas_node] Current: %f" % current)
				mean /= 10
				if (rospy.Time.now().to_sec() - t1) > 300:
					raise TimeoutError
			
			self.expander.send_cmd('Cal,%s,%d' % (req.type, req.point))

			return CalibratePHResponse("[atlas_node] Calibration completed successfully.")
		except:
			return CalibratePHResponse("[atlas_node] Calibration failed!")
	
	# ROS service to calibrate Dissolved Oxygen sensor
	def calibrate_DO_sensor(self, req):
		rospy.loginfo("[atlas_node] Dissolved Oxygen sensor calibration")
		try:
			self.expander.select_SE_port(self.do_address)
			if req.point == 0:
				rospy.loginfo("[atlas_node] Zero-Concentration calibration: Remove the cap from your sensor probe.\n\
								Place your sensor probe in the Zero-Concentration solution, in a downward position.\n\
								Shake the sensor to remove any air bubbles from the sensing area.\n\
								Wait until the DO readings stabilize. The procedure will complete automatically.")
			else:
				rospy.loginfo("[atlas_node] Dry calibration: Remove the cap from your sensor probe.\n\
								Place your sensor probe in dry air, in a downward position.\n\
								Wait until the DO readings stabilize. The procedure will complete automatically.")
			

			# Keep running calibration until values are stable within a certain tolerance (0.05)
			current, mean, t1 = 0, 99999999, rospy.Time.now().to_sec()
			while sqrt((current - mean)**2) > 0.05:
				mean = 0
				for _ in range(10):
					current = float(self.expander.get_data(self.do_address).strip('\r'))
					mean += current
					rospy.loginfo("[atlas_node] Current: %f" % current)
				mean /= 10
				if (rospy.Time.now().to_sec() - t1) > 300:
					raise TimeoutError
			
			if req.point == 0:
				self.expander.send_cmd('Cal')
			else:
				self.expander.send_cmd('Cal,0')

			return CalibrateDOResponse("[atlas_node] Calibration completed successfully.")
		except:
			return CalibrateDOResponse("[atlas_node] Calibration failed!")


	def run(self):
		# Get EC sensor data and publish it
		self.ec_msg.header.stamp = rospy.Time.now()
		data = self.expander.get_data(self.ec_address).strip('\r').split(',')
		self.ec_msg.ec = float(data[0])
		self.ec_msg.ppm = int(data[1])
		self.ec_msg.salinity = float(data[2])
		self.ec_msg.specificGrav = float(data[3])
		self.ecPub.publish(self.ec_msg)

		# Get Redox Potential sensor data and publish it
		self.orp_msg.header.stamp = rospy.Time.now()
		self.orp_msg.orp = float(self.expander.get_data(self.orp_address).strip('\r'))
		self.orpPub.publish(self.orp_msg)

		# Get ph sensor data and publish it
		self.pH_msg.header.stamp = rospy.Time.now()
		self.pH_msg.pH = float(self.expander.get_data(self.ph_address).strip('\r'))
		self.phPub.publish(self.pH_msg)

		# Get dissolved oxygen sensor data and publish it
		self.do_msg.header.stamp = rospy.Time.now()
		self.do_msg.do = float(self.expander.get_data(self.do_address).strip('\r'))
		self.doPub.publish(self.do_msg)

		# Get RTD sensor data and publish it
		self.temp_msg.header.stamp = rospy.Time.now()
		self.temp_msg.celsius = float(self.expander.get_data(self.temp_address).strip('\r'))
		self.temp_msg.fahrenheit = self.temp_msg.celsius*1.8 + 32.0	# Conversion to Fahrenheit
		self.tempPub.publish(self.temp_msg)

		# Sleep
		self.rate.sleep()

if __name__ == "__main__":
	node = Node()
	signal.signal(signal.SIGINT, node.handle_sigint)
	
	while not rospy.is_shutdown():
		node.run()
