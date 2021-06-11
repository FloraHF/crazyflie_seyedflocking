#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32


class CFFlockingRecorder(object):
	""" This is just a ros node that subscribes to 
		topics needing recording, to use rosbag functionality that
		record topics subscribed by a ros node."""

	def __init__(self, cfs='cf0 cf1 cf2 cf3 cf4 cf5'):

		self.cfs = cfs.split(' ')

		self.subs = {cf:[] for cf in self.cfs}
		for cf in self.cfs:
			self.generate_subscriber(cf)

		print(self.subs)

	def generate_subscriber(self, cf):
		imu_topic = '/' + cf + '/imu'
		pose_topic = '/vrpn_client_node/' + cf + '/pose'
		pres_topic = '/' + cf + '/pressure'
		temp_topic = '/' + cf + '/temperature'
		cmdVtemp_topic = '/' + cf + '/cmdVtemp'

		self.subs[cf].append(rospy.Subscriber(imu_topic, Imu, self.generate_null_callback()))
		self.subs[cf].append(rospy.Subscriber(pose_topic, PoseStamped, self.generate_null_callback()))
		self.subs[cf].append(rospy.Subscriber(pres_topic, Float32, self.generate_null_callback()))
		self.subs[cf].append(rospy.Subscriber(temp_topic, Temperature, self.generate_null_callback()))
		self.subs[cf].append(rospy.Subscriber(cmdVtemp_topic, TwistStamped, self.generate_null_callback()))

	def generate_null_callback(self):
		def null_callback(msg):
			pass
		return null_callback

if __name__ == '__main__':

	rospy.init_node('seyedflocking_recorder', anonymous=True)

	cfs = rospy.get_param("~cfs", 'cf0 cf1 cf2 cf3 cf4 cf5')

	recorder = CFFlockingRecorder(cfs=cfs)

	rospy.spin()	