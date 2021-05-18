#!/usr/bin/env python3

import gzip
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

from crazyflie_seyedflocking.srv import DroneStatus, DroneStatusResponse

class CFAgent(object):
	"""docstring for CFAgent"""
	def __init__(self, i, cf_id, traj):

		self.idx = i
		self.cf_id = cf_id
		self.traj = traj

		rospy.loginfo('creating crazyflie agent ' + str(self) + '......................')

		# publisher
		self.cmdX_pub = rospy.Publisher('/'+self.cf_id+'/goal', PoseStamped, queue_size=1)

		rospy.Service('/crazyflie2_'+str(self)+'/set_status', DroneStatus, self.status_srv_callback)


		self.set_status = self.get_status_service_client()

		self.takeoff_client = self.get_controller_service_client(self.cf_id, '/cftakeoff')
		self.land_client = self.get_controller_service_client(self.cf_id, '/cfland')
		self.towpt_client = self.get_controller_service_client(self.cf_id, '/cfauto')

	def set_controller_status(self, status):
		self.controller_status_pub.publish(status)

	def to_waypoint(self, k, landing=False):
		# move to the k th point in self.traj
		cmdX_msg = PoseStamped()
		cmdX_msg.header.frame_id = '/world'
		cmdX_msg.header.stamp = rospy.Time.now()
		cmdX_msg.pose.position.x = self.traj[k][0]
		cmdX_msg.pose.position.y = self.traj[k][1]
		cmdX_msg.pose.position.z = (1-int(landing))*self.traj[k][2]

		self.cmdX_pub.publish(cmdX_msg)

	# ============ service callbacks ============
	def status_srv_callback(self, req):
		self.status = req.status
		print("received service call")
		if req.status == 'takeoff':
			self.takeoff_client()
		if req.status == 'land':
			self.land_client()
		if req.status == 'towpt':
			self.towpt_client()
		return DroneStatusResponse(self.status)

	# ============ service clients ============
	def get_controller_service_client(self, cf, name): 
		# server for this service is defined in the controller
		srv_name = '/' + cf + name
		rospy.wait_for_service(srv_name)
		rospy.loginfo(str(self) +' found ' + srv_name + ' service')
		return rospy.ServiceProxy(srv_name, Empty)			

	def get_status_service_client(self): # server for this service is defined in cfagent.py
		srv_name = '/crazyflie2_'+str(self)+'/set_status'
		rospy.loginfo('waiting for ' + srv_name + ' Service ...........')
		rospy.wait_for_service(srv_name)
		rospy.loginfo('found ' + srv_name + ' Service')
		return rospy.ServiceProxy(srv_name, DroneStatus)

	def __repr__(self):
		return 'agent'+str(self.idx)