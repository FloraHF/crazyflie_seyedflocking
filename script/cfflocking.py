#!/usr/bin/env python3
# import os

import gzip
import numpy as np
import rospy

# from utils import norm

from cfagent import CFAgent
from crazyflie_seyedflocking.srv import DroneStatus, DroneStatusResponse

class CFSwarm(object):
	"""docstring for CFSwarm"""
	def __init__(self, traj_fname='simple', ncf=1):

		# cwd = os.getcwd()
		# print('!!!!!!!!!!!!', ncf)
		# print(cwd)

		datadir = '/home/flora/seyedflocking_path/'
		# cfs = [0, 1, 2, 3, 4, 5]
		cfs = ['cf0', 'cf1', 'cf2', 'cf3', 'cf4', 'cf5'][:ncf]
		# print(cfs)

		self.n_wpt, trajectory = self.read_trajectory(datadir+traj_fname+'.gz')

		# for traj in trajectory:
		# 	print(traj[0])

		self.n_current = 0
		self.status = 'in prep' # takeoff land towpt

		self.agents = []
		# print(cfs)
		# print(len(trajectory))
		for i, (cf, traj) in enumerate(zip(cfs, trajectory)):
			# print('creating !!!!!!!!!!!!', i)
			self.agents.append(CFAgent(i, cf, traj))
		# CFAgent(4, 'cf4', trajectory[4])		

		# print(i, '?????????????')
		# for agent in self.agents:
		# 	print(agent.traj[0])

		# for i in range(self.n_wpt):
		# 	x1, x2, x3 = trajectory[0][i][:2], trajectory[1][i][:2], trajectory[2][i][:2]
		# 	print(norm(x1-x2), norm(x1-x3), norm(x3-x2))

		# services to set status for all agents
		# print('!!!!!!!!!!!!!!!!!!!!!!!!!!!1')

		rospy.Service('set_status_for_all', DroneStatus, self.set_status_for_all_srv_callback)

		rospy.wait_for_service('set_status_for_all')
		rospy.loginfo('found set_status_for_all service')


	def iteration(self, event):

		if not rospy.is_shutdown():

			if self.status == 'takeoff':
				for agent in self.agents:
					agent.to_waypoint(self.n_current)

			# if self.status == 'land':
			# 	pass

			if self.status == 'towpt':
				if self.n_current < self.n_wpt:				
					for agent in self.agents:
						agent.to_waypoint(self.n_current)

					self.n_current += 1
				else:
					print('path finished!! try to land')
					self.status = 'land'
					for agent in self.agents:
						agent.set_status('land')

	def set_status_for_all_srv_callback(self, req):
		self.status = req.status
		for agent in self.agents:
			resp = agent.set_status(req.status)
			print('called set status service')
		return DroneStatusResponse(self.status)

	def read_trajectory(self, fname='simple'):

		# read data
		f = gzip.open(fname, 'rt')
		data = f.read()
		point = []
		step = []
		points = []
		temp = ''
		for x in data:
		    if x == ',':
		        point.append(float(temp))
		        if len(point) == 3:
		            step.append(point)
		            point = []
		        if len(step) == 6:
		            points.append(step)
		            step = []
		        temp = ''
		        continue
		    temp += x
		trajectory_temp = np.asarray(points)

		n_wpt, n_agent, n_dim = trajectory_temp.shape

		trajectory = []
		for i in range(6):
			trajectory.append(trajectory_temp[:, i, :])

		return n_wpt, trajectory


if __name__ == '__main__':

	rospy.init_node('seyedflocking', anonymous=True)

	traj = rospy.get_param("~traj_fname", 'simple')
	ncf = rospy.get_param("~n_cf", 1)

	# print('!!!!!!!!!!', ncf)

	swarm = CFSwarm(traj_fname=traj, ncf=ncf)

	rospy.Timer(rospy.Duration(.1), swarm.iteration)
	rospy.spin()	