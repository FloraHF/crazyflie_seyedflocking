#!/usr/bin/env python3

import numpy as np

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist


# ===== geometry calculations
def norm(x):
	return np.sqrt(sum([xx**2 for xx in x]))
def dist(x, y):
	return norm(x-y)

# ===== message conversions
def create_multi_dof_joint_trajectory_msg(npts):
	trajectory_msg = MultiDOFJointTrajectory()

	trajectory_point = MultiDOFJointTrajectoryPoint()
	trajectory_point.transforms.append(Transform())
	trajectory_point.velocities.append(Twist())
	trajectory_point.accelerations.append(Twist())

	for i in range(npts):
		trajectory_msg.points.append(trajectory_point)
		trajectory_msg.joint_names.append('')

	return trajectory_msg