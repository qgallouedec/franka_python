# -*- coding: utf-8 -*-
"""Displace the end-effector randomly, 10 times."""

import franka_gym
import rospy
rospy.init_node('my_node')
arm = franka_gym.ArmInterface()
arm.move_to_neutral()
