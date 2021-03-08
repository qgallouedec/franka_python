#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

"""Python2 helper code to command robot"""

import json
import sys
import copy

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from bottle import run, post, request, response


class FrankaInterface(object):
    """FrankaInterface"""

    def __init__(self):
        super(FrankaInterface, self).__init__()
        moveit_commander.roscpp_initialize([''])

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20)

        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

    def move_joints(self, goal):
        """Move joint."""
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[:7] = goal
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()  # ensures that there is no residual movement

    def go_to_pose_goal(self, goal):
        """Move cartesian."""
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = goal[0]
        pose_goal.position.y = goal[1]
        pose_goal.position.z = goal[2]
        pose_goal.orientation.x = goal[3]
        pose_goal.orientation.y = goal[4]
        pose_goal.orientation.z = goal[5]
        pose_goal.orientation.w = goal[6]
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()  # ensures that there is no residual movement
        # It is always good to clear your targets after planning with poses.
        self.move_group.clear_pose_targets()

    def display_trajectory(self, plan):
        """Display trajectory."""
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)


@post('/process')
def my_process():
    req_obj = json.loads(request.body.read())
    req_type = req_obj['type']
    if req_type == 'move_joints':
        fi.move_joints(req_obj['goal'])
    elif req_type == 'move_ee':
        fi.go_to_pose_goal(req_obj['goal'])
    elif req_type == 'ready':
        pass
    return True


if __name__ == '__main__':
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True, disable_signals=True)
    fi = FrankaInterface()
    run(host='localhost', port=8080, debug=True)
