"""This module contains a interface with the arm of the Franka Emika Panda
robot.

Example:
    >>> import rospy
    >>> rospy.init_node('my_node')
    >>> from franka_gym.interface import ArmInterface
    >>> arm = ArmInterface()
    >>> arm.move_to_neutral() # arm move to its neutral position
    >>> arm.get_joint_positions()
    (-0.018, -0.7601, 0.0198, -2.3421, 0.0298, 1.5412, 0.7534)
"""
import os
import http.client
import subprocess
from math import pi
import json
import copy
import time
from typing import List, Tuple

import rospy
from sensor_msgs.msg import JointState

from franka_msgs.msg import ErrorRecoveryActionGoal

from franka_gym.utils import subscriber, publisher
from franka_gym.common import *


class ArmInterface:
    JOINT_NAMES = [
        'panda_joint1',
        'panda_joint2',
        'panda_joint3',
        'panda_joint4',
        'panda_joint5',
        'panda_joint6',
        'panda_joint7']

    LOWER_LIMIT = [-2.8973, -1.7628,  # joints 1 and 2
                   -2.8973, -3.0718,  # joints 3 and 4
                   -2.8973, -0.0175,  # joints 5 and 6
                   -2.8973]           # joint 7

    UPPER_LIMIT = [2.8973, 1.7628,   # joints 1 and 2
                   2.8973, -0.0698,  # joints 3 and 4
                   2.8973, 3.7525,   # joints 5 and 6
                   2.8973]           # joint 7

    NEUTRAL_POSE = [-0.018, -0.7601,  # joints 1 and 2
                    0.0198, -2.3421,  # joints 3 and 4
                    0.0298, 1.5412,   # joints 5 and 6
                    0.7534]           # joint 7

    def __init__(self):
        if rospy.get_name() == '/unnamed':
            raise Exception(
                'You must init a node before the interface. Call `rospy.init_node()`')
        self._run_helper()

        self._joint_positions = list(range(7))
        self._joint_velocities = list(range(7))
        self._joint_efforts = list(range(7))

        self._joint_states_sub = subscriber(
            '/joint_states', JointState, self._joint_states_callback,
            tcp_nodelay=True, timeout=1)

        self._error_recovery_pub = publisher(
            '/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, timeout=1)

        

    def _run_helper(self):
        """Run helper."""
        path = os.path.dirname(os.path.abspath(__file__))
        self._helper_proc = subprocess.Popen(path+'/arm_interface_helper.py')
        self._connection = http.client.HTTPConnection('localhost', 8080)
        time.sleep(5)  # Time to connection to be enable (dirty)
        while True:
            try:
                self._send_request_helper({'type': 'ready'})
            except (ConnectionRefusedError, http.client.CannotSendRequest):
                continue
            break

    def _joint_states_callback(self, msg: JointState) -> None:
        """Called when a message is published on `joint_states` topic.
        Update private attribute `_joint_state`.
        """
        for msg_idx, joint_name in enumerate(msg.name):
            if joint_name in self.JOINT_NAMES:
                idx = self.JOINT_NAMES.index(joint_name)
                self._joint_positions[idx] = msg.position[msg_idx]
                self._joint_velocities[idx] = msg.velocity[msg_idx]
                self._joint_efforts[idx] = msg.effort[msg_idx]

    def _send_call_helper(self, func_name: str, wait_for_result: bool = True, *args, **kwargs):
        req = {'type': 'call',
               'func_name': func_name,
               'wait_for_result': wait_for_result,
               'args': args,
               'kwargs': kwargs}
        ans = self._send_request_helper(req)
        return ans['out']

    def _send_request_helper(self, req: dict) -> str:
        """Send request to arm_interface helper."""
        self._connection.request('POST', '/process', json.dumps(req))
        ans = self._connection.getresponse().read()
        print(ans)
        return json.loads(ans)

    def get_joint_position(self, joint_name: str) -> float:
        """Return the last measured position of the joint.

        Args:
            joint_name (str): The joint name.

        Returns:
            float: The position of the joint.
        """
        idx = self.JOINT_NAMES.index(joint_name)
        return self._joint_positions[idx]

    def get_joint_positions(self) -> JointPosition:
        """Return the last measured positions of every joints.

        Returns:
            JointPosition: The position of the joints.
        """
        return copy.deepcopy(self._joint_positions)

    def get_joint_velocity(self, joint_name: str) -> float:
        """Return the last measured joint position of the joint.

        Args:
            joint_name (str): The joint name.

        Returns:
            float: The position of the joint.
        """
        idx = self.JOINT_NAMES.index(joint_name)
        return self._joint_velocities[idx]

    def get_joint_velocities(self) -> JointVelocity:
        """Return the last measured velocitiy of every joints.

        Returns:
            JointVelocity: The velocity of the joints.
        """
        return copy.deepcopy(self._joint_velocities)

    def get_joint_effort(self, joint_name: str) -> float:
        """Return the last measured joint effort of the joint.

        Args:
            joint_name (str): The joint name.

        Returns:
            float: The effort of the joint.
        """
        idx = self.JOINT_NAMES.index(joint_name)
        return self._joint_efforts[idx]

    def get_joint_efforts(self) -> JointEffort:
        """Return the last measured effort of every joints.

        Returns:
            JointEffort: The effort of the joints.
        """
        return copy.deepcopy(self._joint_efforts)

    def get_current_pose(self) -> Pose:
        """Return the cartesian position and orientation of the end-effector.

        Returns:
            Pose: Cartesian position and orientation of the end-effector.
        """
        pose = self._send_call_helper('get_current_pose', True)
        return pose

    def move_joints(self, positions: JointPosition) -> None:
        """Move joints toward the given position.

        Args:
            positions (JointPosition): The desired position of the joints.
        """
        self._send_call_helper('move_joints', False,  positions)

    def move_joint(self, joint_name: str, position: float) -> None:
        """Move joint toward the given position.

        Args:
            joint_name (str): Joint name.
            position (float): Joint position.
        """
        positions = copy.deepcopy(self._joint_positions)
        idx = self.JOINT_NAMES.index(joint_name)
        positions[idx] = position
        self.move_joints(positions)

    def move_ee(self, position: CartesianPosition, orientation: Orientation) -> None:
        """Move end-effector toward the given pose.

        Args:
            position (CartesianPosition): End-effector taget position
                as (x, y, z).
            orientation (Orientation): End-effector taget orientation as
                quaternion (x, y, z, w).
        """
        pose = (*position, *orientation)
        self._send_call_helper('move_ee', False, pose)

    def displace_ee(self, displacement: CartesianPosition) -> None:
        """Move of the end-effector with respect to its current position.

        Args:
            displacement (CartesianPosition): End-effector relative move.
                `(0, 0, 0)` means no movement.
        """
        position, orientation = self.get_current_pose()
        position = [p+d for (p, d) in zip(position, displacement)]
        return self.move_ee(position, orientation)

    def move_to_neutral(self) -> None:
        """Move the robot to go to its neutral position."""
        self.move_joints(copy.deepcopy(self.NEUTRAL_POSE))

    def error_recovery(self):
        """Recover from an error."""
        return self._error_recovery_pub.publish(ErrorRecoveryActionGoal())

    def __del__(self):
        self._connection.close()
        self._helper_proc.terminate()


if __name__ == '__main__':
    rospy.init_node('ArmInterfaceNode', anonymous=True,
                    disable_signals=True)
    import time
    arm = ArmInterface()
    print('Moving to neutral')
    arm.move_to_neutral()
    time.sleep(3)

    print('Moving joint 1')
    arm.move_joint('panda_joint1', 0.2)
    time.sleep(3)

    print('Moving all joints')
    goal = (-0, -pi/4, 0, -pi/2, 0.2, pi/3, 0)
    arm.move_joints(goal)
    time.sleep(3)

    print('moving end effector')
    arm.move_ee((0.3, 0.4, 0.19), (-1.0, -0.0, -0.0, 0.0))

    print(arm.get_joint_position('panda_joint4'))
    print(arm.get_joint_positions())
    print(arm.get_joint_velocity('panda_joint6'))
    print(arm.get_joint_velocities())
    print(arm.get_joint_effort('panda_joint6'))
    print(arm.get_joint_efforts())
    print(arm.get_current_pose())
