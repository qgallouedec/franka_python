
# import enum
import rospy
# import warnings
# import quaternion
# import numpy as np
# from copy import deepcopy
# from rospy_message_converter import message_converter

from franka_msgs.msg import JointCommand
# from franka_core_msgs.msg import RobotState, EndPointState
from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64

# import franka_dataflow
# from robot_params import RobotParams

from franka_msgs import JointState

import copy

from typing import Optional, Type, Callable, List
import genpy

Dataclass = Type[genpy.Message]
Message = genpy.Message


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
        self.command_publisher = publisher(
            '/panda_simulator/motion_controller/arm/joint_commands',
            JointCommand, timeout=10)

        self._joint_states_state_sub = subscriber(
            '/panda_simulator/custom_franka_state_controller/joint_states',
            JointState, self._joint_states_callback, tcp_nodelay=True)

        self._joint_positions = {}
        self._joint_velocities = {}
        self._joint_efforts = {}

    def _joint_states_callback(self, msg):
        for idx, joint_name in enumerate(msg.name):
            if joint_name in self.JOINT_NAMES:
                self._joint_positions[joint_name] = msg.position[idx]
                self._joint_velocities[joint_name] = msg.velocity[idx]
                self._joint_efforts[joint_name] = msg.effort[idx]

    def get_joint_position(self, joint_name: str) -> float:
        """Return the last measured position of the joint.

        Args:
            joint_name (str): The joint name.

        Returns:
            float: The position of the joint.
        """
        return self._joint_positions[joint_name]

    def get_joint_positions(self) -> dict:
        """Return the last measured positions of every joints.

        Returns:
            dict: Keys are joint names et values are positions.
        """
        return copy.deepcopy(self._joint_positions)

    def get_joint_velocity(self, joint_name: str) -> float:
        """Return the last measured joint position of the joint.

        Args:
            joint_name (str): The joint name.

        Returns:
            float: The position of the joint.
        """
        return self._joint_velocities[joint_name]

    def get_joint_velocities(self) -> dict:
        """Return the last measured velocitiy of every joints.

        Returns:
            dict: Keys are joint names et values are velocities.
        """
        return copy.deepcopy(self._joint_velocities)

    def get_joint_effort(self, joint_name: str) -> float:
        """Return the last measured joint effort of the joint.

        Args:
            joint_name (str): The joint name.

        Returns:
            float: The effort of the joint.
        """
        return self._joint_efforts[joint_name]

    def get_joint_efforts(self) -> dict:
        """Return the last measured effort of every joints.

        Returns:
            dict: Keys are joint names et values are efforts.
        """
        return copy.deepcopy(self._joint_efforts)

    def command(self, mode: int, names: List[str], positions: List[float]):
        """Control the robot.

        Args:
            mode (int): Control mode.
            names (List[str]): Joint names.
            positions (List[float]): Positions.
        """
        msg = JointCommand()
        msg.mode = mode
        msg.names = names
        msg.position = positions
        self.command_publisher.publish(msg)

    def move_joints(self, positions: tuple):
        """Move joints toward the given positions.

        Args:
            command (tuple): Joint positions, in the same order as 
                `self.JOINT_NAMES`
        """
        self.command(
            mode=1, names=copy.deepcopy(self.JOINT_NAMES), positions=positions)

    def move_joint(self, joint_name, position):
        """Move joint toward the given position.

        Args:
            position (float): Joint position.
        """
        self.command(
            mode=1, names=[joint_name], positions=[position])

    def move_to_neutral(self):
        """Move the robot to go to its neutral position."""
        self.move_joints(copy.deepcopy(self.NEUTRAL_POSE))


if __name__ == '__main__':
    import time
    rospy.init_node('test_fri', anonymous=True)
    r = ArmInterface()
    r.move_to_neutral()
    time.sleep(1)

    # rate = rospymove_to_neutral.Rate(10)
    # while not rospy.is_shutdown():
    #     rate.sleep()
