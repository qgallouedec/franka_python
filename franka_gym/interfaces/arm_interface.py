import http.client
import subprocess
import os
from math import pi
import json
import copy
import time
import rospy
from franka_gym.utils import subscriber

from sensor_msgs.msg import JointState


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
        rospy.init_node('ArmInterfaceNode', anonymous=True,
                        disable_signals=True)

        self._run_helper()

        self._joint_states_state_sub = subscriber(
            '/joint_states', JointState, self._joint_states_callback, tcp_nodelay=True, timeout=1)

        self._joint_positions = list(range(7))
        self._joint_velocities = list(range(7))
        self._joint_efforts = list(range(7))

    def _run_helper(self):
        """Run helper."""
        path = os.path.dirname(os.path.abspath(__file__))
        self._helper_proc = subprocess.Popen(path+'/arm_interface_helper.py')
        self._connection = http.client.HTTPConnection('localhost', 8080)
        time.sleep(5)  # Time to connection to be enable (dirty)
        while True:
            try:
                self._send_request({'type': 'ready'})
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

    def _send_request(self, req):
        """Send request to arm_interface helper."""
        self._connection.request('POST', '/process', json.dumps(req))
        return self._connection.getresponse().read()

    def get_joint_position(self, joint_name: str) -> float:
        """Return the last measured position of the joint.

        Args:
            joint_name (str): The joint name.

        Returns:
            float: The position of the joint.
        """
        idx = self.JOINT_NAMES.index(joint_name)
        return self._joint_positions[idx]

    def get_joint_positions(self) -> list:
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
        idx = self.JOINT_NAMES.index(joint_name)
        return self._joint_velocities[idx]

    def get_joint_velocities(self) -> list:
        """Return the last measured velocitiy of every joints.

        Returns:
            list: Velocities.
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

    def get_joint_efforts(self) -> list:
        """Return the last measured effort of every joints.

        Returns:
            list: Keys are joint names et values are efforts.
        """
        return copy.deepcopy(self._joint_efforts)

    def move_joints(self, positions: tuple):
        """Move joint toward the given position.

        Args:
            positions (float): Joint positions.
        """
        req = {'type': 'move_joints',
               'goal': positions}
        self._send_request(req)

    def move_joint(self, joint_name: str, position: tuple):
        """Move joint toward the given position.

        Args:
            joint_name (str): Joint name.
            position (float): Joint position.
        """
        positions = copy.deepcopy(self._joint_positions)
        idx = self.JOINT_NAMES.index(joint_name)
        positions[idx] = position
        req = {'type': 'move_joints',
               'goal': positions}
        self._send_request(req)

    def move_ee(self, position: tuple, orientation: tuple) -> None:
        """Move joint toward the given position.

        Args:
            joint_name (str): Joint name.
            position (float): Joint position.
        """
        goal = (0.4, 0.1, 0.1, 1, 0, 0, 0)
        req = {'type': 'move_ee',
               'goal': goal}
        self._send_request(req)

    def move_to_neutral(self):
        """Move the robot to go to its neutral position."""
        self.move_joints(copy.deepcopy(self.NEUTRAL_POSE))

    def __del__(self):
        self._connection.close()
        self._helper_proc.terminate()


if __name__ == '__main__':
    import time
    arm = ArmInterface()
    goal = (-0, -pi/4, 0, -pi/2, 0.2, pi/3, 0)
    arm.move_to_neutral()
    time.sleep(2)
    arm.move_joint('panda_joint1', 0.1)
    
