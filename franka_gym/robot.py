from typing import Tuple

from franka_gym.interfaces import GripperInterface, ArmInterface, RealSenseInterface
from franka_gym.common import CartesianState


class Robot:
    """Robot interface. Containing the arm and the gripper interface."""

    def __init__(self):
        self.arm_interface = ArmInterface()
        self.gripper_interface = GripperInterface()

    def homing(self) -> None:
        """Move the robot to it's neutral position and home the gripper."""
        self.arm_interface.move_to_neutral()
        self.gripper_interface.homing(wait_for_result=True)

    def move(self, position: CartesianState) -> None:
        """Move the end-effector and the fingers to the given position.

        Args:
            position (CartesianState): Desired state as
                (ee_x, ee_y, ee_z, gripper_width).
        """
        self.arm_interface.displace_ee(position[:3])
        self.gripper_interface.move(position[3])

    def close_gripper(self, wait_for_result=False) -> None:
        pass

    def get_joint_positions(self) -> Tuple

    def __del__(self):
        del(self.arm_interface)


if __name__ == '__main__':
    import rospy
    rospy.init_node('RobotNode', anonymous=True, disable_signals=True)
    robot = Robot()
    robot.move_to_neutral()
    import time
    time.sleep(6)
    robot.move((0.01, 0.01, 0.01, 0.002))
