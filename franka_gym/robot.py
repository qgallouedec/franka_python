from franka_gym.interfaces import GripperInterface, ArmInterface

class Robot:
    def __init__(self):
        self.arm_interface = ArmInterface()
        self.gripper_interface = GripperInterface()

    def move_to_neutral(self):
        self.arm_interface.move_to_neutral()
        self.gripper_interface.open()

if __name__=='__main__':
    robot = Robot()
    robot.move_to_neutral()

    