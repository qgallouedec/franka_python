from franka_gym.interfaces import GripperInterface, ArmInterface, RealSenseInterface

class Robot:
    def __init__(self):
        self.arm_interface = ArmInterface()
        self.gripper_interface = GripperInterface()
        # self.realsense_interface = RealSenseInterface()

    def move_to_neutral(self):
        self.arm_interface.move_to_neutral()
        self.gripper_interface.homing()
    
    def move(self, position):
        self.arm_interface.displace_ee(position[:3])
        self.gripper_interface.move(position[3])
        

    def __del__(self):
        del(self.arm_interface)

if __name__=='__main__':
    import rospy
    rospy.init_node('RobotNode', anonymous=True, disable_signals=True)
    robot = Robot()
    robot.move_to_neutral()
    import time
    time.sleep(6)
    robot.move((0.01, 0.01, 0.01, 0.002))
    

    