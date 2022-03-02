import actionlib
import rospy
from franka_gripper.msg import (GraspAction, GraspEpsilon, GraspGoal,
                                HomingAction, HomingGoal, MoveAction, MoveGoal,
                                StopAction, StopGoal)
from sensor_msgs.msg import JointState

from franka_python.utils import subscriber


class GripperInterface:
    """Interface class for the gripper on the Franka Panda robot."""

    def __init__(self):
        """Constructor."""
        if rospy.get_name() == "/unnamed":
            raise Exception("You must init a node before the interface. Call `rospy.init_node()`")
        self.name = "/franka_gripper"

        ns = self.name + "/"

        self._width = None
        self._joint_states_state_sub = subscriber(
            ns + "joint_states", JointState, self._joint_states_callback, tcp_nodelay=True, timeout=1
        )

        self._homing_action_client = actionlib.SimpleActionClient("{}homing".format(ns), HomingAction)
        self._grasp_action_client = actionlib.SimpleActionClient("{}grasp".format(ns), GraspAction)
        self._move_action_client = actionlib.SimpleActionClient("{}move".format(ns), MoveAction)
        self._stop_action_client = actionlib.SimpleActionClient("{}stop".format(ns), StopAction)

        self._homing_action_client.wait_for_server()
        self._grasp_action_client.wait_for_server()
        self._move_action_client.wait_for_server()
        self._stop_action_client.wait_for_server()

        self.MIN_FORCE = 20
        self.MAX_FORCE = 100

        self.MIN_WIDTH = 0.00
        self.MAX_WIDTH = 0.08

    def _joint_states_callback(self, msg: JointState) -> None:
        """Called when a message is published on `joint_states` topic.
        Update private attribute `_joint_state`.
        """
        idx = msg.name.index("panda_finger_joint1")
        self._width = 2 * msg.position[idx]

    def _clip_width(self, width: float) -> float:
        """Clip the width to in interval [MIN_WIDTH, MAX_WIDTH].

        Warning is raised if width was outside the interval.

        Args:
            width (float): Width to be clipped.

        Returns:
            float: Clipped width.
        """
        if width > self.MAX_WIDTH:
            rospy.logwarn("The width is clipped at {} (maximum permissible " "width).".format(self.MAX_WIDTH))
            width = self.MAX_WIDTH
        if width < self.MIN_WIDTH:
            rospy.logwarn("The width is clipped at {} (minimum permissible " "width).".format(self.MIN_WIDTH))
            width = self.MIN_WIDTH
        return width

    def _clip_force(self, force):
        if force > self.MAX_FORCE:
            rospy.logwarn("The force is clipped at {} (maximum permissible " "force).".format(self.MAX_FORCE))
            force = self.MAX_FORCE
        if force < self.MIN_FORCE:
            rospy.logwarn("The force is clipped at {} (minimum permissible " "force).".format(self.MIN_FORCE))
            force = self.MIN_FORCE
        return force

    def get_width(self) -> float:
        """Return the gripper width.

        Returns:
            float: The gripper width.
        """
        return self._width

    def homing(self, wait_for_result: bool = False):
        """Performs homing of the gripper.

        After changing the gripper fingers, a homing needs to be done.
        This is needed to estimate the maximum grasping width.

        Args:
            wait_for_result (bool, optionnal): if True, this method will block
                until response is recieved from server. Defaults False.

        Returns:
            bool: success
        """
        goal = HomingGoal()
        self._homing_action_client.send_goal(goal)
        if wait_for_result:
            return self._homing_action_client.wait_for_result(timeout=rospy.Duration(15.0))
        else:
            return True

    def move(self, width: float, speed: float = 0.05, wait_for_result: bool = False) -> bool:
        """Moves the gripper fingers to a specified width.

        Args:
            width (float): Intended opening width [m].
            speed (float, optionnal): Closing speed [m/s]. Default to 0.05.
            wait_for_result (bool): If True, this method will block until
                response is recieved from server.

        Returns:
            bool: Whether command was successful.
        """
        width = self._clip_width(width)
        goal = MoveGoal()
        goal.width = width
        goal.speed = speed
        self._move_action_client.send_goal(goal)
        if wait_for_result:
            return self._move_action_client.wait_for_result(timeout=rospy.Duration(15.0))
        else:
            return True

    def open(self, wait_for_result: bool = False) -> bool:
        """Open gripper to max possible width.

        Returns:
            bool: Whether command was successful.
        """
        return self.move(self.MAX_WIDTH)

    def close(self, wait_for_result: bool = False) -> bool:
        """Close gripper to min possible width.

        Args:
            wait_for_result (bool): If True, this method will block until
                response is recieved from server.

        Returns:
            bool: Whether command was successful.
        """
        return self.move(self.MIN_WIDTH)

    def stop(self) -> bool:
        """Stops a currently running gripper move or grasp.

        Args:
            wait_for_result (bool): If True, this method will block until
                response is recieved from server.

        Returns:
            bool: Whether command was successful.
        """
        goal = StopGoal()
        self._stop_action_client.send_goal(goal)
        return self._stop_action_client.wait_for_result(rospy.Duration(15.0))

    def grasp(
        self,
        width: float,
        force: float,
        speed: float = 0.05,
        epsilon_inner: float = 0.005,
        epsilon_outer: float = 0.005,
        wait_for_result: bool = False,
    ) -> bool:
        """Grasps an object.

        First, the gripper moves with speed `speed` in the direction of `width`:
            - open if param `width` if higher than current gripper width
            - close if param `width` if lower than current gripper width
        Opening/closing stops when a collision is detected (if no collision is
        detected, the action is aborted).
        Then, the force `force` is applied. The force is released as soon as the
        gripper width goes outside the interval
        $[(\text{width} - \text{epsilon_inner}) ; (\text{width} + \text{epsilon_outer})]$.

        Args:
            width (float): Size [m] of the object to grasp.
            force (float): Grasping force [N].
            speed (float, optionnal): Closing speed [m/s]. Default to 0.05.
            epsilon_inner (float, optionnal): Maximum inner tolerated deviation
                [m]. Defaults to 0.005.
            epsilon_outer (float, optionnal): Maximum outter tolerated
                deviation [m]. Defaults to 0.005.

        Returns:
            bool: Whether command was successful.
        """
        width = self._clip_width(width)
        force = self._clip_force(force)
        goal = GraspGoal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon = GraspEpsilon(inner=epsilon_inner, outer=epsilon_outer)
        self._grasp_action_client.send_goal(goal)
        if wait_for_result:
            return self._grasp_action_client.wait_for_result(timeout=rospy.Duration(15.0))
        else:
            return True


if __name__ == "__main__":
    rospy.init_node("GripperInterfaceNode", anonymous=True, disable_signals=True)
    p = GripperInterface()
    # rospy.spin()
