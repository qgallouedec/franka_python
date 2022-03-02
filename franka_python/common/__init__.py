from typing import Tuple

JointPosition = Tuple[float, float, float, float, float, float, float]  # Franka's arm has 7 joints
JointVelocity = Tuple[float, float, float, float, float, float, float]  # Franka's arm has 7 joints
JointEffort = Tuple[float, float, float, float, float, float, float]  # Franka's arm has 7 joints
CartesianPosition = Tuple[float, float, float]
Orientation = Tuple[float, float, float, float]
Pose = Tuple[CartesianPosition, Orientation]
CartesianState = Tuple[float, float, float, float]  # (ee_x, ee_y, ee_z, gripper_width)
