# franka_python

**Under construction, not guaranteed to work**

Python-ROS interface for Franka robot.

[![Actions Status](https://github.com/qgallouedec/franka_python/workflows/build/badge.svg)](https://github.com/qgallouedec/franka_python/actions)


## Installation

- Install `libfranka` and `franka-ros` (more details [here](https://frankaemika.github.io/docs/installation_linux.html))
- Install `panda_moveit_config` in your `catkin_ws`.

```bash
cd ~/catkin_ws/src
git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro melodic
```

- Clone the repository and install it.

```bash
git clone https://github.com/qgallouedec/franka_python
pip install -e franka_python
```

## Usage

Source your `catkin_ws` and run

```bash
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<robot_ip>
```


### Arm interface

Example script to move the robot to its neutral position:

```python
import franka_python
import rospy

rospy.init_node('my_node')
arm = franka_python.ArmInterface() 
arm.move_to_neutral()
```


### Gripper iterface

Example script to open then close the gripper:

```python
import franka_python
import rospy
import time

rospy.init_node('my_node')
gripper = franka_python.GripperInterface() 
gripper.open()
time.sleep(2)
gripper.close()
```