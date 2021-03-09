# franka_gym

[![Actions Status](https://github.com/qgallouedec/franka_gym/workflows/build/badge.svg)](https://github.com/qgallouedec/franka_gym/actions)

Real OpenAI gym interface for Franka Emika Panda robot

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
git clone https://github.com/qgallouedec/franka_gym
pip install -e franka_gym
```

## Usage

### Arm interface

Source your `catkin_ws` and run

```bash
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<robot_ip>
```

Then, run this python script.

```python
import franka_gym
arm = franka_gym.ArmInterface() 
arm.move_to_neutral()
```

## RealSense

```bash
roslaunch realsense2_camera rs_camera.launch 
```
