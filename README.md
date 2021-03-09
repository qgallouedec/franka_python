# franka_gym

[![Actions Status](https://github.com/qgallouedec/franka_gym/workflows/build/badge.svg)](https://github.com/qgallouedec/franka_gym/actions)

Real OpenAI gym interface for Franka Emika Panda robot


## Installation

- Install `libfranka` and `franka-ros` (more details [here](https://frankaemika.github.io/docs/installation_linux.html))

Clone the repository and install it.

```bash
git clone https://github.com/qgallouedec/franka_gym
pip install -e franka_gym
```

## Usage

```bash
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<robot_ip>
```

```python
import franka_gym
```


## RealSense

```bash
roslaunch realsense2_camera rs_camera.launch 
```
