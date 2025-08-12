# Joint Mover C++ Package

C++ ROS 2 package for controlling joint movements on Fanuc robots.

## Dependencies
- rclcpp
- trajectory_msgs
- sensor_msgs
- std_msgs

## Installation
```bash
cd ~/your_ws/src
git clone https://github.com/jatinmayekar/fanuc_joint_mover_cpp.git
cd ..
colcon build --packages-select joint_mover_cpp
source install/setup.bash
```

## Usage
```bash
ros2 launch joint_mover_cpp j2_mover_cpp.launch.py
```

## Description
C++ implementation that moves J2 joint in a sine wave pattern with 15-degree amplitude and sets a DO 1 to True.
Equivalent functionality to the Python joint_mover package but implemented in C++.
