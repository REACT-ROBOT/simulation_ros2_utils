# simulation_ros2_utils

## Overview
This package provides utility functions and tools for ROS2-based simulations.

## Features
- Communication utilities between ROS2 and Simulator
- Spawn Entity
- Set simulator state (to start / stop / pause simulation)

## Dependencies
- ROS2 (Humble or later recommended)
- geometry_msgs
- std_msgs
- tf2_ros
- rclcpp
- simulation_interfaces

## Installation

```bash
# Clone the repository into your workspace
cd ~/your_workspace/src
git clone https://github.com/hijimasa/simulation_ros2_utils.git

# Build the package
cd ..
colcon build --packages-select simulation_ros2_utils
source install/setup.bash
```

## License
This project is licensed under the Apache License 2.0 - see the LICENSE file for details.