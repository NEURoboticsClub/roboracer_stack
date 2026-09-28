#!/bin/bash
set -e

# Setup Development Environment
source /opt/ros/humble/setup.bash
cd /home/ros_ws
colcon build
source /home/ros_ws/install/setup.bash

# AutoDRIVE Devkit Workspace
cd /home/ros_ws

# Launch AutoDRIVE Devkit with GUI
# ros2 launch autodrive_roboracer bringup_graphics.launch.py

# Launch AutoDRIVE Devkit Headless
# ros2 launch autodrive_roboracer bringup_headless.launch.py
# ros2 launch neo_telemetry neo_telemetry_launch.py


exec "$@"