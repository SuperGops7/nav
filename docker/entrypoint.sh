#!/bin/bash
set -e # Exit on error

source nav_ws/install/setup.bash

ros2 launch turtlebot3_gazebo empty_world.launch.py

# Execute the command provided as arguments
exec "$@"
