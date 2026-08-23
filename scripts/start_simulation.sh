#!/usr/bin/env bash

# start_simulation.sh
#
# Starts the entire OpenRover simulation stack with one command:
# Gazebo, the world with the rover in it, the ROS 2 <-> Gazebo
# bridge, and the command interface node — everything that used to
# require three separate terminals.
#
# Usage (from anywhere, run inside WSL):
#   ~/ros2_ws/src/OpenRover/scripts/start_simulation.sh
#
# "set -e" means: if any command in this script fails, stop
# immediately instead of continuing on and hiding the real problem
# behind a confusing later error.
set -e

echo "Starting OpenRover simulation..."

# Make ROS 2 itself available in this shell.
source /opt/ros/jazzy/setup.bash

# Make our own built packages (openrover_simulation, openrover_control)
# available. This assumes the standard workspace location used
# throughout this project's setup: ~/ros2_ws.
source "$HOME/ros2_ws/install/setup.bash"

# "$@" forwards any extra arguments this script was called with
# straight through to "ros2 launch" — for example, if you ever want
# to override a launch argument without editing this script.
ros2 launch openrover_simulation full_system.launch.py "$@"
