#!/bin/bash
# start the F1TENTH control panel with the ROS environment loaded
export ROS_LOCALHOST_ONLY=1   # ROS only inside the Jetson (same as ~/.bashrc)
source /opt/ros/humble/setup.bash
source "$HOME/MAGP/MAGP/install/setup.bash"
exec python3 "$HOME/MAGP/MAGP/control_panel.py" "$@"
