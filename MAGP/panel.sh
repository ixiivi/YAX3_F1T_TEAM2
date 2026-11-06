#!/bin/bash
# start the F1TENTH control panel with the ROS environment loaded
source /opt/ros/humble/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"
exec python3 "$HOME/f1tenth_ws/control_panel.py" "$@"
