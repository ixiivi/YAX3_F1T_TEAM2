#!/bin/bash
# start the F1TENTH control panel with the ROS environment loaded
source /opt/ros/humble/setup.bash
source "$HOME/MAGP/MAGP/install/setup.bash"
exec python3 "$HOME/MAGP/MAGP/control_panel.py" "$@"
