#!/bin/bash
# start the F1TENTH control panel with the ROS environment loaded
# full power for driving (MAXN_SUPER: CPU ~30% faster than 25 W). Needs ./setup_power.sh once;
# without it sudo -n just fails quietly and the panel starts as before
if ! nvpmodel -q 2>/dev/null | grep -q MAXN_SUPER; then
  sudo -n /usr/sbin/nvpmodel -m 2 >/dev/null 2>&1 < /dev/null
fi
sudo -n /usr/bin/jetson_clocks >/dev/null 2>&1 < /dev/null
export ROS_LOCALHOST_ONLY=1   # ROS only inside the Jetson (same as ~/.bashrc)
source /opt/ros/humble/setup.bash
source "$HOME/MAGP/MAGP/install/setup.bash"
exec python3 "$HOME/MAGP/MAGP/control_panel.py" "$@"
