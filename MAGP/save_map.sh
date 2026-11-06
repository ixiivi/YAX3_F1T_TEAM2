#!/bin/bash
# Save the running slam_toolbox map to ~/f1tenth_ws/maps/<name>.{pgm,yaml,posegraph,data}
# usage: ~/f1tenth_ws/save_map.sh [name]     (default name: map_MMDD_HHMM)
set -e
source /opt/ros/humble/setup.bash
NAME=${1:-map_$(date +%m%d_%H%M)}
DIR=$HOME/f1tenth_ws/maps
mkdir -p "$DIR" && cd "$DIR"

ros2 run nav2_map_server map_saver_cli -f "$NAME" --ros-args -p map_subscribe_transient_local:=true
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$DIR/$NAME'}" > /dev/null

echo
echo "saved: $DIR/$NAME.pgm / .yaml / .posegraph"
echo "waypoints: ros2 run waypoint_nav waypoint_editor $DIR/$NAME.yaml"
