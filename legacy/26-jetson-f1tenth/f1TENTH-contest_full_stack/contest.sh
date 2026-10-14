#!/usr/bin/env bash
# One entry point for build, joystick mapping, automatic track extraction and racing.
set -Eeo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$ROOT/f1tenth_ws"
DATA="$ROOT/track_data"
STATE="$DATA/active_track"
ROS_DISTRO="${ROS_DISTRO:-humble}"
mkdir -p "$DATA/maps" "$DATA/waypoints"

# ROS setup files may read optional variables before they are defined.  Keep
# nounset disabled only while sourcing them, then restore strict checking.
set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

build_stack() {
  cd "$WS"
  colcon build --symlink-install --packages-up-to contest_autonomy f1tenth_stack
  set +u
  source "$WS/install/setup.bash"
  set -u
}

stop_mapping() {
  if [[ -n "${MAPPING_PID:-}" ]]; then
    kill -INT "$MAPPING_PID" 2>/dev/null || true
    wait "$MAPPING_PID" 2>/dev/null || true
  fi
}

learn_track() {
  local name="${1:?usage: ./contest.sh learn TRACK_NAME}"
  [[ "$name" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "TRACK_NAME may contain only A-Z, 0-9, _ and -"; exit 2; }
  local map_base="$DATA/maps/$name"
  local waypoint="$DATA/waypoints/${name}.csv"
  build_stack
  ros2 launch contest_autonomy mapping.launch.py &
  MAPPING_PID=$!
  trap stop_mapping EXIT INT TERM
  echo
  echo "[1/3] Hold the joystick deadman (LB) and drive 2-3 slow complete laps."
  echo "      Keep walls visible on both sides and finish at the starting pose."
  echo "      When loop closure has settled, STOP the car and press Enter here."
  read -r
  ros2 topic info /map | grep -q 'Publisher count: [1-9]' || {
    echo "ERROR: /map has no publisher; mapping remains running for diagnosis."; exit 3; }
  echo "[2/3] Saving map -> $map_base.yaml + $map_base.pgm"
  ros2 run nav2_map_server map_saver_cli -f "$map_base" \
    --ros-args -p map_subscribe_transient_local:=true
  stop_mapping; MAPPING_PID=""; trap - EXIT INT TERM
  echo "[3/3] Extracting and validating safe racing line -> $waypoint"
  ros2 run contest_autonomy auto_track --map "$map_base.yaml" --output "$waypoint" \
    --safety 0.28 --vmax 3.0 --vmin 0.45 --mu 0.75
  printf '%s\n' "$name" > "$STATE"
  echo
  echo "READY: $name"
  echo "Preview: ${waypoint%.csv}_preview.png"
  echo "After visually checking the preview, run: ./contest.sh race $name X Y YAW"
}

race_track() {
  local name="${1:-}"
  if [[ -z "$name" && -f "$STATE" ]]; then name="$(<"$STATE")"; fi
  [[ -n "$name" ]] || { echo "No active track. Run ./contest.sh learn TRACK_NAME first."; exit 2; }
  local map="$DATA/maps/$name.yaml" waypoint="$DATA/waypoints/$name.csv"
  [[ -f "$map" && -f "$waypoint" ]] || { echo "Missing map or waypoint for '$name'."; exit 2; }
  local x="${2:-0.0}" y="${3:-0.0}" heading="${4:-0.0}"
  build_stack
  printf '%s\n' "$name" > "$STATE"
  echo "RACE: track=$name initial_pose=($x,$y,$heading)"
  exec ros2 launch contest_autonomy race.launch.py \
    map:="$map" waypoints:="$waypoint" x:="$x" y:="$y" yaw:="$heading"
}

case "${1:-help}" in
  build) build_stack ;;
  learn) learn_track "${2:-}" ;;
  race) shift; race_track "$@" ;;
  *)
    echo "Usage:"
    echo "  ./contest.sh build"
    echo "  ./contest.sh learn TRACK_NAME"
    echo "  ./contest.sh race [TRACK_NAME] [START_X START_Y START_YAW]"
    exit 2 ;;
esac
