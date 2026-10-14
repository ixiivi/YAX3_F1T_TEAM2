#!/usr/bin/env bash
# ============================================================
#  envsetup.sh — 진단 도구를 돌릴 두 번째 터미널용 환경 설정.
#
#      cd ~/yax/26-jetson-f1tenth
#      source envsetup.sh
#      python3 odom_dir_check.py
#
#  ★ 왜 필요한가
#    ./race.sh 는 내부에서 config/dds_env.sh 를 불러 ROS_DOMAIN_ID 와
#    RMW(CycloneDDS)를 설정한다. 그런데 두 번째 터미널에서 ROS 만 source 하면
#    **도메인 0 + FastDDS** 로 떠서 스택과 서로 아예 안 보인다.
#    토픽이 하나도 안 잡히는데 스택은 멀쩡히 돌고 있어 원인을 찾기 어렵다
#    (2026-08-10: odom_dir_check 가 /odom_imu 를 한 건도 못 받았다).
#
#    ros2 topic list / echo, odom_dir_check.py, odom_scale, yaw_check.py,
#    imu_yaw_scale_check.py 등 **스택 밖에서 도는 것은 전부** 이게 필요하다.
#    (./race.sh check · savemap 은 race.sh 안에서 알아서 한다)
#
#  ※ source 로 실행할 것. ./envsetup.sh 로 실행하면 자식 셸에만 적용돼
#    아무 효과가 없다.
# ============================================================

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "✗ source 로 실행하세요:   source envsetup.sh"
    exit 1
fi

_ENV_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

source /opt/ros/humble/setup.bash
source "$HOME/f1tenth_ws/install/setup.bash"

# race.sh 와 같은 규약 — 도구들이 repo 경로를 이걸로 찾는다.
export RACE_REPO="$_ENV_DIR"
export RACE_WS="${RACE_WS:-$HOME/f1tenth_ws}"
export RACE_SCRIPT="$_ENV_DIR/race_script.yaml"

# 차량 overlay 가 있으면 도메인·피어가 여기서 정해진다 (.car → apply_vehicle.py).
# 없으면 race.sh 를 한 번이라도 돌려서 만들어야 한다.
if [ -f "$_ENV_DIR/config/.vehicle_overlay.yaml" ]; then
    export RACE_VEHICLE_OVERLAY="$_ENV_DIR/config/.vehicle_overlay.yaml"
else
    echo "[envsetup] ⚠ config/.vehicle_overlay.yaml 이 없습니다."
    echo "           ./race.sh 를 한 번 돌리면 생깁니다 (apply_vehicle.py)."
fi

source "$_ENV_DIR/config/dds_env.sh"

echo "[envsetup] 준비 완료 — ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW=${RMW_IMPLEMENTATION:-기본}"
echo "[envsetup]   ★ 스택 터미널과 이 두 줄이 같아야 토픽이 보입니다."
