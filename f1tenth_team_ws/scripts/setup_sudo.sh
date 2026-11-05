#!/bin/bash
# 새 Jetson에서 한 번만 실행: udev 규칙, 의존 패키지, Hokuyo 이더넷 설정 (sudo 필요)
# 지금 차의 Jetson은 이미 설정돼 있어서 실행할 필요 없음
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "== udev 규칙"
echo 'KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"' \
  | sudo tee /etc/udev/rules.d/99-vesc.rules
echo 'KERNEL=="js[0-9]*", ACTION=="add", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c219", SYMLINK+="input/joypad-f710"' \
  | sudo tee /etc/udev/rules.d/99-joypad-f710.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "== 의존 패키지"
source /opt/ros/humble/setup.bash
cd "$WS"
sudo apt-get update
sudo apt-get install -y ros-humble-asio-cmake-module ros-humble-slam-toolbox \
  ros-humble-nav2-map-server ros-humble-nav2-amcl ros-humble-nav2-lifecycle-manager
rosdep install --include-eol --from-paths src -i -y --rosdistro=humble

echo "== Hokuyo 이더넷 프로파일 (192.168.0.15/24, 게이트웨이 없음: Wi-Fi 인터넷 유지)"
if ! nmcli -t -f NAME con show | grep -qx Hokuyo; then
  nmcli con add type ethernet ifname enP8p1s0 con-name Hokuyo \
    ipv4.method manual ipv4.addresses 192.168.0.15/24 ipv4.never-default yes \
    ipv6.method disabled connection.autoconnect-priority 10
fi
nmcli con up Hokuyo

echo "== 완료"
ls -l /dev/sensors/
ping -c 2 192.168.0.10
