#!/bin/bash
# RoboRacer driver stack: steps that need sudo (udev rules, rosdep deps, Hokuyo network)
set -e

echo "== 3. udev rules"
echo 'KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"' \
  | sudo tee /etc/udev/rules.d/99-vesc.rules
echo 'KERNEL=="js[0-9]*", ACTION=="add", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c219", SYMLINK+="input/joypad-f710"' \
  | sudo tee /etc/udev/rules.d/99-joypad-f710.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "== 4. rosdep dependencies"
source /opt/ros/humble/setup.bash
cd "$HOME/f1tenth_ws"
sudo apt-get update
sudo apt-get install -y ros-humble-asio-cmake-module
rosdep install --include-eol --from-paths src -i -y --rosdistro=humble

echo "== 5. Hokuyo ethernet profile (192.168.0.15/24, no gateway so WiFi keeps internet)"
if ! nmcli -t -f NAME con show | grep -qx Hokuyo; then
  nmcli con add type ethernet ifname enP8p1s0 con-name Hokuyo \
    ipv4.method manual ipv4.addresses 192.168.0.15/24 ipv4.never-default yes \
    ipv6.method disabled connection.autoconnect-priority 10
fi
nmcli con up Hokuyo

echo "== done"
ls -l /dev/sensors/
ping -c 2 192.168.0.10
