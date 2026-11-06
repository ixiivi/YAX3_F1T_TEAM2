#!/bin/bash
# Make the F710 (D mode, 046d:c219) usable on this Jetson. Run once: sudo ./setup_joystick.sh
#
# The kernel's Logitech HID driver claims this pad, then fails its probe (error -1) because
# CONFIG_LOGIRUMBLEPAD2_FF is off, so no /dev/input/js* appears. Binding the generic HID driver
# instead gives a normal joystick (no rumble). A udev rule repeats this at boot and on replug.
set -e
[ "$(id -u)" = 0 ] || { echo "sudo로 실행하세요: sudo $0"; exit 1; }

RULE=/etc/udev/rules.d/99-f710-hid-generic.rules
HELPER=/usr/local/sbin/f710-hid-generic

cat > "$HELPER" <<'EOF'
#!/bin/sh
# $1: HID device name. Wait out the failing Logitech probe, then bind the generic driver.
echo 1 > /sys/module/hid/parameters/ignore_special_drivers
sleep 1
[ -e "/sys/bus/hid/devices/$1/driver" ] || echo "$1" > /sys/bus/hid/drivers/hid-generic/bind
EOF
chmod 755 "$HELPER"

echo "ACTION==\"add\", SUBSYSTEM==\"hid\", ENV{HID_ID}==\"0003:0000046D:0000C219\", RUN+=\"$HELPER %k\"" > "$RULE"
udevadm control --reload-rules

found=
for dev in /sys/bus/hid/devices/0003:046D:C219.*; do
    [ -e "$dev" ] || continue
    found=1
    "$HELPER" "$(basename "$dev")"
done
[ -n "$found" ] || { echo "F710이 안 보여요. 수신기를 꽂고 D 모드인지 확인한 뒤 다시 실행하세요."; exit 1; }

sleep 1
if ls /dev/input/js* >/dev/null 2>&1; then
    echo "완료:"
    ls -l /dev/input/js*
else
    echo "실패: /dev/input/js*가 안 생겼어요. 수신기를 뺐다 다시 꽂아 보고, dmesg | tail 을 확인하세요."
    exit 1
fi
