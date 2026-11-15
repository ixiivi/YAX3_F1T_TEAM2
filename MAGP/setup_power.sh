#!/bin/bash
# One-time setup (asks for the sudo password once): let the control panel switch the Jetson to
# full power (MAXN_SUPER + jetson_clocks) by itself at start, without a password. Only these two
# exact commands are allowed, nothing else. Undo: sudo rm /etc/sudoers.d/f1t-power
set -e
RULE="$USER ALL=(root) NOPASSWD: /usr/sbin/nvpmodel -m 2, /usr/bin/jetson_clocks"
TMP=$(mktemp)
echo "$RULE" > "$TMP"
sudo visudo -cf "$TMP"                      # refuse a broken rule before installing it
sudo install -m 0440 -o root -g root "$TMP" /etc/sudoers.d/f1t-power
rm -f "$TMP"
echo "installed: $RULE"
sudo -n /usr/sbin/nvpmodel -m 2 && sudo -n /usr/bin/jetson_clocks && nvpmodel -q | head -1
