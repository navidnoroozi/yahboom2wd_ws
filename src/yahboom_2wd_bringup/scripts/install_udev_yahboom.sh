#!/usr/bin/env bash
set -euo pipefail

# Creates /dev/myserial for common CH340/USB-serial Yahboom boards.
# Run once on every Raspberry Pi: sudo bash install_udev_yahboom.sh

RULE_FILE=/etc/udev/rules.d/99-yahboom-rosboard.rules
cat <<'RULE' | sudo tee "$RULE_FILE" >/dev/null
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="myserial", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="5523", SYMLINK+="myserial", MODE="0666", GROUP="dialout"
RULE
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -a -G dialout "$USER" || true

echo "Installed $RULE_FILE. Unplug/replug the USB cable, then run: ls -l /dev/myserial"
echo "Log out/in or run 'newgrp dialout' so your current shell sees the dialout group."
