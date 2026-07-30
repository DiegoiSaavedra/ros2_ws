#!/bin/bash
# Crea la regla udev para el ESP32 (LOLIN S2 Mini): symlink estable /dev/esp32
# Ejecutar con: sudo bash instalar_udev_esp32.sh
set -e
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="80c2", SYMLINK+="esp32", MODE="0660", GROUP="dialout"' > /etc/udev/rules.d/99-esp32.rules
udevadm control --reload
udevadm trigger
sleep 1
ls -l /dev/esp32 && echo "OK: /dev/esp32 creado"
