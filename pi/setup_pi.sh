#!/bin/bash
# Set up the robot's Raspberry Pi 5 (Raspberry Pi OS trixie). Safe to run again.
#
#   pi/setup_pi.sh                            # keeps the hotspot password if one is set
#   HOTSPOT_PASSWORD=... pi/setup_pi.sh       # sets or changes it (8+ characters)
#
# Run it as your normal user (it uses sudo), then reboot. It:
#   - turns on I2C at 400 kHz for the IMU and power monitors, and installs i2c-tools and iw
#   - turns on the servo UART, /dev/ttyAMA0
#   - installs the udev rule that names the lidar /dev/rplidar
#   - names the Pi "hexapod", including in cloud-init, which otherwise renames it at boot
#   - adds a "hexapod" Wi-Fi hotspot (192.168.50.1) on a second interface, uap0, while wlan0
#     stays on the home Wi-Fi for internet. The radio can only use one channel at a time and the
#     hotspot can't use 5 GHz radar channels, so wlan0 is kept to 2.4 GHz and the hotspot follows
#     its channel.
#   - starts the robot at boot: pull the latest code, rebuild if needed, docker compose up
set -euo pipefail
PI_DIR=$(cd "$(dirname "$0")" && pwd)
REPO=${HEXAPOD_REPO:-$(cd "$PI_DIR/.." && pwd)}
HOSTNAME_NEW=hexapod
HOTSPOT_SSID=hexapod
HOTSPOT_ADDRESS=192.168.50.1/24

echo "== I2C"
sudo raspi-config nonint do_i2c 0
grep -q '^dtparam=i2c_arm_baudrate=' /boot/firmware/config.txt ||
    sudo sed -i '/^dtparam=i2c_arm=on/a dtparam=i2c_arm_baudrate=400000' /boot/firmware/config.txt
sudo apt-get install -y -qq i2c-tools iw >/dev/null

echo "== Servo UART (/dev/ttyAMA0 on GPIO14/15)"
grep -q '^dtparam=uart0=on' /boot/firmware/config.txt ||
    echo 'dtparam=uart0=on' | sudo tee -a /boot/firmware/config.txt >/dev/null

echo "== Lidar udev rule"
sudo install -m 644 "$REPO/hexapod_ws/udev/99-rplidar.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules

echo "== Hostname: $HOSTNAME_NEW"
sudo hostnamectl set-hostname "$HOSTNAME_NEW"
sudo sed -i "s/^127\.0\.1\.1\s.*/127.0.1.1 $HOSTNAME_NEW/" /etc/hosts
if [ -f /boot/firmware/user-data ]; then
    sudo sed -i "s/^hostname: .*/hostname: $HOSTNAME_NEW/" /boot/firmware/user-data
fi

echo "== Hotspot"
sudo install -m 755 "$PI_DIR/hexapod-uap0.sh" /usr/local/bin/
sudo install -m 644 "$PI_DIR/hexapod-uap0.service" /etc/systemd/system/
sudo install -m 755 "$PI_DIR/hotspot-channel" /etc/NetworkManager/dispatcher.d/90-hexapod-hotspot-channel
sudo systemctl daemon-reload
sudo systemctl enable hexapod-uap0.service
# Home Wi-Fi on 2.4 GHz only, so the hotspot can share its channel
nmcli -t -f NAME,TYPE connection show | while IFS=: read -r name type; do
    if [ "$type" = 802-11-wireless ] && [ "$name" != hexapod-hotspot ]; then
        sudo nmcli connection modify "$name" 802-11-wireless.band bg
        echo "Home Wi-Fi '$name' set to 2.4 GHz"
    fi
done
if ! nmcli connection show hexapod-hotspot >/dev/null 2>&1; then
    if [ -z "${HOTSPOT_PASSWORD:-}" ]; then
        # tr is cut off by head mid-write; that's expected, not a failure
        HOTSPOT_PASSWORD=$(tr -dc 'a-km-np-zA-HJ-NP-Z2-9' </dev/urandom | head -c 12 || true)
        echo "Hotspot password (generated): $HOTSPOT_PASSWORD"
    fi
    # PMF off: the Pi's Wi-Fi chip doesn't support it as an access point
    sudo nmcli connection add type wifi ifname uap0 con-name hexapod-hotspot autoconnect yes \
        ssid "$HOTSPOT_SSID" 802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 6 \
        wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$HOTSPOT_PASSWORD" wifi-sec.proto rsn \
        wifi-sec.pairwise ccmp wifi-sec.group ccmp wifi-sec.pmf disable \
        ipv4.method shared ipv4.addresses "$HOTSPOT_ADDRESS" ipv6.method disabled >/dev/null
elif [ -n "${HOTSPOT_PASSWORD:-}" ]; then
    sudo nmcli connection modify hexapod-hotspot wifi-sec.psk "$HOTSPOT_PASSWORD"
    echo "Hotspot password changed"
fi
echo "Hotspot '$HOTSPOT_SSID' at ${HOTSPOT_ADDRESS%/*}"

echo "== Robot at boot"
sudo install -m 755 "$PI_DIR/hexapod-start.sh" /usr/local/bin/
sed -e "s#@USER@#$USER#" -e "s#@REPO@#$REPO#" "$PI_DIR/hexapod-start.service" |
    sudo tee /etc/systemd/system/hexapod-start.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable hexapod-start.service

echo "Done. Reboot to apply: sudo reboot"
