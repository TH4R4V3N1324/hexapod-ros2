#!/bin/sh
# Create uap0, a second interface on the Wi-Fi radio for the hotspot, alongside wlan0 (home Wi-Fi).
# Run at boot by hexapod-uap0.service, before NetworkManager starts.
/usr/sbin/iw dev uap0 info >/dev/null 2>&1 || /usr/sbin/iw dev wlan0 interface add uap0 type __ap || exit 1
# Its own MAC address: wlan0's with the locally administered bit set, plus one more bit, since the
# driver's Wi-Fi Direct device already uses wlan0's address with just that bit
mac=$(cat /sys/class/net/wlan0/address)
first=$(printf '%02x' $((0x${mac%%:*} | 6)))
ip link set uap0 address "$first:${mac#*:}"
