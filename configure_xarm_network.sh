#!/usr/bin/env bash
set -euo pipefail

connection_name="xarm-eth0"
xarm_host_address="192.168.1.100/24"

# Use a dedicated NetworkManager keyfile instead of modifying netplan-eth0:
# that generated profile retains DHCP even after an nmcli method change.
if ! nmcli --get-values NAME connection show | grep -Fxq "$connection_name"; then
  nmcli connection add type ethernet ifname eth0 con-name "$connection_name"
fi

# Give the wired xArm network a persistent static address. It intentionally has
# no gateway or DNS and is never eligible to replace Wi-Fi's default route.
nmcli connection modify "$connection_name" \
  connection.autoconnect yes \
  connection.autoconnect-priority 100 \
  ipv4.method manual \
  ipv4.addresses "$xarm_host_address" \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv4.never-default yes \
  ipv4.ignore-auto-routes yes \
  ipv6.method disabled \
  ipv6.never-default yes

nmcli connection up "$connection_name"

echo "Configured $connection_name with $xarm_host_address"
ip -brief address show dev eth0
ip route show default
ip route show 192.168.1.0/24
