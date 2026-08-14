#!/usr/bin/env bash
# Обновить server_ip в config.toml на текущий IP этой машины.
# Берём первый non-loopback IPv4 адрес.

set -e

IP=$(ip route get 1.1.1.1 2>/dev/null | awk '/src/ {print $7; exit}')

if [ -z "$IP" ]; then
    echo "ERROR: не удалось определить IP" >&2
    exit 1
fi

CONFIG="$(dirname "$0")/config.toml"

sed -i "s/^server_ip = \".*\"/server_ip = \"$IP\"/" "$CONFIG"

echo "server_ip → $IP"
