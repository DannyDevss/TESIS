#!/usr/bin/env bash
# setup_vcan.sh — Crea y levanta la interfaz CAN virtual vcan0 (SocketCAN).
#
# Correr con root una vez por arranque del PC (la interfaz no persiste reinicios):
#     sudo bash src/ugv_bridge/scripts/setup_vcan.sh
#
# Funciona desde el host o desde el contenedor rosdev (comparten red y kernel).
# En CachyOS el soporte vcan viene integrado al kernel; el modprobe es por si
# la distro lo trae como módulo aparte.
set -e

IFACE="${1:-vcan0}"

if ! ip link show "$IFACE" &>/dev/null; then
    modprobe vcan 2>/dev/null || true
    ip link add dev "$IFACE" type vcan
    echo "[setup_vcan] interfaz $IFACE creada"
fi

ip link set up "$IFACE"
echo "[setup_vcan] $IFACE arriba:"
ip -brief link show "$IFACE"
