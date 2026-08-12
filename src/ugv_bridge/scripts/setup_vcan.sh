#!/usr/bin/env bash
# setup_vcan.sh — Crea y levanta la interfaz CAN virtual vcan0 (SocketCAN).
#
# Correr con root una vez por arranque del PC (la interfaz no persiste reinicios):
#     sudo bash src/ugv_bridge/scripts/setup_vcan.sh
#
# CORRER EN EL HOST, no dentro de rosdev. El contenedor comparte red y kernel con
# el host, así que verá la interfaz igual, pero NO puede cargar módulos: no tiene
# /lib/modules ni privilegios para hacerlo.
#
# OJO: crear la interfaz NO basta. Abrir un socket SocketCAN necesita además el
# núcleo del protocolo ('can') y el socket crudo ('can_raw'). El kernel los
# autocargaría al primer socket(AF_CAN), pero eso no ocurre desde un proceso sin
# privilegios ni desde dentro de rosdev: falla con "Address family not supported
# by protocol" y el emulador/driver mueren al arrancar. Por eso se cargan aquí,
# fuera del `if`: en la segunda corrida la interfaz ya existe pero los módulos
# pueden seguir sin estar.
set -e

IFACE="${1:-vcan0}"

# Tras actualizar el kernel sin reiniciar, el árbol de módulos del kernel EN
# MARCHA desaparece y modprobe no puede cargar nada nuevo. Se detecta aquí
# porque si no el síntoma es incomprensible: `ip link` muestra vcan0 arriba y
# aun así ningún socket CAN abre.
if [ ! -d "/lib/modules/$(uname -r)" ]; then
    echo "[setup_vcan] AVISO: no existe /lib/modules/$(uname -r)."
    echo "[setup_vcan]        El kernel en marcha ya no tiene su árbol de módulos"
    echo "[setup_vcan]        (típico tras actualizarlo sin reiniciar). Hay que"
    echo "[setup_vcan]        REINICIAR antes de poder usar SocketCAN."
fi

for mod in can can_raw vcan; do
    modprobe "$mod" 2>/dev/null || true
done

if ! ip link show "$IFACE" &>/dev/null; then
    ip link add dev "$IFACE" type vcan
    echo "[setup_vcan] interfaz $IFACE creada"
fi

ip link set up "$IFACE"
echo "[setup_vcan] $IFACE arriba:"
ip -brief link show "$IFACE"

# Comprobación real: que un socket CAN se pueda abrir de verdad, no sólo que la
# interfaz figure en `ip link`.
if python3 -c 'import socket; socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)' 2>/dev/null; then
    echo "[setup_vcan] socket AF_CAN OK"
else
    echo "[setup_vcan] AVISO: la interfaz existe pero no se puede abrir un socket AF_CAN."
    echo "[setup_vcan]        Revisar que 'can' y 'can_raw' estén cargados: lsmod | grep can"
fi
