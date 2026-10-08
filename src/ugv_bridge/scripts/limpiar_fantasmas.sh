#!/usr/bin/env bash
# limpiar_fantasmas.sh — Matar procesos huerfanos que siguen hablando por ROS.
#
# POR QUE EXISTE
# --------------
# El proyecto se controlaba antes con ventanas: RViz, los sliders de
# joint_state_publisher_gui y la ventana Qt del viejo can_monitor. Esas ventanas
# NO siempre mueren con el Ctrl+C del launch que las abrio, y las dos ultimas
# PUBLICAN /cmd_flippers y /cmd_tracks. Como flipper_node retiene el ultimo
# comando que le llega, un fantasma vivo pisa lo que mandas desde Foxglove: el
# flipper se va solo a otro angulo, o simplemente ignora tu orden, y no hay nada
# en pantalla que lo explique.
#
# Hoy ningun launch abre esas ventanas, pero una sesion antigua o un
# `ros2 topic pub -r` olvidado siguen pudiendo hacerlo. Esto los barre.
#
# Uso:
#     bash src/ugv_bridge/scripts/limpiar_fantasmas.sh
#     bash src/ugv_bridge/scripts/limpiar_fantasmas.sh --ver   # solo listar
set -eo pipefail

PATRONES=(
    "ros2 launch ugv_bridge"
    "lib/ugv_bridge/"
    "ugv_bridge.can_monitor"
    "ugv_bridge.gui_a_comandos"
    "joint_state_publisher_gui"
    "rviz2"
    "robot_localization/ekf_node"
    "foxglove_bridge/foxglove_bridge"
    "lib/robot_state_publisher"
    # Publicadores olvidados: `ros2 topic pub -r 20 /cmd_tracks ...` deja el
    # robot conduciendose solo para siempre.
    "topic pub"
)

echo "Procesos ROS del proyecto que siguen vivos:"
ENCONTRADO=0
for p in "${PATRONES[@]}"; do
    if pgrep -af "$p" >/dev/null 2>&1; then
        pgrep -af "$p" | sed 's/^/  /'
        ENCONTRADO=1
    fi
done
[ "$ENCONTRADO" = 0 ] && { echo "  (ninguno)"; exit 0; }

if [ "${1:-}" = "--ver" ]; then
    echo
    echo "Modo --ver: no se ha matado nada. Repite sin --ver para cerrarlos."
    exit 0
fi

echo
echo "Cerrando..."
# Primero por las buenas (los nodos cierran sus sockets CAN y SPI).
pkill -INT -f "ros2 launch ugv_bridge" 2>/dev/null || true
sleep 2
for p in "${PATRONES[@]}"; do
    pkill -9 -f "$p" 2>/dev/null || true
done
sleep 1

# El daemon cachea el grafo: sin esto `ros2 node list` sigue mostrando muertos.
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "Listo. Comprobar con: ros2 node list"
