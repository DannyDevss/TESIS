#!/usr/bin/env bash
# tesis_lanzar.sh — Compila y levanta la tesis en la Raspberry, de una sola vez.
#
# No se invoca directamente: se instala en ~/.local/bin con dos nombres, y el
# nombre con el que se llama decide el modo de los motores.
#
#     compilar_simu   motores SIMULADOS (gemelo digital) + IMU FISICA real
#     compilar_real   motores REALES por los buses CAN del pi3hat + IMU real
#
# En los dos casos: EKF encendido (sin el, el chasis no se inclina) y puente
# Foxglove en el 8765, que es la unica visualizacion posible en una Pi headless.
#
# Instalar (o reinstalar tras cambiar este archivo):
#     bash src/ugv_bridge/scripts/instalar_comandos.sh
#
# Cualquier argumento extra se pasa tal cual al launch, asi que se puede hacer:
#     compilar_simu frecuencia_hz:=200.0
#     compilar_real use_foxglove:=false
# Sin `set -u`: los setup.bash de ROS 2 referencian variables sin definir
# (AMENT_TRACE_SETUP_FILES y compañía) y abortarian el script al sourcearlos.
set -eo pipefail

WS="${TESIS_WS:-$HOME/TESIS}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

case "$(basename "$0")" in
    compilar_real)  MODO_ARGS=(modo:=pi3hat) ; ETIQUETA="MOTORES REALES (pi3hat)" ;;
    compilar_simu)  MODO_ARGS=(modo:=gemelo) ; ETIQUETA="motores simulados (gemelo digital)" ;;
    *)
        echo "Llamar como compilar_real o compilar_simu, no como $(basename "$0")." >&2
        exit 2
        ;;
esac

echo "=============================================================="
echo " TESIS UGV  |  $ETIQUETA  +  IMU FISICA del pi3hat"
echo " Workspace: $WS"
echo "=============================================================="

[ -f "$ROS_SETUP" ] || { echo "No existe $ROS_SETUP. ¿Es esta la Raspberry?" >&2; exit 1; }
[ -d "$WS/src" ]    || { echo "No existe $WS/src (define TESIS_WS si el repo esta en otro sitio)." >&2; exit 1; }

# --- 1. Matar restos de una corrida anterior --------------------------------
# El pi3hat admite UN solo dueño del bus SPI. Si un launch anterior fallo a
# medias, sus nodos siguen vivos y el arranque muere con
# "pi3hat: could not acquire lock", que parece un fallo de la placa y no lo es.
echo "[1/3] Cerrando nodos de una corrida anterior..."
pkill -INT -f "ros2 launch ugv_bridge"          2>/dev/null || true
sleep 2
pkill -9    -f "lib/ugv_bridge/"                2>/dev/null || true
pkill -9    -f "robot_localization/ekf_node"    2>/dev/null || true
pkill -9    -f "foxglove_bridge/foxglove_bridge" 2>/dev/null || true
pkill -9    -f "lib/robot_state_publisher"      2>/dev/null || true
sleep 1

# --- 2. Compilar ------------------------------------------------------------
echo "[2/3] Compilando ugv_bridge..."
# shellcheck disable=SC1090
source "$ROS_SETUP"
cd "$WS"
colcon build --packages-select ugv_bridge

# --- 3. Lanzar --------------------------------------------------------------
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

IP=$(hostname -I | awk '{print $1}')
echo "[3/3] Lanzando. Foxglove desde el PC:  ws://$(hostname).local:8765   (o ws://$IP:8765)"
echo "      Panel 3D con Display frame = odom.  Ctrl+C para cerrar."
echo

exec ros2 launch ugv_bridge flipper.launch.py \
    "${MODO_ARGS[@]}" \
    imu_fuente:=pi3hat_real \
    use_ekf:=true \
    use_foxglove:=true \
    "$@"
