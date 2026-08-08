#!/usr/bin/env bash
# generar_urdf.sh — Regenera urdf/ugv.urdf a partir de urdf/ugv.urdf.xacro y
# config/geometria_robot.yaml.
#
# ¿Por qué existe un .urdf plano si los launch ya procesan el .xacro en caliente?
# Porque hay consumidores que NO procesan xacro: Foxglove Studio cargando el
# modelo desde archivo, visores online, check_urdf, y cualquier script del equipo
# que abra el URDF directo. Este comando los mantiene sincronizados.
#
# Uso (desde la raíz del repo o desde cualquier lado):
#     bash src/ugv_bridge/scripts/generar_urdf.sh
#
# No necesita colcon build previo: lee el YAML del árbol de fuentes.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
XACRO_IN="${PKG_DIR}/urdf/ugv.urdf.xacro"
YAML_IN="${PKG_DIR}/config/geometria_robot.yaml"
URDF_OUT="${PKG_DIR}/urdf/ugv.urdf"

if ! command -v xacro >/dev/null 2>&1; then
    echo "ERROR: no se encontró 'xacro'." >&2
    echo "  ¿Hiciste 'source /opt/ros/jazzy/setup.bash'?" >&2
    echo "  Si falta el paquete:  sudo apt install ros-jazzy-xacro" >&2
    exit 1
fi

echo "Geometría : ${YAML_IN}"
echo "Plantilla : ${XACRO_IN}"

xacro "${XACRO_IN}" "geometria:=${YAML_IN}" -o "${URDF_OUT}"

echo "Generado  : ${URDF_OUT}"

if command -v check_urdf >/dev/null 2>&1; then
    check_urdf "${URDF_OUT}" >/dev/null && echo "check_urdf : OK (árbol de links válido)"
fi

echo
echo "Recuerda: 'colcon build' para que el URDF nuevo llegue a install/, y commitea"
echo "el .urdf junto con el YAML para que no se separen."
