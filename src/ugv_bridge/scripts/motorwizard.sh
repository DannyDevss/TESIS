#!/usr/bin/env bash
# motorwizard.sh — Abre el Motor Wizard de SteadyWin (Windows) bajo Wine.
#
#     motorwizard            # instalado como comando (ver instalar_comandos.sh)
#     bash src/ugv_bridge/scripts/motorwizard.sh
#
# CORRER EN EL PC, no en la Raspberry: el Wizard es un .exe de 32 bits y la Pi es
# ARM. Tampoco sirve dentro de rosdev (no hay Wine ni acceso al USB del host).
#
# El Wizard es un VISOR, no la via de configuracion de la tesis. Solo habla USB,
# motor a motor, y no toca el bus CAN. Lo que hay que grabar en cada driver
# (node_id, enable_can_a, baud_rate, limites) se hace con odrivetool, que corre
# nativo: ver COMO_EJECUTAR.md, "Configurar los 8 motores por USB".
#
# El paso de Zadig que pide el manual (seccion 3.1.2) NO aplica en Linux: es el
# instalador del driver USB de Windows. Aqui el permiso lo da la regla udev
# /etc/udev/rules.d/91-odrive.rules, la misma que ya usa odrivetool.
set -euo pipefail

PREFIJO="${WINEPREFIX_MOTORWIZARD:-$HOME/.local/share/wineprefixes/motorwizard}"
APP="$PREFIJO/drive_c/Program Files (x86)/SteadyWin/Motor Wizard"
EXE="$APP/motorwizard.exe"

command -v wine >/dev/null || { echo "[motorwizard] No hay wine instalado." >&2; exit 1; }

if [ ! -f "$EXE" ]; then
    echo "[motorwizard] No encuentro $EXE" >&2
    echo "[motorwizard] Para instalarlo (una sola vez):" >&2
    echo "    curl -L -o /tmp/motorwizard.exe https://bl.cyberbeast.cn/actuator/steadywin_motorwizard.exe" >&2
    echo "    WINEPREFIX=$PREFIJO wineboot -i" >&2
    echo "    WINEPREFIX=$PREFIJO wine /tmp/motorwizard.exe /VERYSILENT /SUPPRESSMSGBOXES /NORESTART" >&2
    exit 1
fi

# Sin motor alimentado y conectado por Type-C, el Wizard abre pero no muestra una
# lectura de tension valida. Se avisa aqui para no confundir eso con un fallo.
if command -v lsusb >/dev/null && ! lsusb | grep -qi '1209:0d32'; then
    echo "[motorwizard] AVISO: no veo ningun driver CyberBeast (1209:0d32) en el USB."
    echo "[motorwizard]        Alimenta por XT30 ANTES de conectar el Type-C; el USB"
    echo "[motorwizard]        es solo consola y no mueve el motor."
fi

# El Wizard lista su carpeta de registros al arrancar y NO la crea: si no existe
# revienta con 'PathNotFoundException: ... MotorWizard\datalog'. Dentro del
# prefijo, Documents es un enlace a ~/Documentos, asi que los registros acaban
# ahi, que es donde conviene que esten para la tesis.
mkdir -p "$PREFIJO/drive_c/users/$USER/Documents/MotorWizard/datalog"

# WINEDEBUG=-all calla el ruido de Wine. Los avisos 'MESA-EGL: failed to create
# dri2 screen' (la NVIDIA de este equipo) y el error de 'direct_manipulation'
# (tactil/lapiz, que Wine no implementa) son inofensivos: la app arranca igual.
cd "$APP"
exec env WINEPREFIX="$PREFIJO" WINEDEBUG="${WINEDEBUG:--all}" wine motorwizard.exe "$@"
