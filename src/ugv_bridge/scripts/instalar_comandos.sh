#!/usr/bin/env bash
# instalar_comandos.sh — Deja `compilar_real` y `compilar_simu` disponibles como
# comandos en la Raspberry.
#
#     bash src/ugv_bridge/scripts/instalar_comandos.sh
#
# Crea dos ENLACES SIMBOLICOS en ~/.local/bin apuntando a tesis_lanzar.sh, que
# decide el modo segun el nombre con el que se le invoca. Al ser enlaces y no
# copias, editar el script del repositorio actualiza los dos comandos sin
# reinstalar nada.
#
# No necesita root: ~/.local/bin es del usuario.
set -euo pipefail

DESTINO="$HOME/.local/bin"
ORIGEN="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/tesis_lanzar.sh"

[ -f "$ORIGEN" ] || { echo "No encuentro $ORIGEN" >&2; exit 1; }

mkdir -p "$DESTINO"
chmod +x "$ORIGEN"

for nombre in compilar_real compilar_simu; do
    ln -sf "$ORIGEN" "$DESTINO/$nombre"
    echo "  $DESTINO/$nombre  ->  $ORIGEN"
done

# ~/.local/bin no siempre esta en el PATH de shells no interactivos.
if ! echo "$PATH" | tr ':' '\n' | grep -qx "$DESTINO"; then
    LINEA='export PATH="$HOME/.local/bin:$PATH"'
    if ! grep -qF "$LINEA" "$HOME/.bashrc" 2>/dev/null; then
        printf '\n# Comandos de la tesis (compilar_real / compilar_simu)\n%s\n' "$LINEA" >> "$HOME/.bashrc"
        echo "  PATH: agregada la linea a ~/.bashrc"
    fi
    echo "  OJO: abre una terminal nueva (o corre: export PATH=\"\$HOME/.local/bin:\$PATH\")"
fi

echo
echo "Listo. Desde cualquier directorio:"
echo "    compilar_simu    motores simulados + IMU fisica real"
echo "    compilar_real    motores reales por el pi3hat + IMU fisica real"
