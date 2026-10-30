#!/bin/bash
# Renderiza los HTML traducidos a PDF con weasyprint
set -e
cd "$(dirname "$0")"
VP=/tmp/claude-1000/-home-benja-Proyectos-TESIS/e9262d89-04eb-456a-bb09-dfc7c98a3e7b/scratchpad/.venv/bin/weasyprint
for f in "${@:-*.html}"; do
  base=$(basename "$f" .html)
  "$VP" "$f" "../$base.pdf" && echo "OK  ../$base.pdf"
done
