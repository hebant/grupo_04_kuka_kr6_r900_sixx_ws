#!/usr/bin/env bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=============================================="
echo "    VERIFICACIÓN DEL ENTORNO ROS 2 - GRUPO 04"
echo "=============================================="

if [ ! -f "$DIR/install/setup.bash" ]; then
  echo "[!] Workspace no compilado. Ejecuta primero ./instalar.sh o ./recompilar.sh"
  exit 1
fi

source "$DIR/entorno.sh"

echo "[✓] RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo
echo "--- Paquetes del grupo instalados ---"
ros2 pkg list | grep -E "grupo04|kuka" || echo "No se encontraron paquetes."

echo
echo "--- Nodos activos ---"
ros2 node list 2>/dev/null || echo "No hay nodos activos en este momento."

echo
echo "--- Tópicos activos ---"
ros2 topic list 2>/dev/null || echo "No hay tópicos activos en este momento."

echo "=============================================="
