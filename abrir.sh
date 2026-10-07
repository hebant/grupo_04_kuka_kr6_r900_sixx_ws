#!/usr/bin/env bash
set -eo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f "$DIR/install/setup.bash" ]; then
  echo "ERROR: El workspace no está compilado. Ejecuta primero ./instalar.sh o ./recompilar.sh"
  exit 1
fi

source "$DIR/entorno.sh"
echo "Lanzando visualización del robot KUKA KR 6 R900 sixx en RViz2..."
ros2 launch grupo04_kuka_kr6_bringup display.launch.py "$@"
