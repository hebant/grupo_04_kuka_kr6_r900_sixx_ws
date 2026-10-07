#!/usr/bin/env bash
set -eo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f "$DIR/install/setup.bash" ]; then
  echo "ERROR: El workspace no está compilado. Ejecuta primero ./instalar.sh o ./recompilar.sh"
  exit 1
fi

source "$DIR/entorno.sh"
echo "Iniciando nodo de Cinemática Directa (FK)..."
ros2 run grupo04_robot_kinematics fk_node "$@"
