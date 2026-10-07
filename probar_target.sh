#!/usr/bin/env bash
set -eo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$DIR/entorno.sh"

X="${1:-0.60}"
Y="${2:-0.20}"
Z="${3:-0.50}"

echo "Publicando objetivo cartesiano en /target:"
echo "  X: $X m"
echo "  Y: $Y m"
echo "  Z: $Z m"

ros2 topic pub /target geometry_msgs/msg/Point "{x: $X, y: $Y, z: $Z}" --once
