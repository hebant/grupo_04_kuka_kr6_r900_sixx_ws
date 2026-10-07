#!/usr/bin/env bash
set -eo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$DIR"
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install
source install/setup.bash
echo "Recompilación completada exitosamente."
