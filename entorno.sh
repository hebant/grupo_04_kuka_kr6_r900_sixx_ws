#!/usr/bin/env bash
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
elif [ -f "$HOME/grupo_04_kuka_kr6_r900_sixx_ws/install/setup.bash" ]; then
  source "$HOME/grupo_04_kuka_kr6_r900_sixx_ws/install/setup.bash"
fi
