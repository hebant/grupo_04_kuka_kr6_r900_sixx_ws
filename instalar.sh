#!/usr/bin/env bash
set -eo pipefail

echo "============================================================"
echo "   INSTALACIÓN AUTOMATIZADA - GRUPO 04 KUKA KR 6 R900 sixx"
echo "   IMT-342 Robótica | Universidad Católica Boliviana"
echo "============================================================"

# 1. Comprobar que ROS 2 Jazzy este instalado
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo
  echo "ERROR: ROS 2 Jazzy no está instalado en /opt/ros/jazzy."
  echo "Por favor instale ROS 2 Jazzy antes de continuar:"
  echo "  https://docs.ros.org/en/jazzy/Installation.html"
  exit 1
fi

echo
echo "[1/6] ROS 2 Jazzy detectado."
source /opt/ros/jazzy/setup.bash

# 2. Instalar paquetes de sistema y herramientas
echo
echo "[2/6] Instalando dependencias del sistema y ROS 2..."
sudo apt update
sudo apt install -y \
  git \
  build-essential \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-numpy \
  python3-sympy \
  python3-pip \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-urdf \
  ros-jazzy-urdfdom \
  ros-jazzy-urdf-tutorial \
  ros-jazzy-rmw-cyclonedds-cpp \
  liburdfdom-tools

# 3. Inicializar y actualizar rosdep si no esta configurado
echo
echo "[3/6] Verificando rosdep..."
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update || true

# 4. Obtener el modelo KUKA KR6 R900 sixx (kuka_robot_descriptions)
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$WS_DIR/src"

echo
echo "[4/6] Verificando paquetes de descripcion KUKA..."
if [ ! -d "$WS_DIR/src/kuka_robot_descriptions" ]; then
  echo "Descargando modelo oficial de KUKA (kuka_agilus_support y kuka_resources)..."
  git clone --depth 1 --branch master --filter=blob:none --sparse https://github.com/kroshu/kuka_robot_descriptions.git "$WS_DIR/src/kuka_robot_descriptions"
  cd "$WS_DIR/src/kuka_robot_descriptions"
  git sparse-checkout set kuka_resources kuka_agilus_support
  cd "$WS_DIR"
else
  echo "El modelo KUKA ya se encuentra presente en src/kuka_robot_descriptions."
fi

# 5. Instalar dependencias faltantes via rosdep
echo
echo "[5/6] Resolviendo dependencias de paquetes ROS 2..."
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy || true

# 6. Compilar workspace
echo
echo "[6/6] Compilando workspace con colcon..."
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install

# Asegurar permisos ejecutables para todos los scripts
chmod +x "$WS_DIR"/*.sh 2>/dev/null || true

echo
echo "============================================================"
echo "             INSTALACIÓN COMPLETADA CON ÉXITO"
echo "============================================================"
echo "Workspace ubicado en: $WS_DIR"
echo
echo "Para comenzar a trabajar:"
echo "  source entorno.sh"
echo
echo "Comandos disponibles:"
echo "  1. Abrir robot en RViz2:  ./abrir.sh"
echo "  2. Ejecutar FK:           ./ejecutar_fk.sh"
echo "  3. Ejecutar IK:           ./ejecutar_ik.sh"
echo "  4. Enviar objetivo IK:    ./probar_target.sh"
echo "  5. Recompilar cambios:    ./recompilar.sh"
echo "  6. Verificar entorno:     ./verificar.sh"
echo "============================================================"
