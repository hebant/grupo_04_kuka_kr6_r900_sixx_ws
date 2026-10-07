# Cinemática Directa e Inversa del Robot KUKA KR 6 R900 sixx con ROS 2 Jazzy

**Materia:** IMT-342 Robótica – Universidad Católica Boliviana "San Pablo"  
**Docente:** Ing. Carlos Daniel Aguilar Mancachi  
**Grupo:** Grupo 04  
**Integrantes:**
- Heber Poma
- Sebastian Ibañez

---

## 1. Identificación del Proyecto y del Robot

- **Fabricante:** KUKA Roboter GmbH
- **Modelo:** KR 6 R900 sixx (Familia KR AGILUS)
- **Número de grados de libertad (DOF):** 6 articulaciones rotacionales (revolutas)
- **Marco base:** `base_link`
- **Efector final:** `tool0`
- **Unidades:** Sistema Internacional (metros y radianes)
- **Paquetes ROS 2 oficiales utilizados:** `kuka_agilus_support` y `kuka_resources` del repositorio [kuka_robot_descriptions](https://github.com/kroshu/kuka_robot_descriptions.git).

---

## 2. Objetivo del Trabajo

1. Obtener la cadena cinemática y la asignación sistemática de marcos de referencia del manipulador KUKA KR 6 R900 sixx mediante la convención **Denavit–Hartenberg estándar**.
2. Formular e implementar el nodo de **Cinemática Directa (FK)** en ROS 2 Jazzy que calcule la posición cartesiana $(x, y, z)$ del efector final a partir de los estados articulares leídos en `/joint_states`.
3. Deducir el **Jacobiano posicional** $J_v(q) \in \mathbb{R}^{3 \times 6}$ de forma analítica y geométrica.
4. Implementar el nodo de **Cinemática Inversa (IK)** iterativa utilizando la pseudoinversa de Moore-Penrose $J^+$, suscribiéndose al tópico `/target` mediante mensajes `geometry_msgs/msg/Point` y publicando la solución articular en `/joint_states`.
5. Visualizar y validar la solución en tiempo real en **RViz2** con el modelo URDF/XACRO oficial.

---

## 3. Software y Versiones Requeridas

- **Sistema Operativo:** Ubuntu 24.04 LTS (Noble Numbat)
- **Middleware:** ROS 2 Jazzy Jalisco
- **Middleware RMW:** `rmw_cyclonedds_cpp`
- **Lenguaje:** Python 3.12+
- **Librerías principales:** NumPy, SymPy
- **Herramientas de ROS 2:** `rviz2`, `robot_state_publisher`, `joint_state_publisher_gui`, `xacro`, `colcon`, `rosdep`, `vcstool`

---

## 4. Estructura del Repositorio


```text
grupo_04_kuka_kr6_r900_sixx_ws/
├── .gitignore
├── dependencias.repos
├── dependencies.repos
├── requirements.txt
├── entorno.sh
├── instalar.sh
├── abrir.sh
├── ejecutar_fk.sh
├── ejecutar_ik.sh
├── probar_target.sh
├── recompilar.sh
├── verificar.sh
├── INSTRUCCIONES.txt
├── README.md
├── docs/
│   └── guia_primer_parcial_practico_IMT342.pdf
└── src/
    ├── grupo04_kuka_kr6_bringup/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── launch/
    │       └── display.launch.py
    └── grupo04_robot_kinematics/
        ├── package.xml
        ├── setup.cfg
        ├── setup.py
        ├── resource/
        │   └── grupo04_robot_kinematics
        └── grupo04_robot_kinematics/
            ├── __init__.py
            ├── fk_node.py
            └── ik_node.py
```

> **Nota:** La carpeta del modelo oficial externo (`src/kuka_robot_descriptions/`) y las carpetas de compilación (`build/`, `install/`, `log/`) están excluidas del control de versiones mediante `.gitignore` y se configuran automáticamente al instalar.

---

## 5. Procedimiento de Instalación

En una computadora limpia con Ubuntu 24.04 y ROS 2 Jazzy:

### Paso 1: Clonar el repositorio

```bash
git clone https://github.com/<TU_USUARIO>/grupo_04_kuka_kr6_r900_sixx_ws.git
cd grupo_04_kuka_kr6_r900_sixx_ws
```

### Paso 2: Ejecutar el instalador automatizado

```bash
chmod +x *.sh
./instalar.sh
```

El script `./instalar.sh` realiza automáticamente:
1. Comprobación de ROS 2 Jazzy en `/opt/ros/jazzy`.
2. Instalación de paquetes necesarios de `apt` y librerías (`numpy`, `sympy`, `cyclonedds`, herramientas URDF/Xacro).
3. Descarga del modelo oficial de KUKA (`kuka_agilus_support` y `kuka_resources`) mediante sparse-checkout.
4. Resolución de dependencias mediante `rosdep`.
5. Compilación completa con `colcon build --symlink-install`.
6. Configuración de variables de entorno portables en `entorno.sh`.

---

## 6. Procedimiento Manual Alternativo de Compilación

Si se prefiere realizar el proceso de forma manual paso a paso:

```bash
# 1. Recuperar modelo KUKA oficial
mkdir -p src
vcs import src < dependencias.repos

# 2. Configurar entorno y dependencias
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

# 3. Compilar
colcon build --symlink-install
source install/setup.bash
```

---

## 7. Instrucciones de Ejecución

Cada terminal debe tener cargado el entorno del workspace:
```bash
source entorno.sh
```
*(O de forma equivalente: `source /opt/ros/jazzy/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source install/setup.bash`)*

### 7.1. Lanzar el robot en RViz2

Opción directa con atajo:
```bash
./abrir.sh
```

Opción con comando ROS 2:
```bash
ros2 launch grupo04_kuka_kr6_bringup display.launch.py
```

Se abrirán:
- **RViz2**: visualización 3D del robot con sus eslabones y mallas oficiales.
- **Joint State Publisher GUI**: ventana con controles deslizantes para las articulaciones de `joint_1` a `joint_6`.

---

### 7.2. Ejecutar el nodo de Cinemática Directa (FK)

En una **segunda terminal**:
```bash
./ejecutar_fk.sh
```
*(O manualmente: `ros2 run grupo04_robot_kinematics fk_node`)*

El nodo `fk_node`:
- Se suscribe a `/joint_states`.
- Extrae y ordena los valores de $[q_1, q_2, q_3, q_4, q_5, q_6]$.
- Evalúa las matrices Denavit–Hartenberg para obtener $T_0^6$.
- Imprime continuamente en consola la posición $(x, y, z)$ del efector final.

---

### 7.3. Ejecutar el nodo de Cinemática Inversa (IK)

> **IMPORTANTE (Sección VIII-D de la guía):**  
> La ventana `Joint State Publisher GUI` publica continuamente en `/joint_states`. Para que el nodo de IK pueda posicionar el robot en RViz2 sin que sus mensajes sean sobreescritos, **cierre la ventana pequeña de Joint State Publisher GUI** antes de probar la cinemática inversa.

En una **segunda terminal** (con la ventana GUI cerrada):
```bash
./ejecutar_ik.sh
```
*(O manualmente: `ros2 run grupo04_robot_kinematics ik_node`)*

---

### 7.4. Enviar un objetivo cartesiano al robot

En una **tercera terminal**, envíe una coordenada cartesiana $(x, y, z)$ al tópico `/target`:

Opción con atajo:
```bash
./probar_target.sh 0.60 0.20 0.50
```

Opción con comando oficial `ros2 topic pub`:
```bash
ros2 topic pub /target geometry_msgs/msg/Point "{x: 0.60, y: 0.20, z: 0.50}" --once
```

**Flujo en tiempo de ejecución:**
```text
/target (Point: x,y,z)
      │
      ▼
   ik_node (resuelve con pseudoinversa)
      │
      ▼
/joint_states
   ├──► robot_state_publisher ──► RViz2 (actualiza postura 3D)
   └──► fk_node ──► Verificación numérica de la posición alcanzada
```

---



## 8. Tópicos y Mensajes ROS 2

| Tópico | Tipo de Mensaje | Función |
| :--- | :--- | :--- |
| `/target` | `geometry_msgs/msg/Point` | Recibe las coordenadas deseadas $(x_d, y_d, z_d)$ para la cinemática inversa. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Publica/escucha los ángulos $[q_1, \dots, q_6]$ para actualizar la postura en RViz2. |
| `/robot_description` | `std_msgs/msg/String` | Modelo URDF/XACRO cargado del robot. |

---


## 9. Scripts de Soporte Rápido

- `./instalar.sh`: Instalación automatizada completa en un solo comando.
- `./abrir.sh`: Lanza RViz2 y la visualización del KUKA KR6 R900 sixx.
- `./ejecutar_fk.sh`: Ejecuta el nodo de cinemática directa.
- `./ejecutar_ik.sh`: Ejecuta el nodo de cinemática inversa.
- `./probar_target.sh <x> <y> <z>`: Publica coordenadas al tópico `/target` (por defecto: `0.60 0.20 0.50`).
- `./recompilar.sh`: Recompila el workspace con `colcon` en cualquier momento.
- `./verificar.sh`: Comprueba el estado de la compilación, tópicos y nodos activos.
