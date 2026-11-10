# Laboratorio 1 — Introducción a ROS 2: Mini INS

## Objetivo

Implementar un sistema mínimo de navegación inercial en ROS 2 Jazzy. El sistema recibe velocidad angular simulada en `/imu`, integra la orientación mediante cuaterniones y publica la transformación TF `world` → `imu_link`.

## Entorno

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Python 3
- Paquete ROS 2: `imu_orientation`

## Nodos

- `imu_simulator`: publica mensajes `sensor_msgs/msg/Imu` a 100 Hz.
- `imu_integrator`: integra la velocidad angular y publica la orientación como TF.

El simulador inicia con `wx=0`, `wy=0` y `wz=0.5 rad/s`.

## Dependencias

En una instalación ROS 2 Jazzy con `rosdep` disponible:

```bash
sudo apt update
sudo apt install python3-numpy python3-scipy
```

## Compilación

Desde la carpeta `intro_ros2`, que contiene el paquete `imu_orientation`:

```bash
cd ~/robotica-man4101136/labs/codigo/intro_ros2
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build
source install/setup.bash
```

Si `colcon` no está instalado:

```bash
sudo apt install python3-colcon-common-extensions
```

## Ejecución

Abrir dos terminales. En ambas cargar ROS y el workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robotica-man4101136/labs/codigo/intro_ros2
source install/setup.bash
```

Terminal 1 — simulador:

```bash
ros2 run imu_orientation imu_simulator
```

Terminal 2 — integrador:

```bash
ros2 run imu_orientation imu_integrator
```

## Visualización en RViz2

En otra terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robotica-man4101136/labs/codigo/intro_ros2
source install/setup.bash
rviz2
```

En RViz2:

1. Establecer `world` como **Fixed Frame**.
2. Agregar el display **TF**.
3. Agregar **Axes** y seleccionar `imu_link` como frame de referencia.
4. Observar la rotación alrededor del eje Z.

## Validaciones del laboratorio

### 1. Duplicar la velocidad angular en Z

Detener el simulador anterior y ejecutarlo con:

```bash
ros2 run imu_orientation imu_simulator --ros-args -p angular_velocity_z:=1.0
```

La velocidad angular en Z pasa de `0.5` a `1.0 rad/s`. La rotación debe observarse aproximadamente al doble de rapidez.

### 2. Cambiar el eje de rotación

Ejecutar el simulador con componentes en los tres ejes:

```bash
ros2 run imu_orientation imu_simulator --ros-args \
  -p angular_velocity_x:=0.2 \
  -p angular_velocity_y:=0.3 \
  -p angular_velocity_z:=0.5
```

El frame `imu_link` presenta una rotación compuesta tridimensional.

## Verificación del árbol TF

```bash
ros2 run tf2_tools view_frames
```

También se puede consultar la transformación en tiempo real:

```bash
ros2 run tf2_ros tf2_echo world imu_link
```

## Notas

- La primera muestra de IMU se utiliza para inicializar el tiempo; no se integra.
- La orientación se inicializa con la rotación identidad.
- El intervalo `dt` se calcula con el reloj de ROS.
- La traslación se mantiene en cero porque esta práctica estima únicamente orientación.
- Esta implementación reproduce el alcance del laboratorio; no incluye estimación de posición ni corrección de deriva mediante magnetómetro o acelerómetro.
