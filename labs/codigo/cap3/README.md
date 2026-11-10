# Laboratorio 4 — Capítulo 3: Movimiento, velocidad y trayectorias en ROS 2

Código reconstruido a partir del informe del laboratorio. Entorno de referencia: Ubuntu 24.04 LTS, ROS 2 Jazzy y Python 3.12.

## Contenido

- Actividad 1: integración discreta de orientación con SciPy.
- Actividades 2 y 3: simulador de IMU e integrador que publica TF2.
- Actividades 4 y 5: trayectorias quíntica y trapezoidal con Robotics Toolbox for Python.
- Actividad 6: interpolación de orientación SLERP.
- Actividad 7: visualización del frame `imu_link` en RViz2.

## 1. Dependencias

```bash
sudo apt update
sudo apt install python3-numpy python3-scipy python3-matplotlib \
  python3-colcon-common-extensions ros-jazzy-ros-base \
  ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-rviz2
```

Para las trayectorias, instala `roboticstoolbox-python` en un entorno Python compatible. La instalación de Robotics Toolbox puede requerir dependencias de compilación y variar según la versión de Python:

```bash
python3 -m pip install --user roboticstoolbox-python
```

Si el sistema bloquea instalaciones globales por PEP 668, utiliza un entorno virtual. Para evitar conflictos con los paquetes de ROS, conserva disponibles los paquetes del sistema o instala las dependencias ROS mediante apt.

## 2. Actividad 1: integración de orientación

Desde la carpeta `cap3`:

```bash
cd ~/robotica-man4101136/labs/codigo/cap3
python3 python/a1_integracion_orientacion.py
```

El script ejecuta:
- 100 iteraciones con `omega = [0, 0, 1] rad/s` y `dt = 0.01 s`.
- 200 iteraciones con `omega = [0.1, 0.2, 0.3] rad/s` y `dt = 0.01 s`.

## 3. Actividades 2 y 3: nodos ROS 2

Compila el workspace:

```bash
cd ~/robotica-man4101136/labs/codigo/cap3/ros2_cap3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Terminal 1 — simulador:

```bash
ros2 run cap3_imu imu_simulator
```

Terminal 2 — integrador:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robotica-man4101136/labs/codigo/cap3/ros2_cap3_ws
source install/setup.bash
ros2 run cap3_imu imu_integrator
```

Verificar el tópico:

```bash
ros2 topic echo /imu
ros2 topic hz /imu
```

Verificar TF2:

```bash
ros2 run tf2_ros tf2_echo world imu_link
ros2 run tf2_tools view_frames
```

Para cambiar la velocidad angular y probar los otros ejes:

```bash
ros2 run cap3_imu imu_simulator --ros-args \
  -p angular_velocity_x:=0.1 \
  -p angular_velocity_y:=0.2 \
  -p angular_velocity_z:=0.3
```

## 4. Actividades 4 y 5: perfiles temporales

```bash
python3 python/a4_trayectoria_quintica.py
python3 python/a5_trayectoria_trapezoidal.py
```

Ambos scripts generan un movimiento de 0 a 1 rad durante 2 segundos, con muestreo de 0.01 s, y muestran posición, velocidad y aceleración. Los valores máximos dependen de los parámetros de trayectoria y de la versión de Robotics Toolbox.

**Importante:** el informe registra como resultados de su ejecución 1.8734 rad/s y 5.7680 rad/s² para la quíntica, y 1.5000 rad/s y 4.5000 rad/s² para la trapezoidal. El código no fuerza esos valores: calcula las curvas con la biblioteca para los parámetros declarados.

## 5. Actividad 6: SLERP

```bash
python3 python/a6_interpolacion_slerp.py
```

Interpola cinco orientaciones entre 0° y 90° alrededor del eje Z.

## 6. Actividad 7: RViz2

Con los nodos `imu_simulator` e `imu_integrator` ejecutándose:

```bash
rviz2 -d rviz/imu_visualization.rviz
```

El archivo de configuración establece `world` como marco fijo y activa la visualización TF y Axes.

## Notas

- Los cuaterniones se representan en el orden `[qx, qy, qz, qw]`, que coincide con los campos de `geometry_msgs/msg/Quaternion`.
- El integrador usa la marca de tiempo de cada mensaje IMU para calcular `dt`.
- La traslación de `imu_link` se mantiene en cero: el informe implementa integración de orientación, no estimación de posición.
- El informe no especifica en el texto los parámetros numéricos de los perfiles de trayectoria. Se usa un movimiento demostrativo de 0 a 1 rad en 2 s; ajusta esos valores si el notebook original utilizó otros.
