# Laboratorio 3 — Capítulo 2: Representación de pose y cuaterniones

Implementación de las seis actividades documentadas en el informe del laboratorio, sobre Ubuntu 24.04 LTS y ROS 2 Jazzy.

## Estructura

- `python/`: actividades 1, 2, 5 y 6, ejecutables sin ROS 2.
- `ros2_ws/src/pose_pub/`: actividad 3, publicador del tópico `/pose`.
- `ros2_ws/src/tf_broadcaster/`: actividad 4, transformación dinámica `world` → `robot`.

## 1. Dependencias

```bash
sudo apt update
sudo apt install python3-numpy python3-scipy python3-matplotlib \
  ros-jazzy-ros-base ros-jazzy-tf2-ros ros-jazzy-rviz2 \
  python3-colcon-common-extensions
```

Carga ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

## 2. Actividades Python

Desde la carpeta `cap2`:

```bash
cd ~/robotica-man4101136/labs/codigo/cap2
```

Ejecuta cada actividad:

```bash
python3 python/a1_conversion_rotaciones.py
python3 python/a2_no_conmutatividad.py
python3 python/a5_error_orientacion.py
python3 python/a6_slerp.py
```

La actividad 2 imprime la diferencia angular calculada por SciPy para las dos composiciones. La cifra depende de la definición de composición y de la métrica usada; se muestra el resultado calculado por el programa, sin fijar artificialmente el valor escrito en el informe.

## 3. Compilar los paquetes ROS 2

```bash
cd ~/robotica-man4101136/labs/codigo/cap2/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## 4. Actividad 3: publicador de pose

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robotica-man4101136/labs/codigo/cap2/ros2_ws
source install/setup.bash
ros2 run pose_pub pose_publisher
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /pose
```

Se publica una posición `(1.0, 2.0, 0.0)` y una orientación de 90 grados alrededor de Z cada 0.5 segundos.

## 5. Actividad 4: TF2

Ejecuta:

```bash
ros2 run tf_broadcaster tf_broadcaster
```

En otra terminal verifica:

```bash
ros2 run tf2_ros tf2_echo world robot
ros2 run tf2_tools view_frames
```

Para visualizar en RViz2:

```bash
rviz2
```

Configura `world` como Fixed Frame y agrega los displays `TF` y `Axes`. El frame hijo es `robot`.

## 6. Actividades 5 y 6

Las actividades de error de orientación y SLERP se ejecutan con los scripts Python:

```bash
python3 python/a5_error_orientacion.py
python3 python/a6_slerp.py
```

## Notas de implementación

- SciPy entrega cuaterniones en orden `[x, y, z, w]`, igual que los campos de `geometry_msgs`.
- En la actividad 4, el ángulo aumenta 0.1 radianes por publicación, con periodo de 0.1 s, siguiendo el código descrito en el informe.
- Este repositorio reconstruye las actividades y parámetros visibles en el informe entregado. No incluye notebooks originales ni material externo que no fue adjuntado.
