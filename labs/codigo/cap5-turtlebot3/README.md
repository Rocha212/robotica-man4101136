# Laboratorio 5 — TurtleBot3, ROS 2 y Gazebo

Paquete Python para probar tres controladores del capítulo 4 en TurtleBot3:
1. `go_to_goal`: ir a (2, 2) m.
2. `line_following`: seguir la recta `x - 2y + 4 = 0`.
3. `pure_pursuit`: recorrer una lista de waypoints.

## Entorno previsto

Ubuntu 24.04, ROS 2 Jazzy y TurtleBot3 Burger. La guía del capítulo 4 indica
`/odom` (`nav_msgs/msg/Odometry`) y `/cmd_vel` (`geometry_msgs/msg/TwistStamped`).
Comprueba el tipo real del tópico en tu instalación antes de ejecutar:

```bash
ros2 topic info /cmd_vel
ros2 topic info /odom
```

Si el tópico `/cmd_vel` de tu simulación usa `geometry_msgs/msg/Twist` en vez
de `TwistStamped`, este paquete requiere adaptar el tipo de mensaje antes de
ejecutarse.

## 1. Instalar dependencias

```bash
sudo apt update
sudo apt install ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-turtlebot3-msgs python3-colcon-common-extensions
```

Configurar el modelo Burger en cada terminal:

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

## 2. Construir el paquete

Desde la carpeta `ros2_ws`:

```bash
cd ~/robotica-man4101136/labs/codigo/cap5-turtlebot3/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Ejecutar Gazebo

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Si ese launch no existe en la instalación Jazzy, consulta los launch disponibles:

```bash
ros2 pkg prefix turtlebot3_gazebo
ros2 launch turtlebot3_gazebo --show-args empty_world.launch.py
```

Terminal 2: comprobar tópicos:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic info /odom
ros2 topic info /cmd_vel
```

## 4. Ejecutar cada controlador

En una terminal nueva, para cada prueba:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotica-man4101136/labs/codigo/cap5-turtlebot3/ros2_ws/install/setup.bash
ros2 run lab_robotica go_to_goal
```

Para seguimiento de línea:

```bash
ros2 run lab_robotica line_following
```

Para Pure Pursuit:

```bash
ros2 run lab_robotica pure_pursuit
```

Ejecuta un controlador a la vez. Para detenerlo, presiona `Ctrl+C`; el nodo publica velocidad cero al cerrarse.

## 5. Cambiar parámetros

Ejemplo para cambiar la meta:

```bash
ros2 run lab_robotica go_to_goal --ros-args -p goal_x:=1.0 -p goal_y:=1.5
```

Ejemplo para cambiar la recta:

```bash
ros2 run lab_robotica line_following --ros-args -p a:=1.0 -p b:=-2.0 -p c:=4.0
```

Ejemplo para cambiar lookahead:

```bash
ros2 run lab_robotica pure_pursuit --ros-args -p lookahead:=0.7
```

## 6. SLAM y navegación Nav2

La actividad pide probar SLAM y navegación siguiendo el tutorial oficial de
TurtleBot3. Los comandos exactos dependen de las versiones y paquetes instalados.
No se incluyen aquí como scripts automáticos para evitar asumir nombres de
launch que pueden cambiar entre distribuciones. Sigue el tutorial oficial:

https://docs.robotis.com/docs/systems/turtlebot3/simulation/gazebo_simulation

## Notas técnicas importantes

- `go_to_goal` sigue los parámetros de la guía: meta (2, 2), Kv=0.5,
  Kh=2.0 y tolerancia 0.05 m. Añade límites de velocidad para el Burger.
- `line_following` adapta el controlador de la guía del capítulo 4 al robot
  diferencial. Se usa la recta `x - 2y + 4 = 0`, con Kd=1 y Kh=2 como
  valores iniciales de prueba.
- `pure_pursuit` conserva la lógica de avance entre waypoints por radio
  lookahead de la implementación del capítulo 4. Es una adaptación educativa
  de waypoint pursuit, no la formulación geométrica completa de Pure Pursuit.
- Verifica el tipo de `/cmd_vel` en tu Gazebo. La guía del capítulo 4 usa
  `TwistStamped`; si el plugin publica/recibe `Twist`, hay que cambiar el
  tipo de mensaje.
- Los controladores se han preparado como código base; deben compilarse y
  probarse en el computador antes de registrar resultados en el informe.
