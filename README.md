# Robótica – MAN-4101136-1-2026-03

Repositorio de los laboratorios y del proyecto final del curso de Robótica, Universidad Nacional de Colombia, sede Manizales (2026-2).

**Integrantes:** Lina Robsana Jamioy Tisoy y Tomás Rocha Castaño
**Profesor:** Jaime Enrique Arango Castro

## Estructura

```
robotica-man4101136/
├── labs/
│   ├── codigo/          # Código de cada laboratorio
│   │   ├── cap5/        # Navegación: Braitenberg, Bug2, A*, Distance Transform, Nav2
│   │   ├── cap6/        # Localización: odometría, EKF, landmarks, SLAM en Python
│   │   └── ros2/
│   │       └── localization_lab/   # Paquete ROS 2: Cartographer con TurtleBot3
│   └── informes/        # Informes en PDF, capturas y mapas
│       ├── cap5-navegacion/
│       └── cap6-slam/
├── informes/            # Informes de capítulos 1 a 4
├── codigo/              # Código de capítulos 2 a 4
├── proyecto_final/      # Proyecto final: seguidor de línea con caja
└── README.md
```

## Laboratorios

| Capítulo | Tema | Código | Informe | Video |
|---|---|---|---|---|
| Cap. 2 | Pose y cuaterniones en ROS 2 | `codigo/cap2` | `informes/cap2-posicion-orientacion` | [YouTube](https://youtu.be/64t5pewBsx8) |
| Cap. 3 | Movimiento, velocidad y trayectorias | `codigo/cap3` | `informes/cap3-tiempo-movimiento` | [YouTube](https://youtu.be/LusyeibyE-I) |
| Cap. 4 | Cinemática (bicicleta, TurtleBot3) | `codigo/cap4` | `informes/cap4-cinematica` | [YouTube](https://youtu.be/my6pogCSxcU), [YouTube](https://youtu.be/BL9vo8aDkw) |
| Cap. 5 | Navegación (Braitenberg, Bug2, A*, Nav2) | `labs/codigo/cap5` | `labs/informes/cap5-navegacion` | [YouTube](https://youtu.be/go3Mom6UoRg), [YouTube](https://youtu.be/YQFfcxXbcW4) |
| Cap. 6 | Localización y SLAM | `labs/codigo/cap6`, `labs/codigo/ros2/localization_lab` | `labs/informes/cap6-slam` | [YouTube](https://youtu.be/rzjI1KvMs-s) |

## Capítulo 6: SLAM con Cartographer

Mapeo de la casa `turtlebot3_house` con un TurtleBot3 burger en Gazebo Harmonic, Cartographer y RViz2, en ROS 2 Jazzy.

**Hallazgo principal:** el mapa duplicaba paredes porque Gazebo y Cartographer publicaban el mismo marco `odom → base_footprint`. La solución fue que Cartographer use la odometría y publique solo `map → odom` (ver `config/cartographer.lua`).

### Uso del paquete

```bash
mkdir -p ~/ros2_ws/src
cp -r labs/codigo/ros2/localization_lab ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Luego, en terminales separadas:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=-2.0142 y_pose:=0.5000
ros2 launch localization_lab cartographer.launch.py
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -p use_sim_time:=true
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

Para guardar el mapa:

```bash
ros2 run nav2_map_server map_saver_cli -f mapa_casa
```

El nodo `explorador` (`explorador_lento.py`) es un controlador automático de prueba. No se usó en el mapa final.

### Entorno

- Ubuntu 24.04.5 LTS
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic 8.15.0
- TurtleBot3 (burger), Cartographer, Nav2
- Python 3.12.3, numpy 1.26.4, matplotlib 3.6.3

## Entorno general de trabajo

- Ubuntu 24.04 LTS
- Python 3.12
- ROS 2 Jazzy Jalisco (capítulos 2, 3, 4, 5 parte E y 6 parte 5)
- Gazebo Harmonic

## Autores

- Lina Robsana Jamioy Tisoy
- Tomás Rocha Castaño
