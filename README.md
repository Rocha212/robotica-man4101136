# Robótica — MAN-4101136-1-2026-03

Repositorio del curso de Robótica: laboratorios y proyecto final.

**Integrantes:** Lina Robsana Jamioy Tisoy, Tomás Rocha Castaño
**Profesor:** Jaime Enrique Arango Castro

## Estructura

├── labs/
│ ├── codigo/
│ │ ├── cap5/ Navegación reactiva y planeación
│ │ ├── cap6/ Odometría, EKF, landmarks y SLAM
│ │ └── ros2/ Paquete localization_lab (Cartographer)
│ └── informes/
│ ├── cap5-navegacion/
│ └── cap6-slam/
└── proyecto_final/ Seguidor de línea con pinza y elevador


## Laboratorios

### Capítulo 5 — Navegación
| Parte | Archivo | Tema |
|-------|---------|------|
| A | `a1_braitenberg.py`, `a1_iter600.py` | Vehículos de Braitenberg |
| B | `b1_bug2.py`, `b2_bug2_escenario2.py` | Algoritmo Bug2 |
| C | `c1_astar.py` | Planeación con A* |
| D | `d1_distance_transform.py` | Distance transform |
| E | `e2_reactive_navigator.py` | Navegador reactivo en ROS2 + Gazebo + RViz |

Informe: `labs/informes/cap5-navegacion/informe.pdf`

### Capítulo 6 — Localización y SLAM
| Parte | Archivo | Tema |
|-------|---------|------|
| 1 | `parte1_odometria.py` | Odometría |
| 2 | `parte2_ekf.py` | Filtro de Kalman extendido (EKF) |
| 3 | `parte3_landmarks.py` | Localización con landmarks |
| 4 | `parte4_slam.py` | SLAM |
| 5 | `ros2/localization_lab` | SLAM con Cartographer en ROS2 |

Verificación del entorno: `labs/codigo/cap6/verificar_entorno.sh`
Informe: `labs/informes/cap6-slam/`

## Proyecto final

Seguidor de línea que transporta una caja de A a B usando una pinza (eje X) y un elevador (eje Y).

- **Fase 1 (simulación en Python):** modelo cinemático, sensores, PID de línea, A*, Pure Pursuit, misión completa y repetibilidad.
- **Fase 2 (ROS2):** en progreso (`proyecto_final/codigo/ros2/`).

Detalles, capturas y gráficas en [`proyecto_final/README.md`](proyecto_final/README.md).

## Ejecutar el código

Requisitos: Ubuntu 24.04, ROS2 Jazzy, Python 3.

```bash
# Simulaciones Python (ejemplo)
cd proyecto_final/codigo/python
python3 p6_mision_completa.py

# Paquete ROS2 (ejemplo)
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch localization_lab cartographer.launch.py
```
