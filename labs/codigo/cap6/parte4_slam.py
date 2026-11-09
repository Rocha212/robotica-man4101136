from roboticstoolbox import *
import numpy as np
import matplotlib.pyplot as plt
from math import pi

# Covarianza de odometría
V = np.diag([0.02, np.deg2rad(0.5)])**2

# Robot
robot = Bicycle(covar=V, animation=None)
robot.control = RandomPath(workspace=10)

# Mapa real (solo para graficar; el EKF SLAM no lo usa)
mapa = LandmarkMap(20, workspace=10)

# Sensor
W = np.diag([0.1, np.deg2rad(1)])**2
sensor = RangeBearingSensor(
    robot=robot,
    map=mapa,
    covar=W,
    angle=[-pi/2, pi/2],
    range=4
)

# Covarianza inicial
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)])**2

# EKF SLAM: no conoce el mapa
slam = EKF(
    robot=(robot, V),
    P0=P0,
    sensor=(sensor, W),
    map=None
)

# Ejecutar simulación
slam.run(T=40)

# Trayectorias y mapa
plt.figure(figsize=(10, 8))
mapa.plot()
robot.plot_xy(color='blue', label='Trayectoria real')
slam.plot_xy(color='red', label='Trayectoria estimada')
slam.plot_ellipse()
plt.title('SLAM usando EKF')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.savefig('/home/tomas/lab_cap6/capturas/cap6_parte4_slam_trayectoria.png', dpi=150)

# Incertidumbre
plt.figure(figsize=(8, 6))
plt.plot(slam.get_t(), slam.get_Pnorm(), linewidth=2)
plt.xlabel('Tiempo [s]')
plt.ylabel('Incertidumbre')
plt.title('Evolución de incertidumbre en SLAM')
plt.grid(True)
plt.savefig('/home/tomas/lab_cap6/capturas/cap6_parte4_slam_incertidumbre.png', dpi=150)

plt.show()
