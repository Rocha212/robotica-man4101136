from roboticstoolbox import *
import numpy as np
import matplotlib.pyplot as plt
from math import pi

# Covarianza odometría
V = np.diag([0.02, np.deg2rad(0.5)])**2

# Robot
robot = Bicycle(covar=V, animation=None)
robot.control = RandomPath(workspace=10)

# Mapa de landmarks
mapa = LandmarkMap(20, workspace=10)

# Covarianza sensor de rango
W = np.diag([0.1, np.deg2rad(1)])**2

# Sensor de rango, tipo Lidar
sensor = RangeBearingSensor(
    robot=robot,
    map=mapa,
    covar=W,
    angle=[-pi/2, pi/2],
    range=4
)

# Covarianza inicial
P0 = np.diag([0.05, 0.05, np.deg2rad(0.5)])**2

# EKF
ekf = EKF(
    robot=(robot, V),
    P0=P0,
    map=mapa,
    sensor=(sensor, W)
)

# Simulación
ekf.run(T=40)

# Trayectorias
plt.figure(figsize=(8, 6))
mapa.plot()
robot.plot_xy(color='blue')
ekf.plot_xy(color='red')
ekf.plot_ellipse()
plt.title('EKF con landmarks')
plt.grid(True)
plt.savefig('/home/tomas/lab_cap6/capturas/cap6_parte3_landmarks_trayectoria.png', dpi=150)

# Incertidumbre
plt.figure(figsize=(8, 6))
plt.plot(ekf.get_t(), ekf.get_Pnorm(), linewidth=2)
plt.xlabel('Tiempo [s]')
plt.ylabel('Incertidumbre')
plt.title('Incertidumbre con correcciones')
plt.grid(True)
plt.savefig('/home/tomas/lab_cap6/capturas/cap6_parte3_landmarks_incertidumbre.png', dpi=150)

plt.show()
