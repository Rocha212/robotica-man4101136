# Braitenberg Vehicle - Parte A (Actividad A1)
import numpy as np
import matplotlib.pyplot as plt

# Campo escalar (objetivo)
def sensorfield(x, y):
    xc, yc = (60, 90)
    return 1000 / ((x - xc)**2 + (y - yc)**2 + 50)

# Parámetros
dt = 0.1
Ks = 17
Kv = 14
d = 2  # separación lateral de los sensores

# Estado inicial
x, y, theta = 10, 10, 0
traj_x, traj_y = [], []

for i in range(600):
    # Sensores (offset lateral)
    s_left = sensorfield(x - d*np.sin(theta), y + d*np.cos(theta))
    s_right = sensorfield(x + d*np.sin(theta), y - d*np.cos(theta))

    # Ley de control
    v = -Kv * (s_left + s_right)        # velocidad lineal
    omega = Ks * (s_right - s_left)     # velocidad angular

    # Cinemática
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega * dt

    traj_x.append(x)
    traj_y.append(y)

# Visualización
plt.figure()
plt.plot(traj_x, traj_y, label="trayectoria")
plt.scatter([10], [10], c='blue', label="inicio")
plt.scatter([60], [90], c='red', label="objetivo")
plt.title("Navegación Braitenberg")
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.savefig("graficas/A_trayectoria_600.png", dpi=150)
plt.show()
