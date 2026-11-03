# Distance Transform - Parte D (Actividad D1)
import numpy as np
import matplotlib.pyplot as plt

# ========================= MAPA =========================
grid = np.zeros((20, 20))
grid[8:12, 5:15] = 1          # obstáculo (filas y=8..11, columnas x=5..14)

goal = (2, 2)                  # celda objetivo (x, y)
start = (18, 18)               # punto inicial (x, y)

vecinos = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# ========================= DISTANCE TRANSFORM =========================
dist = np.full(grid.shape, np.inf)
dist[goal[1], goal[0]] = 0

# propagación iterativa desde el objetivo
for _ in range(100):
    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            if grid[y, x] == 0 and (x, y) != goal:
                for dx, dy in vecinos:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                        dist[y, x] = min(dist[y, x], dist[ny, nx] + 1)

# ========================= SEGUIMIENTO DEL CAMINO =========================
pos = start
path = [pos]
for _ in range(200):
    if pos == goal:
        break
    x, y = pos
    candidatos = []
    for dx, dy in vecinos:
        nx, ny = x + dx, y + dy
        if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0] and grid[ny, nx] == 0:
            candidatos.append(((nx, ny), dist[ny, nx]))
    pos = min(candidatos, key=lambda t: t[1])[0]
    path.append(pos)

path = np.array(path)
print("Longitud del camino (pasos):", len(path) - 1)
print("Distancia en el mapa desde el inicio:", dist[start[1], start[0]])

# ========================= VISUALIZACIÓN =========================
cmap = plt.cm.hot.copy()
cmap.set_bad('gray')                       # obstáculos en gris
dist_vis = np.where(grid == 1, np.nan, dist)

plt.figure(figsize=(6, 6))
plt.imshow(dist_vis, origin='lower', cmap=cmap)
plt.colorbar(label="Distancia al objetivo")
plt.plot(path[:, 0], path[:, 1], 'cyan', linewidth=2, label="trayectoria")
plt.scatter(start[0], start[1], c='blue', s=80, label="inicio")
plt.scatter(goal[0], goal[1], c='green', s=80, label="objetivo")
plt.title("Distance Transform - Mapa de distancias")
plt.legend()
plt.savefig("graficas/D_distancias.png", dpi=150)
plt.show()
