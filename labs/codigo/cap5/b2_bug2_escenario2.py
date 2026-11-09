# Bug2 simplificado en grid - Parte B (Actividad B1)
import numpy as np
import matplotlib.pyplot as plt

grid = np.zeros((20, 20))
grid[1:4, 8:12] = 1          # obstáculo (filas y=8..11, columnas x=5..14)

start = np.array([2, 2])
goal = np.array([18, 2])
pos = start.copy()
path = [pos.copy()]

def is_free(p):
    return grid[p[1], p[0]] == 0

for _ in range(500):
    direccion = goal - pos
    if abs(direccion[0]) > abs(direccion[1]):
        step = np.array([np.sign(direccion[0]), 0])
    else:
        step = np.array([0, np.sign(direccion[1])])

    next_pos = pos + step
    if is_free(next_pos):
        pos = next_pos
    else:
        for move in [np.array([1, 0]), np.array([0, 1]), np.array([-1, 0]), np.array([0, -1])]:
            if is_free(pos + move):
                pos = pos + move
                break

    path.append(pos.copy())
    if np.linalg.norm(pos - goal) < 1:
        break

path = np.array(path)
print("Pasos realizados:", len(path) - 1)
print("Posición final:", path[-1], "| objetivo alcanzado:", np.array_equal(path[-1], goal))

plt.figure()
plt.imshow(grid, origin='lower', cmap='gray_r')
plt.plot(path[:, 0], path[:, 1], 'g', label='trayectoria')
plt.scatter(start[0], start[1], c='blue', label='inicio')
plt.scatter(goal[0], goal[1], c='red', label='objetivo')
plt.title("Bug2 simplificado")
plt.legend()
plt.savefig("graficas/B_trayectoria_esc2.png", dpi=150)
plt.show()
