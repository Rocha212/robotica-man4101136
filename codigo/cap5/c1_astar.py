# A* desde cero - Parte C (Actividad C1)
import heapq
import numpy as np
import matplotlib.pyplot as plt

grid = np.zeros((20, 20))
grid[8:12, 5:15] = 1          # obstáculo (filas y=8..11, columnas x=5..14)

start = (2, 2)
goal = (18, 18)

def heuristica(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def astar(grid, start, goal, usar_heuristica=True):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    cost = {start: 0}
    expandidos = 0

    while open_list:
        _, current = heapq.heappop(open_list)
        expandidos += 1
        if current == goal:
            break

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < grid.shape[1] and
                    0 <= neighbor[1] < grid.shape[0] and
                    grid[neighbor[1], neighbor[0]] == 0):
                new_cost = cost[current] + 1
                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    h = heuristica(neighbor, goal) if usar_heuristica else 0
                    heapq.heappush(open_list, (new_cost + h, neighbor))
                    came_from[neighbor] = current

    # reconstruir camino
    path = [goal]
    node = goal
    while node != start:
        node = came_from[node]
        path.append(node)
    path.reverse()
    return path, expandidos

path, exp_astar = astar(grid, start, goal, usar_heuristica=True)
_, exp_ucs = astar(grid, start, goal, usar_heuristica=False)
path = np.array(path)

print("Longitud del camino (pasos):", len(path) - 1)
print("Nodos expandidos A*:", exp_astar)
print("Nodos expandidos UCS (sin heurística):", exp_ucs)

plt.figure()
plt.imshow(grid, origin='lower', cmap='gray_r')
plt.plot(path[:, 0], path[:, 1], 'r', linewidth=2, label='A*')
plt.scatter(start[0], start[1], c='blue', label='inicio')
plt.scatter(goal[0], goal[1], c='red', label='objetivo')
plt.title("A* Path")
plt.legend()
plt.savefig("graficas/C_ruta.png", dpi=150)
plt.show()
