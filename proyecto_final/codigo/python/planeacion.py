"""planeacion.py - A* en malla de 4 vecinos."""
import heapq
import numpy as np
import config as C


def h_manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def h_euclidiana(a, b):
    return float(np.hypot(a[0] - b[0], a[1] - b[1]))


def h_cero(a, b):
    return 0.0


def crear_malla():
    """malla[fila, col]: 0 = libre, 1 = obstáculo."""
    m = np.zeros((C.N_CELDAS, C.N_CELDAS), int)
    for c, f in C.CELDAS_BLOQUEADAS:
        m[f, c] = 1
    return m


def astar(malla, inicio, meta, h=h_manhattan):
    """Devuelve (camino en celdas, nodos expandidos). Usa conjunto cerrado
    (cada nodo se expande una sola vez) para que el conteo sea exacto."""
    n = malla.shape[0]
    abierta = [(h(inicio, meta), 0, inicio)]
    g = {inicio: 0}
    padre = {}
    cerrados = set()
    cont = 0
    while abierta:
        _, gc, cur = heapq.heappop(abierta)
        if cur in cerrados:
            continue
        cerrados.add(cur)
        cont += 1
        if cur == meta:
            break
        for dc, df in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nb = (cur[0] + dc, cur[1] + df)
            if 0 <= nb[0] < n and 0 <= nb[1] < n and malla[nb[1], nb[0]] == 0:
                ng = gc + 1
                if ng < g.get(nb, 1e9):
                    g[nb] = ng
                    padre[nb] = cur
                    heapq.heappush(abierta, (ng + h(nb, meta), ng, nb))
    if meta not in padre and inicio != meta:
        return None, cont
    camino = [meta]
    while camino[-1] != inicio:
        camino.append(padre[camino[-1]])
    return camino[::-1], cont


def simplificar(camino):
    """Elimina celdas colineales: deja inicio, esquinas y meta."""
    if len(camino) < 3:
        return list(camino)
    out = [camino[0]]
    for a, b, c in zip(camino[:-2], camino[1:-1], camino[2:]):
        if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
            out.append(b)
    out.append(camino[-1])
    return out
