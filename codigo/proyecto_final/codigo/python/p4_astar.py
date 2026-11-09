"""p4_astar.py - A* en la malla: comparación de heurísticas (UCS, euclidiana, Manhattan)."""
import numpy as np
import matplotlib.pyplot as plt
import config as C
from modelos import Arena
from planeacion import crear_malla, astar, h_cero, h_euclidiana, h_manhattan, simplificar
from visual import dibujar_arena

malla = crear_malla()
inicio, meta = (1, 0), C.CELDA_B          # robot ya con la caja, en la celda junto a la caja
print(f"Malla {C.N_CELDAS}x{C.N_CELDAS}, inicio={inicio}, meta={meta}, bloqueadas={C.CELDAS_BLOQUEADAS}\n")
print(f"{'heurística':14s} {'pasos':>6s} {'expandidos':>11s}")
for nombre, h in (("UCS (h=0)", h_cero), ("Euclidiana", h_euclidiana), ("Manhattan", h_manhattan)):
    camino, n = astar(malla, inicio, meta, h)
    print(f"{nombre:14s} {len(camino)-1:6d} {n:11d}")

camino, _ = astar(malla, inicio, meta, h_manhattan)
esquinas = simplificar(camino)
print("\nCamino (celdas):", camino)
print("Esquinas       :", esquinas)

fig, ax = plt.subplots(figsize=(6.4, 6.4))
dibujar_arena(ax, Arena(), malla, "A* con heurística Manhattan")
cs = np.array([((c + .5) * C.CELDA, (f + .5) * C.CELDA) for c, f in camino])
ax.plot(cs[:, 0], cs[:, 1], "o-", color="tab:red", label="camino A*")
ax.scatter(*cs[0], c="tab:blue", s=90, zorder=6, label="inicio")
ax.scatter(*cs[-1], c="tab:green", s=90, zorder=6, label="meta B")
ax.legend(loc="upper left")
fig.savefig(C.DIR_GRAFICAS / "p4_astar_camino.png", dpi=150)
plt.show()
