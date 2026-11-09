"""p5_pure_pursuit.py - Pure Pursuit sobre la ruta de A*: efecto del lookahead."""
import numpy as np
import matplotlib.pyplot as plt
import config as C
from modelos import Arena, RobotDiferencial, centro_celda
from controladores import PurePursuit
from planeacion import crear_malla, astar, simplificar, h_manhattan
from visual import dibujar_arena

malla = crear_malla()
camino, _ = astar(malla, (1, 0), C.CELDA_B, h_manhattan)
esq = simplificar(camino)
p0 = centro_celda(*camino[0])
ruta = np.array([p0] + [centro_celda(*c) for c in esq[1:]])

fig, ax = plt.subplots(figsize=(6.4, 6.4))
dibujar_arena(ax, Arena(), malla, "Pure Pursuit: efecto del lookahead")
ax.plot(ruta[:, 0], ruta[:, 1], "k--", lw=1, label="ruta A*")
print(f"{'Ld [m]':>7s} {'llegó':>6s} {'t [s]':>6s} {'err. lateral máx [cm]':>22s}")
for Ld in (0.05, 0.12, 0.30):
    rng = np.random.default_rng(0)
    rb = RobotDiferencial((p0[0], p0[1], 0.0), rng)
    pp = PurePursuit(ruta, Ld, C.V_TRANSPORTE)
    xs, ys, err = [], [], []
    for k in range(int(40 / C.DT)):
        v, w = pp.comando(*rb.pose)
        rb.paso(v, w)
        xs.append(rb.x); ys.append(rb.y)
        # error transversal a la ruta (distancia al punto más cercano)
        err.append(np.min(np.hypot(pp.ruta[:, 0] - rb.x, pp.ruta[:, 1] - rb.y)))
        if pp.llego:
            break
    ax.plot(xs, ys, label=f"Ld = {Ld} m")
    print(f"{Ld:7.2f} {str(pp.llego):>6s} {k*C.DT:6.1f} {100*max(err):22.1f}")
ax.legend(loc="upper left", fontsize=8)
fig.savefig(C.DIR_GRAFICAS / "p5_pure_pursuit_lookahead.png", dpi=150)
plt.show()
