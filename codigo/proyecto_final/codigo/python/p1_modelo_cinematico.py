"""p1_modelo_cinematico.py - Modelo diferencial: recta, curva y giro en sitio."""
import numpy as np
import matplotlib.pyplot as plt
import config as C
from modelos import RobotDiferencial

# nombre: (v, w, duración [s])
casos = {"Recta (v=0.15, w=0)": (0.15, 0.0, 4.0),
         "Curva (v=0.15, w=1.0)": (0.15, 1.0, 4.0),
         "Giro en sitio (v=0, w=2)": (0.0, 2.0, 1.0)}
rng = np.random.default_rng(0)
fig, ax = plt.subplots(figsize=(6, 6))
for nombre, (v, w, T) in casos.items():
    rb = RobotDiferencial((0, 0, 0), rng)
    xs, ys = [], []
    for _ in range(int(T / C.DT)):
        rb.paso(v, w, C.DT, ruido=False)
        xs.append(rb.x); ys.append(rb.y)
    ax.plot(xs, ys, label=nombre)
    print(f"{nombre:28s} -> x={rb.x:6.3f} m  y={rb.y:6.3f} m  th={np.degrees(rb.th):7.1f} deg")
ax.set_aspect("equal"); ax.grid(True); ax.legend()
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title(f"Modelo diferencial (dt = {C.DT} s, motores de 1er orden)")
fig.savefig(C.DIR_GRAFICAS / "p1_modelo_cinematico_trayectorias.png", dpi=150)
plt.show()
