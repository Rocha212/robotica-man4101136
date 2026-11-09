"""p6_mision_completa.py - Misión completa (6 fases) con gráficas, animación y MP4.
Uso:  python3 p6_mision_completa.py            # ventana en tiempo real
      python3 p6_mision_completa.py --guardar  # además exporta el MP4
      python3 p6_mision_completa.py --sin-ventana --guardar"""
import argparse
import numpy as np
import matplotlib.pyplot as plt
import config as C
from mision import Mision
from visual import animar, dibujar_arena

ap = argparse.ArgumentParser()
ap.add_argument("--guardar", action="store_true", help="exporta videos/p6_mision_completa.mp4")
ap.add_argument("--sin-ventana", action="store_true")
ap.add_argument("--semilla", type=int, default=1)
a = ap.parse_args()

m = Mision(semilla=a.semilla)
ok = m.ejecutar()
L = {k: np.array(v) if k != "estado" else v for k, v in m.log.items()}
print(f"Resultado: {'ÉXITO' if ok else 'FALLO'} {m.motivo} | t_misión = {m.t_fin:.1f} s | "
      f"nodos A* expandidos = {m.nodos_expandidos}")
fases = {}
for t, s in zip(L["t"], L["estado"]):
    fases.setdefault(s.split("/")[0], t)
for f, t in fases.items():
    print(f"  {f:14s} inicia en t = {t:5.2f} s")

# ---------- Gráficas ----------
fig, ax = plt.subplots(figsize=(6.4, 6.4))
dibujar_arena(ax, m.arena, m.malla, "Trayectoria de la misión")
ax.plot(L["x"], L["y"], color="tab:blue", lw=1.2, label="robot")
ax.plot(m.ruta[:, 0], m.ruta[:, 1], "r--", lw=1, label="ruta A*")
ax.scatter(L["caja_x"][0], L["caja_y"][0], marker="s", c="saddlebrown", label="caja (inicio)")
ax.scatter(L["caja_x"][-1], L["caja_y"][-1], marker="s", c="g", label="caja (final)")
ax.legend(loc="upper left", fontsize=8)
fig.savefig(C.DIR_GRAFICAS / "p6_mision_trayectoria.png", dpi=150)

fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
axs[0].plot(L["t"], L["e_true"] * 100); axs[0].set_ylabel("error lateral [cm]")
axs[0].set_ylim(-3, 3); axs[0].axhline(1, c="r", ls="--"); axs[0].axhline(-1, c="r", ls="--")
axs[1].plot(L["t"], L["dist"] * 100); axs[1].axhline(C.D_DETECCION * 100, c="r", ls="--")
axs[1].set_ylabel("VL53L0X [cm]"); axs[1].set_ylim(0, 40)
axs[2].plot(L["t"], L["v"], label="v [m/s]"); axs[2].plot(L["t"], L["w"], label="w [rad/s]")
axs[2].legend(); axs[2].set_ylabel("velocidades")
axs[3].plot(L["t"], L["pinza"] * 100, label="pinza (X) [cm]")
axs[3].plot(L["t"], L["elev"] * 100, label="elevador (Y) [cm]"); axs[3].legend()
axs[3].set_xlabel("t [s]")
for t in fases.values():
    for x in axs:
        x.axvline(t, color="0.6", lw=0.6)
for f, t in fases.items():
    axs[0].text(t, 2.6, f[:6], rotation=90, fontsize=7, va="top")
for x in axs:
    x.grid(True, alpha=0.4)
fig.tight_layout()
fig.savefig(C.DIR_GRAFICAS / "p6_mision_senales.png", dpi=150)

# ---------- Animación ----------
destino = C.DIR_VIDEOS / "p6_mision_completa.mp4" if a.guardar else None
animar(L, m.arena, m.malla, m.ruta, guardar=destino, mostrar=not a.sin_ventana)
if not a.sin_ventana:
    plt.show()
