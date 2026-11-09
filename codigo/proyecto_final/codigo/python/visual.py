"""visual.py - Dibujo de la arena y animación con FuncAnimation (+ MP4 con ffmpeg)."""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.patches import Polygon, Rectangle
import config as C


def dibujar_arena(ax, arena, malla=None, titulo=""):
    ax.set_xlim(0, C.LADO)
    ax.set_ylim(0, C.LADO)
    ax.set_aspect("equal")
    for k in range(C.N_CELDAS + 1):
        ax.axvline(k * C.CELDA, color="0.8", lw=0.6, zorder=0)
        ax.axhline(k * C.CELDA, color="0.8", lw=0.6, zorder=0)
    if malla is not None:
        for f in range(C.N_CELDAS):
            for c in range(C.N_CELDAS):
                if malla[f, c]:
                    ax.add_patch(Rectangle((c * C.CELDA, f * C.CELDA), C.CELDA, C.CELDA,
                                           color="0.55", zorder=1))
    lw = C.ANCHO_LINEA / C.LADO * 5.0 * 72          # 2 cm expresado en puntos (axes ~5 in)
    ax.plot(arena.linea[:, 0], arena.linea[:, 1], color="k", lw=lw,
            solid_capstyle="butt", zorder=2)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(titulo)


def poligono_robot(x, y, th):
    """Contorno del robot (rectángulo) en coordenadas del mundo."""
    L, A = C.LARGO_ROBOT, C.ANCHO_ROBOT
    pts = np.array([[-L / 2 + 0.02, -A / 2], [L / 2 + 0.02, -A / 2],
                    [L / 2 + 0.02, A / 2], [-L / 2 + 0.02, A / 2]])
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return pts @ R.T + [x, y]


def poligono_caja(c):
    h = C.CAJA_LADO / 2
    return np.array([[c[0] - h, c[1] - h], [c[0] + h, c[1] - h],
                     [c[0] + h, c[1] + h], [c[0] - h, c[1] + h]])


def animar(hist, arena, malla, ruta=None, titulo="Misión completa", guardar=None,
           mostrar=True, salto=2):
    """hist: dict de listas (t, x, y, th, estado, caja_x, caja_y, pinza, elev).
    salto: se dibuja 1 de cada 'salto' pasos (salto=2 y DT=0.02 -> 25 fps)."""
    fig, ax = plt.subplots(figsize=(6.4, 6.4))
    dibujar_arena(ax, arena, malla, titulo)
    if ruta is not None:
        ax.plot(ruta[:, 0], ruta[:, 1], "--", color="tab:red", lw=1, zorder=3, label="ruta A*")
        ax.legend(loc="upper left", fontsize=8)
    for nombre, celda, col in (("A", C.CELDA_A, "tab:blue"), ("B", C.CELDA_B, "tab:green")):
        cx, cy = (celda[0] + .5) * C.CELDA, (celda[1] + .5) * C.CELDA
        ax.text(cx, cy, nombre, ha="center", va="center", color=col, fontsize=14,
                fontweight="bold", zorder=1)
    robot = Polygon(poligono_robot(0, 0, 0), closed=True, fc="tab:orange", ec="k", zorder=5)
    caja = Polygon(poligono_caja((0, 0)), closed=True, fc="saddlebrown", ec="k", zorder=6)
    traza, = ax.plot([], [], color="tab:blue", lw=1, zorder=4)
    texto = ax.text(0.02, 0.02, "", transform=ax.transAxes, fontsize=9,
                    bbox=dict(fc="w", alpha=0.8))
    ax.add_patch(robot)
    ax.add_patch(caja)
    idx = list(range(0, len(hist["t"]), salto))

    def actualizar(k):
        i = idx[k]
        robot.set_xy(poligono_robot(hist["x"][i], hist["y"][i], hist["th"][i]))
        caja.set_xy(poligono_caja((hist["caja_x"][i], hist["caja_y"][i])))
        traza.set_data(hist["x"][:i + 1], hist["y"][:i + 1])
        texto.set_text(f"t = {hist['t'][i]:5.1f} s | {hist['estado'][i]}\n"
                       f"pinza {hist['pinza'][i]*100:4.1f} cm | elevador {hist['elev'][i]*100:3.1f} cm")
        return robot, caja, traza, texto

    ani = FuncAnimation(fig, actualizar, frames=len(idx), interval=1000 * C.DT * salto,
                        blit=False, repeat=False)
    if guardar:
        fps = int(round(1 / (C.DT * salto)))
        ani.save(str(guardar), writer=FFMpegWriter(fps=fps, bitrate=2000))
        print("Video guardado en", guardar)
    if mostrar:
        plt.show()
    else:
        plt.close(fig)
    return ani
