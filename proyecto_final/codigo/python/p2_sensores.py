"""p2_sensores.py - Respuesta del TCRT5000 x5 y del VL53L0X sobre la arena."""
import numpy as np
import matplotlib.pyplot as plt
import config as C
from modelos import Arena, Caja, SensorLinea, SensorDistancia, centro_celda
from visual import dibujar_arena

rng = np.random.default_rng(1)
arena = Arena()
sl, sd = SensorLinea(arena, rng), SensorDistancia(rng)

# (a) Error medido vs desplazamiento lateral real (robot recto sobre el primer tramo)
desp = np.linspace(-0.04, 0.04, 161)
y0 = centro_celda(*C.CELDA_A)[1]
e_med = []
for d in desp:
    r = sl.leer(0.3, y0 + d, 0.0, ruido=False)
    e_med.append(sl.error(r))
e_med = np.array([np.nan if e is None else e for e in e_med], float)

# (b) Lecturas de los 5 canales con el robot centrado y desplazado 1.2 cm
lect_c = sl.leer(0.3, y0, 0.0, ruido=False)
lect_d = sl.leer(0.3, y0 + 0.012, 0.0, ruido=False)

# (c) Distancia: robot acercándose a la caja
caja = Caja(centro_celda(*C.CELDA_CAJA))
xs = np.linspace(0.05, 0.45, 200)
d_real = caja.c[0] - C.CAJA_LADO / 2 - (xs + C.X_SENSOR_DIST)
d_med = [sd.leer(x, y0, 0.0, caja) for x in xs]

fig, axs = plt.subplots(1, 3, figsize=(15, 4.2))
axs[0].plot(desp * 100, -e_med * 100, label="medido (línea vista desde el robot)")
axs[0].plot(desp * 100, desp * 100, "--", label="ideal")
axs[0].set_xlabel("desplazamiento lateral real [cm]"); axs[0].set_ylabel("error medido [cm]")
axs[0].set_title("Sensor de línea: linealidad"); axs[0].grid(True); axs[0].legend(fontsize=8)
w = 0.35; idx = np.arange(C.N_CANALES)
axs[1].bar(idx - w / 2, lect_c, w, label="centrado"); axs[1].bar(idx + w / 2, lect_d, w, label="+1.2 cm")
axs[1].set_xlabel("canal (0 = derecha ... 4 = izquierda)"); axs[1].set_ylabel("lectura normalizada")
axs[1].set_title("Lecturas de los 5 canales"); axs[1].legend()
axs[2].plot(d_real * 100, np.array(d_med) * 100, ".", ms=3, label="VL53L0X simulado")
axs[2].plot(d_real * 100, d_real * 100, "r--", label="distancia real")
axs[2].set_xlim(0, 40); axs[2].set_ylim(0, 40)
axs[2].set_xlabel("distancia real [cm]"); axs[2].set_ylabel("distancia medida [cm]")
axs[2].set_title("Sensor de distancia"); axs[2].grid(True); axs[2].legend(fontsize=8)
fig.tight_layout()
fig.savefig(C.DIR_GRAFICAS / "p2_sensores_respuesta.png", dpi=150)

fig2, ax = plt.subplots(figsize=(6.4, 6.4))
dibujar_arena(ax, arena, titulo="Arena 6x6 y línea de 2 cm")
ax.add_patch(plt.Rectangle(caja.c - C.CAJA_LADO / 2, C.CAJA_LADO, C.CAJA_LADO, color="saddlebrown"))
fig2.savefig(C.DIR_GRAFICAS / "p2_arena_linea.png", dpi=150)
print("sigma distancia a 10 cm:", np.std([sd.leer(0.2, y0, 0, caja) for _ in range(500)]))
plt.show()
