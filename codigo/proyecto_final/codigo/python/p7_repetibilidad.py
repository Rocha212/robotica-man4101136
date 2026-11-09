"""p7_repetibilidad.py - N corridas con ruido y perturbaciones: criterio de éxito >= 9/10."""
import numpy as np
import matplotlib.pyplot as plt
import config as C
from mision import Mision

N = 30
rng = np.random.default_rng(123)
filas = []
for s in range(N):
    pert = (rng.uniform(-0.01, 0.01), rng.uniform(-0.015, 0.015), rng.uniform(-0.15, 0.15))
    off_caja = rng.uniform(-0.01, 0.01)               # caja colocada con +-1 cm de error
    m = Mision(semilla=s, perturbacion_inicial=pert, offset_caja=off_caja)
    ok = m.ejecutar()
    e = np.abs(np.array(m.log["e_true"])[(np.array(m.log["t"]) > 1.0) &
               np.array([x.startswith("SEGUIR") for x in m.log["estado"]])])
    filas.append((s, ok, m.t_fin, e.max() * 100 if len(e) else np.nan, m.motivo))
    print(f"corrida {s:2d}: {'OK ' if ok else 'FALLO'} t={m.t_fin:5.1f}s  e_max_línea={filas[-1][3]:.2f} cm {m.motivo}")

exitos = sum(f[1] for f in filas)
print(f"\nÉxitos: {exitos}/{N} ({100*exitos/N:.0f} %)  -> criterio >= 90 %: {'CUMPLE' if exitos/N >= .9 else 'NO CUMPLE'}")
t_ok = [f[2] for f in filas if f[1]]
print(f"Tiempo medio de misión: {np.mean(t_ok):.1f} s (máx {np.max(t_ok):.1f} s, límite {C.T_MAX_MISION} s)")
np.savetxt(C.DIR_DATOS / "p7_repetibilidad.csv", [(f[0], int(f[1]), f[2], f[3]) for f in filas],
           delimiter=",", header="corrida,exito,t_mision_s,e_max_linea_cm", comments="")
fig, ax = plt.subplots(1, 2, figsize=(10, 4))
ax[0].bar(["éxito", "fallo"], [exitos, N - exitos], color=["g", "r"]); ax[0].set_title("Resultados")
ax[1].hist(t_ok, bins=10); ax[1].set_xlabel("tiempo de misión [s]"); ax[1].set_title("Tiempos")
fig.tight_layout(); fig.savefig(C.DIR_GRAFICAS / "p7_repetibilidad.png", dpi=150)
plt.show()
