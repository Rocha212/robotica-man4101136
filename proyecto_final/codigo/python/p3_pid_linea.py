"""p3_pid_linea.py - Seguimiento de línea con PID sobre toda la pista (sin caja)."""
import numpy as np
import matplotlib.pyplot as plt
import config as C
from modelos import Arena, RobotDiferencial, SensorLinea, centro_celda
from controladores import PID

def correr(kp, ki, kd, v_ref, semilla=0, T=14.0):
    rng = np.random.default_rng(semilla)
    arena = Arena()
    sl = SensorLinea(arena, rng)
    p = centro_celda(*C.CELDA_A)
    rb = RobotDiferencial((p[0], p[1] + 0.012, 0.08), rng)   # arranca 1.2 cm fuera y torcido
    pid, w_prev = PID(kp, ki, kd, C.DT, C.OMEGA_MAX, lim_i=0.05), 0.0
    H = {"t": [], "x": [], "y": [], "e": [], "e_med": [], "w": []}
    meta = arena.linea[-1]
    for k in range(int(T / C.DT)):
        e = sl.error(sl.leer(*rb.pose))
        if e is None:
            w, v = w_prev, 0.5 * v_ref
        else:
            w = pid(e); v = v_ref * (1 - 0.5 * min(1, abs(w) / C.OMEGA_MAX))
        w_prev = w
        rb.paso(v, w)
        H["t"].append(k * C.DT); H["x"].append(rb.x); H["y"].append(rb.y)
        H["e"].append(float(arena.error_lateral(rb.pose[:2])[0]) * 100)
        H["e_med"].append(np.nan if e is None else e * 100); H["w"].append(w)
        if np.hypot(*(rb.pose[:2] - meta)) < 0.03:
            break
    return {k: np.array(v) for k, v in H.items()}, arena

def metricas(H, t0=1.0):
    e = H["e"][H["t"] > t0]
    return np.sqrt(np.mean(e ** 2)), np.max(np.abs(e)), 100 * np.mean(np.abs(e) < 1.0)

if __name__ == "__main__":
    # Comparación de ganancias
    configs = {"P (Kp=100)": (100, 0, 0), "PD (100, Kd=1.5)": (100, 0, 1.5),
               "PID (100, 40, 1.5)": (C.KP, C.KI, C.KD)}
    fig, axs = plt.subplots(2, 1, figsize=(9, 7))
    print(f"{'controlador':22s} {'RMS[cm]':>8s} {'max[cm]':>8s} {'%|e|<1cm':>9s} {'T[s]':>6s}")
    for nombre, (kp, ki, kd) in configs.items():
        H, arena = correr(kp, ki, kd, C.V_LINEA)
        rms, mx, pct = metricas(H)
        print(f"{nombre:22s} {rms:8.2f} {mx:8.2f} {pct:9.1f} {H['t'][-1]:6.1f}")
        axs[0].plot(H["t"], H["e"], label=nombre)
    axs[0].axhline(1, color="r", ls="--"); axs[0].axhline(-1, color="r", ls="--", label="límite ±1 cm")
    axs[0].set_xlabel("t [s]"); axs[0].set_ylabel("error lateral real [cm]")
    axs[0].set_title("Error lateral: P vs PD vs PID"); axs[0].grid(True); axs[0].legend(fontsize=8)
    H, arena = correr(C.KP, C.KI, C.KD, C.V_LINEA)
    axs[1].plot(arena.linea[:, 0], arena.linea[:, 1], "k", lw=4, label="línea")
    axs[1].plot(H["x"], H["y"], "tab:orange", label="robot"); axs[1].set_aspect("equal")
    axs[1].legend(); axs[1].grid(True); axs[1].set_xlabel("x [m]"); axs[1].set_ylabel("y [m]")
    fig.tight_layout(); fig.savefig(C.DIR_GRAFICAS / "p3_pid_error_lateral.png", dpi=150)
    # Barrido de velocidad
    print("\nBarrido de velocidad (PID):")
    for v in (0.10, 0.15, 0.20, 0.25):
        H, _ = correr(C.KP, C.KI, C.KD, v)
        print(f"v={v:.2f} m/s -> RMS={metricas(H)[0]:.2f} cm, max={metricas(H)[1]:.2f} cm, %<1cm={metricas(H)[2]:.0f}")
    plt.show()
