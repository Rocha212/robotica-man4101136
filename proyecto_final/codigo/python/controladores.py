"""controladores.py - PID de línea y Pure Pursuit."""
import numpy as np
from modelos import wrap


class PID:
    def __init__(self, kp, ki, kd, dt, lim_u, lim_i=None, alfa_d=0.6):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.lim_u = lim_u
        self.lim_i = lim_i if lim_i is not None else lim_u / max(ki, 1e-9)
        self.alfa_d = alfa_d
        self.reset()

    def reset(self):
        self.i = 0.0
        self.e_prev = None
        self.d_f = 0.0

    def __call__(self, e):
        self.i = float(np.clip(self.i + e * self.dt, -self.lim_i, self.lim_i))   # anti-windup
        d = 0.0 if self.e_prev is None else (e - self.e_prev) / self.dt
        self.d_f = self.alfa_d * self.d_f + (1 - self.alfa_d) * d                # derivada filtrada
        self.e_prev = e
        u = self.kp * e + self.ki * self.i + self.kd * self.d_f
        return float(np.clip(u, -self.lim_u, self.lim_u))


def densificar(ruta, paso=0.01):
    ruta = np.array(ruta, float)
    out = [ruta[0]]
    for a, b in zip(ruta[:-1], ruta[1:]):
        n = max(int(np.ceil(np.linalg.norm(b - a) / paso)), 1)
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.array(out)


class PurePursuit:
    """Pure Pursuit: w = v * kappa, kappa = 2 sin(alfa) / Ld."""
    def __init__(self, ruta, lookahead, v, tol_meta=0.02):
        self.ruta = densificar(ruta, 0.01)
        self.Ld, self.v, self.tol = lookahead, v, tol_meta
        self.idx = 0
        self.llego = False
        self.objetivo = self.ruta[0]

    def comando(self, x, y, th):
        R = self.ruta
        fin = min(self.idx + 80, len(R))                 # ventana: solo avanza, nunca retrocede
        d = np.hypot(R[self.idx:fin, 0] - x, R[self.idx:fin, 1] - y)
        self.idx += int(d.argmin())
        d_meta = np.hypot(R[-1, 0] - x, R[-1, 1] - y)
        if d_meta < self.tol:
            self.llego = True
            return 0.0, 0.0
        j = self.idx
        while j < len(R) - 1 and np.hypot(R[j, 0] - x, R[j, 1] - y) < self.Ld:
            j += 1
        self.objetivo = R[j]
        alfa = wrap(np.arctan2(R[j, 1] - y, R[j, 0] - x) - th)
        if abs(alfa) > np.deg2rad(80):                   # objetivo detrás: girar en sitio
            return 0.0, 2.0 * np.sign(alfa)
        v = self.v * max(0.35, np.cos(alfa))
        v = min(v, max(0.03, 1.5 * d_meta))              # frenado suave al final
        kappa = 2 * np.sin(alfa) / max(self.Ld, 1e-6)
        return v, v * kappa
