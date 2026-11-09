"""modelos.py - Arena, robot diferencial, sensores y servos (simulación Python pura)."""
import numpy as np
import config as C


def wrap(a):
    """Lleva un ángulo a (-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def centro_celda(c, f):
    return np.array([(c + 0.5) * C.CELDA, (f + 0.5) * C.CELDA])


def celda_de(p):
    return int(p[0] // C.CELDA), int(p[1] // C.CELDA)


def chaikin(pts, n=3):
    """Suaviza esquinas de una polilínea (corte de esquinas de Chaikin)."""
    pts = np.array(pts, float)
    for _ in range(n):
        q = 0.75 * pts[:-1] + 0.25 * pts[1:]
        r = 0.25 * pts[:-1] + 0.75 * pts[1:]
        nuevo = np.empty((2 * len(q), 2))
        nuevo[0::2], nuevo[1::2] = q, r
        pts = np.vstack([pts[0], nuevo, pts[-1]])
    return pts


class Arena:
    """Arena cuadriculada con una línea negra de 2 cm."""
    def __init__(self):
        v = [centro_celda(*c) for c in C.VERTICES_LINEA]
        self.linea = chaikin(v, 3)

    def error_lateral(self, P):
        """Distancia con signo de los puntos P (n,2) a la línea.
        Positivo = el punto está a la IZQUIERDA del sentido de avance de la línea."""
        P = np.atleast_2d(P)
        A, B = self.linea[:-1], self.linea[1:]
        AB = B - A
        L2 = np.maximum((AB ** 2).sum(1), 1e-12)
        AP = P[:, None, :] - A[None]
        t = np.clip((AP * AB[None]).sum(2) / L2[None], 0, 1)
        proj = A[None] + t[..., None] * AB[None]
        diff = P[:, None, :] - proj
        d = np.linalg.norm(diff, axis=2)
        i = d.argmin(1)
        k = np.arange(len(P))
        cruz = AB[i, 0] * diff[k, i, 1] - AB[i, 1] * diff[k, i, 0]
        return np.sign(cruz + 1e-15) * d[k, i]


class Caja:
    def __init__(self, centro):
        self.c = np.array(centro, float)
        self.agarrada = False
        self.z = 0.0


class RobotDiferencial:
    """Modelo cinemático diferencial con motores de primer orden (sin encoder)."""
    def __init__(self, pose, rng):
        self.x, self.y, self.th = pose
        self.vL = self.vR = 0.0           # velocidades reales de rueda
        self.v = self.w = 0.0
        self.rng = rng

    def paso(self, v_cmd, w_cmd, dt=C.DT, ruido=True):
        W = C.DIST_RUEDAS
        cL = np.clip(v_cmd - w_cmd * W / 2, -C.V_MAX_RUEDA, C.V_MAX_RUEDA)
        cR = np.clip(v_cmd + w_cmd * W / 2, -C.V_MAX_RUEDA, C.V_MAX_RUEDA)
        cL *= C.ASIMETRIA_MOTORES[0]
        cR *= C.ASIMETRIA_MOTORES[1]
        if ruido:
            cL *= 1 + C.RUIDO_RUEDA * self.rng.standard_normal()
            cR *= 1 + C.RUIDO_RUEDA * self.rng.standard_normal()
        a = dt / (C.TAU_MOTOR + dt)                    # retardo de primer orden
        self.vL += a * (cL - self.vL)
        self.vR += a * (cR - self.vR)
        self.v = (self.vR + self.vL) / 2
        self.w = (self.vR - self.vL) / W
        # integración discreta (punto medio)
        self.x += self.v * np.cos(self.th + self.w * dt / 2) * dt
        self.y += self.v * np.sin(self.th + self.w * dt / 2) * dt
        self.th = wrap(self.th + self.w * dt)

    @property
    def pose(self):
        return np.array([self.x, self.y, self.th])


class SensorLinea:
    """TCRT5000 x5: lectura analógica normalizada (1 = negro, 0 = blanco)."""
    def __init__(self, arena, rng):
        self.arena, self.rng = arena, rng
        self.off = (np.arange(C.N_CANALES) - C.N_CANALES // 2) * C.PASO_CANAL  # + = izquierda

    def posiciones(self, x, y, th):
        px = x + C.X_SENSOR_LINEA * np.cos(th) - self.off * np.sin(th)
        py = y + C.X_SENSOR_LINEA * np.sin(th) + self.off * np.cos(th)
        return np.column_stack([px, py])

    def leer(self, x, y, th, ruido=True):
        d = np.abs(self.arena.error_lateral(self.posiciones(x, y, th)))
        r = 1.0 / (1.0 + np.exp((d - C.ANCHO_LINEA / 2) / 0.003))
        if ruido:
            r = r + C.RUIDO_LINEA * self.rng.standard_normal(r.shape)
        return np.clip(r, 0, 1)

    def error(self, r):
        """Posición lateral de la línea en el marco del robot (+ izquierda) [m].
        Devuelve None si no ve la línea."""
        w = np.where(r > C.UMBRAL_LINEA, r, 0.0)
        if w.sum() < 0.3:
            return None
        return float((w * self.off).sum() / w.sum())


class SensorDistancia:
    """VL53L0X: rayo frontal contra la caja (AABB) con ruido gaussiano."""
    def __init__(self, rng):
        self.rng = rng

    def leer(self, x, y, th, caja, ruido=True):
        o = np.array([x + C.X_SENSOR_DIST * np.cos(th), y + C.X_SENSOR_DIST * np.sin(th)])
        u = np.array([np.cos(th), np.sin(th)])
        t = np.inf
        if not caja.agarrada:        # la caja sujeta no debe disparar la detección
            lo, hi = caja.c - C.CAJA_LADO / 2, caja.c + C.CAJA_LADO / 2
            t0, t1 = 0.0, np.inf
            for k in range(2):
                if abs(u[k]) < 1e-9:
                    if o[k] < lo[k] or o[k] > hi[k]:
                        t0, t1 = 1, 0
                else:
                    a, b = (lo[k] - o[k]) / u[k], (hi[k] - o[k]) / u[k]
                    t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
            if t0 <= t1:
                t = t0
        if not np.isfinite(t):
            return C.RANGO_MAX
        if ruido:
            t += (C.RUIDO_DIST_BASE + C.RUIDO_DIST_REL * t) * self.rng.standard_normal()
        return float(np.clip(t, C.RANGO_MIN, C.RANGO_MAX))


class Servo:
    """Servo modelado como actuador lineal con velocidad limitada."""
    def __init__(self, pos, vel):
        self.pos, self.vel = pos, vel

    def mover(self, objetivo, dt=C.DT):
        self.pos += float(np.clip(objetivo - self.pos, -self.vel * dt, self.vel * dt))
        return abs(objetivo - self.pos) < 1e-6
	
