"""mision.py - Máquina de estados de la misión completa (6 fases)."""
from enum import Enum
import numpy as np
import config as C
from modelos import (Arena, Caja, RobotDiferencial, SensorLinea, SensorDistancia, Servo,
                     centro_celda, celda_de, wrap)
from controladores import PID, PurePursuit
from planeacion import astar, simplificar, crear_malla, h_manhattan


class Estado(Enum):
    SEGUIR_LINEA = 1
    DETECTAR_CAJA = 2
    RECOGER = 3
    TRANSPORTAR = 4
    DEPOSITAR = 5
    FINALIZAR = 6


class Mision:
    def __init__(self, semilla=0, ruido=True, perturbacion_inicial=(0.0, 0.012, 0.08),
                 offset_caja=0.0):
        self.rng = np.random.default_rng(semilla)
        self.ruido = ruido
        self.arena = Arena()
        self.malla = crear_malla()
        pA = centro_celda(*C.CELDA_A)
        pose0 = (pA[0] + perturbacion_inicial[0], pA[1] + perturbacion_inicial[1],
                 perturbacion_inicial[2])
        self.robot = RobotDiferencial(pose0, self.rng)
        self.caja = Caja(centro_celda(*C.CELDA_CAJA) + [0.0, offset_caja])
        self.s_lin = SensorLinea(self.arena, self.rng)
        self.s_dist = SensorDistancia(self.rng)
        self.pid = PID(C.KP, C.KI, C.KD, C.DT, C.OMEGA_MAX, lim_i=0.05)
        self.pinza = Servo(C.GAP_CERRADA_VACIA, C.VEL_PINZA)
        self.elev = Servo(C.H_ARRIBA, C.VEL_ELEV)
        self.estado, self.sub = Estado.SEGUIR_LINEA, ""
        self.t, self.t_sub = 0.0, 0.0
        self.cnt_det, self.v_ref = 0, C.V_LINEA
        self.w_prev = 0.0
        self.hist_dist = []
        self.pp, self.ruta, self.t_plan = None, None, 0.0
        self.recorrido_retro = 0.0
        self.resultado = None
        self.motivo = ""
        self.log = {k: [] for k in ("t", "x", "y", "th", "estado", "e_true", "e_med", "dist",
                                    "v", "w", "pinza", "elev", "caja_x", "caja_y")}

    # ---------- utilidades ----------
    def _set(self, estado, sub=""):
        self.estado, self.sub, self.t_sub = estado, sub, 0.0

    def _seguir_linea(self, e, v_ref):
        """PID sobre el error lateral medido; si pierde la línea, mantiene el último giro."""
        if e is None:
            w = self.w_prev
            v = 0.5 * v_ref
        else:
            w = self.pid(e)
            v = v_ref * (1 - 0.5 * min(1.0, abs(w) / C.OMEGA_MAX))
        self.w_prev = w
        return v, w

    def _pose_caja_llevada(self):
        x, y, th = self.robot.pose
        return np.array([x + C.L_AGARRE * np.cos(th), y + C.L_AGARRE * np.sin(th)])

    # ---------- un paso de simulación ----------
    def paso(self):
        rb, dt = self.robot, C.DT
        x, y, th = rb.pose
        r = self.s_lin.leer(x, y, th, self.ruido)
        e = self.s_lin.error(r)
        dist = self.s_dist.leer(x, y, th, self.caja, self.ruido)
        v = w = 0.0
        E = self.estado

        if E == Estado.SEGUIR_LINEA:
            v, w = self._seguir_linea(e, C.V_LINEA)
            self.cnt_det = self.cnt_det + 1 if dist <= C.D_DETECCION else 0
            if self.cnt_det >= 3:                       # 3 lecturas seguidas: evita falsos positivos
                self.v_ref = max(abs(rb.v), 0.0)
                self._set(Estado.DETECTAR_CAJA)

        elif E == Estado.DETECTAR_CAJA:
            self.v_ref = max(0.0, self.v_ref - C.A_FRENADO * dt)     # rampa de frenado
            v, w = self._seguir_linea(e, self.v_ref)
            if self.v_ref == 0.0 and abs(rb.v) < 0.005:
                self.d_detenido = dist
                self._set(Estado.RECOGER, "alinear")

        elif E == Estado.RECOGER:
            v, w = self._recoger(e, dist)

        elif E == Estado.TRANSPORTAR:
            v, w = self._transportar()

        elif E == Estado.DEPOSITAR:
            v = self._depositar()

        rb.paso(v, w, dt, self.ruido)
        if self.caja.agarrada:
            self.caja.c = self._pose_caja_llevada()
            self.caja.z = self.elev.pos
        self.t += dt
        self.t_sub += dt

        if self.estado != Estado.FINALIZAR and self.t > C.T_MAX_MISION:
            self._finalizar(False, "tiempo máximo excedido")
        self._registrar(e, dist, v, w)

    # ---------- fase 3: recoger ----------
    def _recoger(self, e, dist):
        s, v, w = self.sub, 0.0, 0.0
        if s == "alinear":                               # v=0: gira hasta centrar el sensor en la línea
            _, w = self._seguir_linea(e, 0.0)
            self.ok_align = getattr(self, "ok_align", 0)
            self.ok_align = self.ok_align + 1 if (e is not None and abs(e) < 0.003) else 0
            if self.ok_align > 15 or self.t_sub > 3.0:
                self._set(Estado.RECOGER, "abrir")
        elif s == "abrir":
            if self.pinza.mover(C.GAP_ABIERTA):
                self._set(Estado.RECOGER, "bajar")
        elif s == "bajar":
            if self.elev.mover(C.H_ABAJO):
                self.hist_dist = []
                self._set(Estado.RECOGER, "acercar")
        elif s == "acercar":
            v, w = self._seguir_linea(e, C.V_ACERCAR)
            self.hist_dist = (self.hist_dist + [dist])[-3:]
            if len(self.hist_dist) == 3 and np.mean(self.hist_dist) <= C.D_AGARRE:
                self._set(Estado.RECOGER, "cerrar")
        elif s == "cerrar":
            if self.t_sub < 2 * C.DT:                    # verificación geométrica al iniciar el cierre
                x, y, th = self.robot.pose
                rel = self.caja.c - [x, y]
                fx = rel[0] * np.cos(th) + rel[1] * np.sin(th) - C.X_SENSOR_DIST - C.CAJA_LADO / 2
                ly = -rel[0] * np.sin(th) + rel[1] * np.cos(th)
                if abs(ly) > (C.GAP_ABIERTA - C.CAJA_LADO) / 2:
                    self._finalizar(False, f"caja desalineada lateralmente ({ly*100:.1f} cm)")
                elif not (0.015 <= fx <= 0.06):
                    self._finalizar(False, f"caja fuera de alcance ({fx*100:.1f} cm)")
            if self.estado == Estado.RECOGER and self.pinza.mover(C.CAJA_LADO):
                self.caja.agarrada = True
                self._set(Estado.RECOGER, "subir")
        elif s == "subir":
            if self.elev.mover(C.H_ARRIBA) and self.elev.pos >= 0.02:
                self._set(Estado.TRANSPORTAR, "planear")
        return v, w

    # ---------- fase 4 y 5 de la lista del usuario: A* + Pure Pursuit ----------
    def _transportar(self):
        if self.sub == "planear":
            if self.ruta is None:                        # llamada al planeador "externo"
                inicio = celda_de(self.robot.pose[:2])
                camino, nexp = astar(self.malla, inicio, C.CELDA_B, h_manhattan)
                if camino is None:
                    self._finalizar(False, "A* sin solución")
                    return 0.0, 0.0
                self.nodos_expandidos = nexp
                esquinas = simplificar(camino)
                pts = [self.robot.pose[:2]] + [centro_celda(*c) for c in esquinas[1:]]
                d = pts[-1] - pts[-2]
                pts[-1] = pts[-1] - C.L_AGARRE * d / np.linalg.norm(d)   # la CAJA queda en B
                self.ruta = np.array(pts)
                self.camino_celdas = camino
            if self.t_sub >= C.LATENCIA_PLANEADOR:       # espera la respuesta (serial/WiFi)
                self.pp = PurePursuit(self.ruta, C.LOOKAHEAD, C.V_TRANSPORTE)
                self.t_sub = 0.0
                self.sub = "seguir"
            return 0.0, 0.0
        x, y, th = self.robot.pose
        v, w = self.pp.comando(x, y, th)                 # (usa pose; en físico: cámara/odometría)
        if self.pp.llego:
            self._set(Estado.DEPOSITAR, "bajar")
        return v, w

    # ---------- fase 5: depositar ----------
    def _depositar(self):
        s, v = self.sub, 0.0
        if s == "bajar":
            if self.elev.mover(C.H_ABAJO):
                self._set(Estado.DEPOSITAR, "abrir")
        elif s == "abrir":
            if self.pinza.mover(C.GAP_ABIERTA):
                self.caja.agarrada = False
                self.caja.z = 0.0
                self.recorrido_retro = 0.0
                self._set(Estado.DEPOSITAR, "retroceder")
        elif s == "retroceder":                          # sale de la caja con la pinza abierta
            v = -0.05
            self.recorrido_retro += abs(self.robot.v) * C.DT
            if self.recorrido_retro >= 0.06:
                v = 0.0
                self._set(Estado.DEPOSITAR, "recoger_mecanismo")
        elif s == "recoger_mecanismo":
            a = self.pinza.mover(C.GAP_CERRADA_VACIA)
            b = self.elev.mover(C.H_ARRIBA)
            if a and b:
                cB = centro_celda(*C.CELDA_B)
                err = np.linalg.norm(self.caja.c - cB)
                ok = err < C.CELDA / 2
                self._finalizar(ok, "" if ok else f"caja a {err*100:.1f} cm del centro de B")
        return v

    def _finalizar(self, ok, motivo=""):
        self.resultado, self.motivo = ok, motivo
        self.estado = Estado.FINALIZAR
        self.t_fin = self.t

    def _registrar(self, e, dist, v, w):
        L, rb = self.log, self.robot
        L["t"].append(self.t); L["x"].append(rb.x); L["y"].append(rb.y); L["th"].append(rb.th)
        L["estado"].append(self.estado.name if not self.sub else f"{self.estado.name}/{self.sub}")
        L["e_true"].append(float(self.arena.error_lateral(rb.pose[:2])[0]))
        L["e_med"].append(np.nan if e is None else e)
        L["dist"].append(dist); L["v"].append(rb.v); L["w"].append(rb.w)
        L["pinza"].append(self.pinza.pos); L["elev"].append(self.elev.pos)
        L["caja_x"].append(self.caja.c[0]); L["caja_y"].append(self.caja.c[1])

    def ejecutar(self):
        while self.estado != Estado.FINALIZAR:
            self.paso()
        for _ in range(int(0.5 / C.DT)):                 # medio segundo de robot detenido
            self.paso()
        return self.resultado
