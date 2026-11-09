"""config.py - Parámetros globales del proyecto (unidades SI: m, s, rad)."""
from pathlib import Path

# ---------- Carpetas (relativas a ~/proyecto_final_robotica) ----------
RAIZ = Path(__file__).resolve().parents[2]
DIR_GRAFICAS = RAIZ / "graficas"
DIR_VIDEOS = RAIZ / "videos"
DIR_DATOS = RAIZ / "datos"
for _d in (DIR_GRAFICAS, DIR_VIDEOS, DIR_DATOS):
    _d.mkdir(parents=True, exist_ok=True)

# ---------- Tiempo ----------
DT = 0.02                      # paso de simulación y de control (50 Hz)

# ---------- Arena ----------
N_CELDAS = 6                   # 6x6 (cambiar a 8 para 8x8)
CELDA = 0.20                   # lado de celda [m]
LADO = N_CELDAS * CELDA        # lado de la arena [m]
ANCHO_LINEA = 0.02             # ancho de la línea negra [m]
CAJA_LADO = 0.05               # caja de 5x5x5 cm
CELDA_A = (0, 0)               # celda de inicio (col, fila)
CELDA_CAJA = (2, 0)            # celda donde está la caja (sobre la línea)
CELDA_B = (5, 5)               # celda de depósito
# Vértices de la línea en celdas (la línea se suaviza en las esquinas)
VERTICES_LINEA = [(0, 0), (3, 0), (3, 2), (5, 2), (5, 5)]
# Celdas bloqueadas para A* (obstáculos extra en la malla)
CELDAS_BLOQUEADAS = [(3, 1), (3, 2), (2, 3), (4, 4), (1, 3)]

# ---------- Robot diferencial ----------
RADIO_RUEDA = 0.0325           # [m]
DIST_RUEDAS = 0.11             # separación entre ruedas W [m]
V_MAX_RUEDA = 0.30             # velocidad máx. por rueda [m/s]
TAU_MOTOR = 0.08               # constante de tiempo del motor [s]
ASIMETRIA_MOTORES = (1.00, 0.97)   # (izq, der): los motores reales no son idénticos
RUIDO_RUEDA = 0.02             # ruido multiplicativo (2 %) en cada rueda
LARGO_ROBOT, ANCHO_ROBOT = 0.14, 0.12

# ---------- Sensor de línea TCRT5000 x5 ----------
X_SENSOR_LINEA = 0.07          # distancia del arreglo al eje de las ruedas [m]
N_CANALES = 5
PASO_CANAL = 0.012             # separación entre canales [m]
RUIDO_LINEA = 0.02             # desviación estándar de la lectura normalizada
UMBRAL_LINEA = 0.15

# ---------- Sensor de distancia VL53L0X ----------
X_SENSOR_DIST = 0.07           # posición frontal del sensor [m]
RANGO_MIN, RANGO_MAX = 0.03, 2.0
RUIDO_DIST_BASE, RUIDO_DIST_REL = 0.004, 0.01   # sigma = 4 mm + 1 % de la distancia

# ---------- Pinza (eje X) y elevador (eje Y): servos SG90 ----------
GAP_ABIERTA = 0.075            # apertura de la pinza abierta [m]
GAP_CERRADA_VACIA = 0.040      # pinza cerrada sin caja [m]
H_ABAJO, H_ARRIBA = 0.0, 0.030 # altura del elevador [m] (>= 0.02 exigido)
VEL_PINZA, VEL_ELEV = 0.08, 0.06   # velocidad de los servos [m/s]

# ---------- Control de línea (PID) ----------
KP, KI, KD = 100.0, 40.0, 1.5
OMEGA_MAX = 6.0
V_LINEA = 0.15

# ---------- Misión ----------
D_DETECCION = 0.15             # la caja se detecta a <= 15 cm
D_AGARRE = 0.04                # distancia sensor-caja para cerrar la pinza
A_FRENADO = 0.30               # desaceleración controlada [m/s^2]
V_ACERCAR = 0.04
L_AGARRE = X_SENSOR_DIST + D_AGARRE + CAJA_LADO / 2   # sensor -> centro de la caja
V_TRANSPORTE = 0.15
LOOKAHEAD = 0.12
LATENCIA_PLANEADOR = 0.30      # retardo serial/WiFi hacia el computador externo [s]
T_MAX_MISION = 60.0
