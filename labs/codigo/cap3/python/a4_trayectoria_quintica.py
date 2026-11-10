#!/usr/bin/env python3
"""Actividad 4: generación y análisis de una trayectoria quíntica."""

import numpy as np
import roboticstoolbox as rtb


def main():
    # Movimiento de 0 a 1 rad durante 2 segundos, muestreado a 100 Hz.
    t = np.arange(0.0, 2.0 + 0.01, 0.01)
    q0 = 0.0
    qf = 1.0

    trajectory = rtb.tools.trajectory.quintic(q0, qf, t)

    print("=== Trayectoria quíntica ===")
    print("Primeras posiciones (rad):", trajectory.q[:5])
    print("Primeras velocidades (rad/s):", trajectory.qd[:5])
    print("Primeras aceleraciones (rad/s^2):", trajectory.qdd[:5])
    print("Velocidad máxima absoluta (rad/s):",
          np.max(np.abs(trajectory.qd)))
    print("Aceleración máxima absoluta (rad/s^2):",
          np.max(np.abs(trajectory.qdd)))

    # Gráficas de posición, velocidad y aceleración.
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
    axes[0].plot(t, trajectory.q)
    axes[0].set_ylabel("Posición (rad)")
    axes[0].grid(True)

    axes[1].plot(t, trajectory.qd)
    axes[1].set_ylabel("Velocidad (rad/s)")
    axes[1].grid(True)

    axes[2].plot(t, trajectory.qdd)
    axes[2].set_ylabel("Aceleración (rad/s²)")
    axes[2].set_xlabel("Tiempo (s)")
    axes[2].grid(True)

    fig.suptitle("Perfil de trayectoria quíntico")
    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
