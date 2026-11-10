#!/usr/bin/env python3
"""
Actividad 1: integración discreta de velocidad angular.

Primero integra una velocidad angular constante en Z durante 100 pasos.
Después integra velocidades angulares simultáneas en X, Y y Z durante
200 pasos. La orientación se representa con scipy.spatial.transform.Rotation.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def integrate_angular_velocity(angular_velocity, dt, iterations):
    """Integra una velocidad angular constante mediante incrementos rotacionales."""
    rotation = R.identity()
    angular_velocity = np.asarray(angular_velocity, dtype=float)

    for _ in range(iterations):
        delta_rotation = R.from_rotvec(angular_velocity * dt)
        rotation = rotation * delta_rotation

    return rotation


def main():
    dt = 0.01

    # Caso 1: rotación alrededor del eje Z.
    omega_z = np.array([0.0, 0.0, 1.0])
    rotation_z = integrate_angular_velocity(omega_z, dt, 100)

    print("=== Integración en un solo eje ===")
    print("Velocidad angular [wx, wy, wz] (rad/s):", omega_z)
    print("Tiempo total (s):", 100 * dt)
    print("Ángulos Euler XYZ (grados):",
          rotation_z.as_euler('xyz', degrees=True))
    print("Cuaternión [qx, qy, qz, qw]:", rotation_z.as_quat())

    # Caso 2: rotación simultánea alrededor de los tres ejes.
    omega_xyz = np.array([0.1, 0.2, 0.3])
    rotation_xyz = integrate_angular_velocity(omega_xyz, dt, 200)

    print("\n=== Integración en tres ejes ===")
    print("Velocidad angular [wx, wy, wz] (rad/s):", omega_xyz)
    print("Tiempo total (s):", 200 * dt)
    print("Cuaternión final [qx, qy, qz, qw]:",
          rotation_xyz.as_quat())
    print("Ángulos Euler XYZ equivalentes (grados):",
          rotation_xyz.as_euler('xyz', degrees=True))


if __name__ == '__main__':
    main()
