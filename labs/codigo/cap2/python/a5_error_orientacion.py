#!/usr/bin/env python3
"""Actividad 5: cálculo del error entre orientación deseada y actual."""

from scipy.spatial.transform import Rotation as R


def main():
    # Orientación objetivo: 90 grados alrededor de Z.
    desired = R.from_euler('z', 90, degrees=True)

    # Orientación medida/actual: 45 grados alrededor de Z.
    actual = R.from_euler('z', 45, degrees=True)

    # Error relativo: q_error = q_des * inverse(q_actual).
    error = desired * actual.inv()

    print("Orientación deseada (Euler XYZ, grados):",
          desired.as_euler('xyz', degrees=True))
    print("Orientación actual (Euler XYZ, grados):",
          actual.as_euler('xyz', degrees=True))
    print("Error (Euler XYZ, grados):",
          error.as_euler('xyz', degrees=True))
    print("Magnitud del error (grados):",
          error.magnitude() * 180.0 / 3.141592653589793)


if __name__ == '__main__':
    main()
