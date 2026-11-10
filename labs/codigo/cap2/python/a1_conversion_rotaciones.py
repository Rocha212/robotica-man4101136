#!/usr/bin/env python3
"""Actividad 1: conversión entre Euler, cuaterniones y matriz de rotación."""

from scipy.spatial.transform import Rotation as R


def main():
    # Rotación de 90 grados alrededor del eje Z.
    rotation = R.from_euler('z', 90, degrees=True)

    # SciPy representa los cuaterniones en el orden [x, y, z, w].
    quaternion = rotation.as_quat()
    rotation_matrix = rotation.as_matrix()

    print("Rotación: 90 grados alrededor de Z")
    print("Cuaternión [x, y, z, w]:", quaternion)
    print("Matriz de rotación:")
    print(rotation_matrix)


if __name__ == '__main__':
    main()
