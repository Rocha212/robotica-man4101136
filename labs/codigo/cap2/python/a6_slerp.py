#!/usr/bin/env python3
"""Actividad 6: interpolación esférica SLERP entre 0 y 90 grados en Z."""

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


def main():
    key_times = [0.0, 1.0]

    # Orientaciones inicial y final.
    key_rotations = R.from_euler('z', [0.0, 90.0], degrees=True)

    # Interpolador esférico entre ambas orientaciones.
    slerp = Slerp(key_times, key_rotations)

    # Cinco instantes equidistantes, como en el informe.
    times = np.linspace(0.0, 1.0, 5)
    interpolated_rotations = slerp(times)

    print(" t       Ángulo Z (grados)")
    for time_value, rotation in zip(times, interpolated_rotations):
        euler = rotation.as_euler('xyz', degrees=True)
        print(f"{time_value:.2f}       {euler[2]:.2f}")


if __name__ == '__main__':
    main()
