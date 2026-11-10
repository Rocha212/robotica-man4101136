#!/usr/bin/env python3
"""Actividad 6: interpolación de orientación mediante SLERP."""

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp


def main():
    # Orientaciones inicial y final: 0 y 90 grados alrededor de Z.
    rotation_initial = R.from_euler('z', 0.0, degrees=True)
    rotation_final = R.from_euler('z', 90.0, degrees=True)

    key_times = [0.0, 1.0]
    key_rotations = R.concatenate([rotation_initial, rotation_final])

    interpolator = Slerp(key_times, key_rotations)
    times = np.linspace(0.0, 1.0, 5)
    interpolated = interpolator(times)

    print("Ángulos Euler XYZ (grados):")
    print(interpolated.as_euler('xyz', degrees=True))

    print("\nCuaterniones [qx, qy, qz, qw]:")
    print(interpolated.as_quat())


if __name__ == '__main__':
    main()
