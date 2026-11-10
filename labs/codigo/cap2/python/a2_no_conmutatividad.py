#!/usr/bin/env python3
"""Actividad 2: demuestra que el orden de las rotaciones importa."""

from scipy.spatial.transform import Rotation as R


def main():
    rotation_z = R.from_euler('z', 90, degrees=True)
    rotation_x = R.from_euler('x', 90, degrees=True)

    # Se conservan las dos composiciones planteadas en el informe.
    total_1 = rotation_z * rotation_x
    total_2 = rotation_x * rotation_z

    print("Composición 1 (Rz * Rx), cuaternión [x, y, z, w]:")
    print(total_1.as_quat())

    print("\nComposición 2 (Rx * Rz), cuaternión [x, y, z, w]:")
    print(total_2.as_quat())

    print("\n¿Son iguales las matrices?:", 
          (abs(total_1.as_matrix() - total_2.as_matrix()) < 1e-9).all())

    # Ángulo de la rotación relativa entre ambos resultados.
    relative_rotation = total_1.inv() * total_2
    print("Magnitud de la diferencia (rad):", relative_rotation.magnitude())
    print("Magnitud de la diferencia (grados):",
          relative_rotation.magnitude() * 180.0 / 3.141592653589793)

    print("\nNota: el orden de multiplicación de rotaciones no es conmutativo.")
    print("SciPy interpreta a * b como aplicar primero b y luego a.")


if __name__ == '__main__':
    main()
