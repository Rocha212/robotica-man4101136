import math


def wrap_angle(angle: float) -> float:
    """Normaliza un ángulo al intervalo [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def yaw_from_quaternion(q) -> float:
    """Extrae yaw de un cuaternión ROS."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
