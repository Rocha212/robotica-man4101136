import math
import random

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

V_LIN = 0.15            # m/s, velocidad lineal
W_TURN = 0.5            # rad/s, velocidad angular maxima al girar
OBST_DIST = 0.40        # m, distancia frontal para detenerse
SIDE_DIST = 0.18        # m, distancia lateral minima
FRONT_HALF_DEG = 30     # sector frontal (+- grados)
CLEAR_DIST = 1.0        # m, espacio libre para elegir una direccion
CAND_STEP_DEG = 15      # paso entre direcciones candidatas
CAND_WINDOW_DEG = 20    # ventana (+-) para medir el espacio libre
MIN_TURN_DEG = 45       # giro minimo al elegir direccion
PAUSE_TIME = 0.8        # s parado antes de girar
TURN_TIMEOUT = 40.0     # s maximo por giro
YAW_TOL = 0.04          # rad, tolerancia del giro
TICK = 0.05             # s, periodo de control


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class VacuumExplorer(Node):
    def __init__(self):
        super().__init__('vacuum_explorer')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(TICK, self.control)

        self.angles = None
        self.rs = None
        self.range_max = 3.5
        self.yaw = None
        self.target_yaw = 0.0

        self.state = 'FORWARD'
        self.state_t0 = self.get_clock().now()
        self.get_logger().info(
            f'Explorador aspiradora: v={V_LIN} m/s, w={W_TURN} rad/s')

    # ---------- sensores ----------
    def scan_cb(self, msg):
        n = len(msg.ranges)
        ang = msg.angle_min + np.arange(n) * msg.angle_increment
        self.angles = np.arctan2(np.sin(ang), np.cos(ang))
        rs = np.array(msg.ranges, dtype=float)
        self.range_max = msg.range_max if math.isfinite(msg.range_max) else 3.5
        bad = ~np.isfinite(rs) | (rs < msg.range_min)
        rs[bad] = self.range_max
        self.rs = np.minimum(rs, self.range_max)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ---------- utilidades ----------
    def sector_min(self, center, half):
        d = np.abs(np.arctan2(np.sin(self.angles - center),
                              np.cos(self.angles - center)))
        sel = self.rs[d <= half]
        return float(sel.min()) if sel.size else self.range_max

    def elapsed(self):
        return (self.get_clock().now() - self.state_t0).nanoseconds * 1e-9

    def set_state(self, new_state):
        self.state = new_state
        self.state_t0 = self.get_clock().now()
        self.get_logger().info(f'Estado: {new_state}')

    def send(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.pub.publish(msg)

    def pick_heading(self):
        cands = []
        half = math.radians(CAND_WINDOW_DEG)
        for deg in range(-180, 180, CAND_STEP_DEG):
            m = self.sector_min(math.radians(deg), half)
            cands.append((deg, m))
        good = [c for c in cands
                if c[1] > CLEAR_DIST and abs(c[0]) >= MIN_TURN_DEG]
        if good:
            return random.choice(good)[0]
        return max(cands, key=lambda c: c[1])[0]

    # ---------- control ----------
    def control(self):
        if self.angles is None or self.yaw is None:
            self.send(0.0, 0.0)
            return

        if self.state == 'FORWARD':
            front = self.sector_min(0.0, math.radians(FRONT_HALF_DEG))
            left = self.sector_min(math.radians(75), math.radians(30))
            right = self.sector_min(math.radians(-75), math.radians(30))
            if front < OBST_DIST or left < SIDE_DIST or right < SIDE_DIST:
                self.set_state('STOP')
            else:
                self.send(V_LIN, 0.0)

        elif self.state == 'STOP':
            self.send(0.0, 0.0)
            if self.elapsed() >= PAUSE_TIME:
                deg = self.pick_heading()
                self.target_yaw = wrap(self.yaw + math.radians(deg))
                self.get_logger().info(f'Girar {deg} grados')
                self.set_state('TURN')

        elif self.state == 'TURN':
            err = wrap(self.target_yaw - self.yaw)
            if abs(err) < YAW_TOL or self.elapsed() > TURN_TIMEOUT:
                self.send(0.0, 0.0)
                self.set_state('FORWARD')
            else:
                w = max(-W_TURN, min(W_TURN, 1.5 * err))
                if abs(w) < 0.08:
                    w = math.copysign(0.08, err)
                self.send(0.0, w)


def main(args=None):
    rclpy.init(args=args)
    node = VacuumExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
