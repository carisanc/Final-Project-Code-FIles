#!/usr/bin/env python3

# ============================================================
# scan_throttle.py — reentrega /scan a una tasa máxima fija.
#
# Por qué: gym_bridge publica /scan a 250 Hz (es el paso de física
# del sim, timer de 0.004 s). Un LiDAR real va a ~40 Hz. A 250 Hz el
# message filter de AMCL se satura ("discarding message because the
# queue is full") y la localización diverge. Este nodo se suscribe a
# /scan y republica en /scan_amcl a una tasa acotada (por defecto
# 40 Hz), preservando header/stamp/frame para que la sincronización
# TF de AMCL siga funcionando.
#
# Parámetros:
#   input_topic  (str, default /scan)
#   output_topic (str, default /scan_amcl)
#   rate_hz      (float, default 40.0) — tasa máxima de salida
# ============================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanThrottle(Node):
    def __init__(self):
        super().__init__('scan_throttle')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_amcl')
        self.declare_parameter('rate_hz', 40.0)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        rate_hz = self.get_parameter('rate_hz').value

        # Periodo mínimo entre publicaciones (s). Si rate_hz<=0, no limita.
        self.min_period = (1.0 / rate_hz) if rate_hz > 0.0 else 0.0
        self.last_pub_time = None

        self.pub = self.create_publisher(LaserScan, out_topic, 10)
        self.create_subscription(LaserScan, in_topic, self.cb, 10)

        self.get_logger().info(
            f"scan_throttle: {in_topic} -> {out_topic} @ max {rate_hz:.1f} Hz"
        )

    def cb(self, msg):
        # Usamos el reloj del nodo (wall clock; use_sim_time=false) para
        # decidir cuándo dejar pasar el siguiente scan.
        now = self.get_clock().now()
        if self.last_pub_time is not None:
            elapsed = (now - self.last_pub_time).nanoseconds * 1e-9
            if elapsed < self.min_period:
                return
        self.last_pub_time = now
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanThrottle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
