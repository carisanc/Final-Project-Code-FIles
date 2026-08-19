#!/usr/bin/env python3

# ============================================================
# pure_pursuit_node.py — controlador geométrico Pure Pursuit.
#
# Sigue la raceline (CSV x,y,heading,kappa,v) usando la pose del
# auto en el marco `map`. Publica /drive (volante + velocidad).
#
# Ciclo (a control_rate Hz):
#   1. punto más cercano de la raceline -> v_target, ancla de búsqueda
#   2. lookahead adaptativo L = clip(k*v_target, L_min, L_max)
#   3. avanzar por la raceline (lazo) hasta distancia >= L -> goal point
#   4. goal al marco del auto -> y_body
#   5. curvatura del arco  gamma = 2*y_body / L_act^2
#      volante            delta = atan(wheelbase * gamma), clip a ±max
#   6. speed = v_target * speed_scale -> publicar /drive
#   7. marker del goal en /pursuit_goal (RViz)
#
# Pose:
#   pose_source=tf   -> lookup map->base_frame (salida de AMCL)
#   pose_source=odom -> /ego_racecar/odom (ground-truth del sim)
# ============================================================

import math

import numpy as np
import rclpy
from rclpy.node import Node

import tf2_ros
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def yaw_from_quat(x, y, z, w):
    """Yaw (rad) de un cuaternión (asumiendo roll=pitch≈0)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('csv_path', '')
        self.declare_parameter('pose_source', 'tf')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering', 0.4)
        self.declare_parameter('lookahead_k', 0.4)
        self.declare_parameter('lookahead_min', 0.8)
        self.declare_parameter('lookahead_max', 2.5)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('control_rate', 30.0)
        # Ventana de búsqueda LOCAL del punto más cercano (índices alrededor del
        # último). Evita que en horquillas/eses (donde la pista se pliega sobre
        # sí misma) el argmin global se enganche al tramo de vuelta equivocado.
        self.declare_parameter('nearest_search_back', 10)
        self.declare_parameter('nearest_search_fwd', 40)

        csv_path = self.get_parameter('csv_path').value
        self.pose_source = self.get_parameter('pose_source').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.k = float(self.get_parameter('lookahead_k').value)
        self.L_min = float(self.get_parameter('lookahead_min').value)
        self.L_max = float(self.get_parameter('lookahead_max').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.search_back = int(self.get_parameter('nearest_search_back').value)
        self.search_fwd = int(self.get_parameter('nearest_search_fwd').value)
        rate = float(self.get_parameter('control_rate').value)

        if not csv_path:
            raise RuntimeError("Parámetro 'csv_path' vacío: pásame el CSV de la raceline.")
        data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
        self.wx, self.wy = data[:, 0], data[:, 1]
        self.wv = data[:, 4]
        self.n = len(self.wx)
        self.get_logger().info(
            f"Raceline cargada: {self.n} waypoints. pose_source={self.pose_source}")

        # Pose actual (x, y, yaw) o None si aún no disponible.
        self.pose = None
        # Índice del último punto más cercano (ancla de la búsqueda local).
        # None hasta el primer ciclo, donde se hace una búsqueda global.
        self.last_idx = None

        if self.pose_source == 'odom':
            self.create_subscription(
                Odometry, self.get_parameter('odom_topic').value,
                self.odom_cb, 10)
        else:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.goal_pub = self.create_publisher(Marker, '/pursuit_goal', 10)

        self.create_timer(1.0 / rate, self.control_loop)

    # ----------------------------------------------------------
    def odom_cb(self, msg):
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation.x, p.orientation.y,
                            p.orientation.z, p.orientation.w)
        self.pose = (p.position.x, p.position.y, yaw)

    def update_pose_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        tr = t.transform.translation
        q = t.transform.rotation
        self.pose = (tr.x, tr.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    # ----------------------------------------------------------
    def control_loop(self):
        if self.pose_source != 'odom':
            self.update_pose_tf()
        if self.pose is None:
            self.publish_drive(0.0, 0.0)   # aún sin pose: quieto
            return

        x, y, yaw = self.pose

        # 1) punto más cercano — búsqueda LOCAL alrededor del último índice
        #    (global solo en el primer ciclo, para engancharse). Así, en los
        #    tramos donde la pista se pliega (horquillas/eses), no salta al
        #    ramal de vuelta que casualmente pasa cerca.
        if self.last_idx is None:
            d2 = (self.wx - x) ** 2 + (self.wy - y) ** 2
            i0 = int(np.argmin(d2))
        else:
            win = (self.last_idx +
                   np.arange(-self.search_back, self.search_fwd + 1)) % self.n
            d2 = (self.wx[win] - x) ** 2 + (self.wy[win] - y) ** 2
            i0 = int(win[int(np.argmin(d2))])
        self.last_idx = i0
        v_target = float(self.wv[i0])

        # 2) lookahead adaptativo
        L = min(self.L_max, max(self.L_min, self.k * v_target))

        # 3) avanzar por la raceline (lazo) hasta distancia >= L
        gi = i0
        for step in range(self.n):
            gi = (i0 + step) % self.n
            dist = math.hypot(self.wx[gi] - x, self.wy[gi] - y)
            if dist >= L:
                break
        gx, gy = self.wx[gi], self.wy[gi]
        L_act = math.hypot(gx - x, gy - y)
        if L_act < 1e-3:
            self.publish_drive(0.0, v_target * self.speed_scale)
            return

        # 4) goal al marco del auto
        dx, dy = gx - x, gy - y
        cy, sy = math.cos(yaw), math.sin(yaw)
        y_body = -sy * dx + cy * dy

        # 5) curvatura -> volante
        gamma = 2.0 * y_body / (L_act * L_act)
        delta = math.atan(self.wheelbase * gamma)
        delta = max(-self.max_steering, min(self.max_steering, delta))

        # 6) publicar
        self.publish_drive(delta, v_target * self.speed_scale)
        self.publish_goal(gx, gy)

    # ----------------------------------------------------------
    def publish_drive(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def publish_goal(self, gx, gy):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'pursuit_goal'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = Point(x=float(gx), y=float(gy), z=0.1)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.35
        m.color.r = 0.1
        m.color.g = 1.0
        m.color.b = 0.1
        m.color.a = 1.0
        self.goal_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
