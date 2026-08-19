#!/usr/bin/env python3

import argparse
import csv
import math
import signal
import sys

import numpy as np
import rclpy
from rclpy.node import Node

import tf2_ros
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PPRunLogger(Node):
    def __init__(self, args):
        super().__init__('pp_run_logger')
        self.args = args

        data = np.loadtxt(args.csv_path, delimiter=',', skiprows=1)
        self.wx, self.wy = data[:, 0], data[:, 1]
        self.n = len(self.wx)
        self.get_logger().info(f"Raceline loaded for error computation: {self.n} waypoints.")

        self.pose = None
        self.last_drive = (0.0, 0.0)  # (steering, speed)
        self.rows = []

        self.pose_source = args.pose_source
        if self.pose_source == 'odom':
            self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 10)
        else:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(AckermannDriveStamped, '/drive', self.drive_cb, 10)
        self.create_timer(1.0 / args.log_rate, self.log_step)

        self.t0 = self.get_clock().now().nanoseconds * 1e-9

    def odom_cb(self, msg):
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        self.pose = (p.position.x, p.position.y, yaw)

    def update_pose_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.args.map_frame, self.args.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        tr = t.transform.translation
        q = t.transform.rotation
        self.pose = (tr.x, tr.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def drive_cb(self, msg):
        self.last_drive = (msg.drive.steering_angle, msg.drive.speed)

    def log_step(self):
        if self.pose_source != 'odom':
            self.update_pose_tf()
        if self.pose is None:
            return

        x, y, yaw = self.pose
        d2 = (self.wx - x) ** 2 + (self.wy - y) ** 2
        idx = int(np.argmin(d2))
        lateral_error = float(np.sqrt(d2[idx]))

        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        steering, speed = self.last_drive
        self.rows.append([t, x, y, yaw, steering, speed, lateral_error, idx, self.args.run_label])

    def flush_and_summarize(self):
        if not self.rows:
            self.get_logger().warn("No data logged (pose never became available?). Nothing written.")
            return
        with open(self.args.out, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t_s', 'x', 'y', 'yaw', 'steering_cmd_rad', 'speed_cmd_mps',
                              'lateral_error_m', 'nearest_idx', 'run_label'])
            writer.writerows(self.rows)

        errs = np.array([r[6] for r in self.rows])
        print(f"\n--- {self.args.out} ---")
        print(f"  samples: {len(errs)}   duration: {self.rows[-1][0]:.1f} s")
        print(f"  lateral error [m]: mean={errs.mean():.4f}  max={errs.max():.4f}  "
              f"p95={np.percentile(errs, 95):.4f}")
        print(f"  run_label: {self.args.run_label}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv_path', required=True, help='Same raceline CSV as pure_pursuit_node.')
    parser.add_argument('--pose_source', default='tf', choices=['tf', 'odom'])
    parser.add_argument('--map_frame', default='map')
    parser.add_argument('--base_frame', default='ego_racecar/base_link')
    parser.add_argument('--odom_topic', default='/ego_racecar/odom')
    parser.add_argument('--out', default='pp_run_log.csv')
    parser.add_argument('--run_label', default='')
    parser.add_argument('--log_rate', type=float, default=30.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = PPRunLogger(args)

    def on_sigint(sig, frame):
        node.flush_and_summarize()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush_and_summarize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
