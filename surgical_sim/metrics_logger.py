"""
metrics_logger.py
=================
Subscribes to /tool_pose and /target_pose, computes position/orientation
error and tip velocity, and writes to a CSV file for offline analysis.

Output: ~/surgical_sim_metrics.csv
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import csv
import os
import math
from datetime import datetime


OUTPUT_PATH = os.path.expanduser('~/surgical_sim_metrics.csv')
FIELDNAMES  = ['timestamp', 'target_x', 'target_y', 'target_z',
               'actual_x', 'actual_y', 'actual_z',
               'error_mm', 'velocity_mms', 'status']


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        self._target_pose  = None
        self._current_pose = None
        self._last_pos     = None
        self._last_time    = None

        self.create_subscription(PoseStamped, '/target_pose',      self._target_cb,  10)
        self.create_subscription(PoseStamped, '/tool_pose',        self._pose_cb,    10)
        self.create_subscription(String,      '/controller_status', self._status_cb, 10)

        self._status = 'IDLE'
        self._csv    = open(OUTPUT_PATH, 'w', newline='')
        self._writer = csv.DictWriter(self._csv, fieldnames=FIELDNAMES)
        self._writer.writeheader()

        self.create_timer(0.1, self._log_tick)   # 10 Hz logging
        self.get_logger().info(f'Metrics logger writing to {OUTPUT_PATH}')

    def _target_cb(self, msg):  self._target_pose  = msg
    def _pose_cb(self,   msg):  self._current_pose = msg
    def _status_cb(self, msg):  self._status = msg.data

    def _log_tick(self):
        if self._current_pose is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        cp  = self._current_pose.pose.position

        # velocity
        velocity_mms = 0.0
        if self._last_pos is not None and self._last_time is not None:
            dt = now - self._last_time
            if dt > 0:
                dp = np.array([cp.x - self._last_pos[0],
                               cp.y - self._last_pos[1],
                               cp.z - self._last_pos[2]])
                velocity_mms = np.linalg.norm(dp) / dt * 1000.0

        self._last_pos  = (cp.x, cp.y, cp.z)
        self._last_time = now

        # error
        error_mm = float('nan')
        tx = ty = tz = float('nan')
        if self._target_pose is not None:
            tp = self._target_pose.pose.position
            tx, ty, tz = tp.x, tp.y, tp.z
            error_mm = math.sqrt(
                (cp.x - tx)**2 + (cp.y - ty)**2 + (cp.z - tz)**2
            ) * 1000.0

        self._writer.writerow({
            'timestamp':   datetime.utcnow().isoformat(),
            'target_x':    f'{tx:.4f}',
            'target_y':    f'{ty:.4f}',
            'target_z':    f'{tz:.4f}',
            'actual_x':    f'{cp.x:.4f}',
            'actual_y':    f'{cp.y:.4f}',
            'actual_z':    f'{cp.z:.4f}',
            'error_mm':    f'{error_mm:.3f}',
            'velocity_mms':f'{velocity_mms:.2f}',
            'status':      self._status,
        })
        self._csv.flush()

    def destroy_node(self):
        self._csv.close()
        self.get_logger().info(f'Metrics saved to {OUTPUT_PATH}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
