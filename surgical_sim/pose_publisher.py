"""
pose_publisher.py
=================
Interactive CLI node for publishing target poses to /target_pose.
Use this for manual testing during development.

Usage:
    ros2 run surgical_sim pose_publisher
    > Enter x y z (metres) or 'home' or 'q' to quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import threading


HOME_POSE = {'x': 0.0, 'y': 0.35, 'z': 0.20}


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self._pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.get_logger().info(
            "Pose publisher ready. Enter 'x y z' in metres, 'home', or 'q'."
        )
        self._thread = threading.Thread(target=self._cli_loop, daemon=True)
        self._thread.start()

    def _build_msg(self, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        # Vertical approach orientation (tool pointing down)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 1.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0
        return msg

    def _cli_loop(self):
        while rclpy.ok():
            try:
                raw = input('> ').strip().lower()
            except EOFError:
                break

            if raw in ('q', 'quit', 'exit'):
                break
            if raw == 'home':
                self._pub.publish(self._build_msg(**HOME_POSE))
                self.get_logger().info(f'Published home pose: {HOME_POSE}')
                continue
            parts = raw.split()
            if len(parts) != 3:
                print('Usage: x y z  (e.g. 0.1 0.3 0.15)')
                continue
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                print('Invalid numbers.')
                continue
            self._pub.publish(self._build_msg(x, y, z))
            self.get_logger().info(f'Published target ({x:.3f}, {y:.3f}, {z:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
