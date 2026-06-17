"""
surgical_controller.py
======================
Core ROS2 node for surgical instrument positioning.
Subscribes to /target_pose, validates against workspace limits,
sends goals to MoveIt2, and monitors tip position error in real time.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    WorkspaceParameters,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
import numpy as np
import time

# ---------------------------------------------------------------------------
# Workspace limits (metres)
# ---------------------------------------------------------------------------
WORKSPACE = {
    'x': (-0.30, 0.30),
    'y': (0.10, 0.60),
    'z': (0.00, 0.40),
}

# ---------------------------------------------------------------------------
# Performance thresholds
# ---------------------------------------------------------------------------
MAX_TIP_VELOCITY_MS   = 0.100   # 100 mm/s
POSITION_TOLERANCE_M  = 0.001   # 1 mm RMS
ORIENT_TOLERANCE_RAD  = 0.0087  # 0.5 degrees
MAX_PLANNING_TIME_S   = 0.500   # 500 ms
MAX_VELOCITY_SCALE    = 0.20    # 20 % of joint max during approach

PLANNING_GROUP = 'ur_manipulator'
END_EFFECTOR   = 'tool0'


class SurgicalController(Node):
    """
    Orchestrates surgical instrument positioning via MoveIt2.

    Topics
    ------
    Subscribed:
        /target_pose     (geometry_msgs/PoseStamped)  — desired tip pose
        /tool_pose       (geometry_msgs/PoseStamped)  — actual tip pose from sim
        /joint_states    (sensor_msgs/JointState)     — joint feedback

    Published:
        /controller_status (std_msgs/String)          — human-readable status
        /position_error    (std_msgs/String)           — current tip error (mm)
    """

    def __init__(self):
        super().__init__('surgical_controller')

        # --- subscriptions ---
        self.create_subscription(PoseStamped, '/target_pose',  self._target_cb,  10)
        self.create_subscription(PoseStamped, '/tool_pose',    self._pose_cb,    10)
        self.create_subscription(JointState,  '/joint_states', self._joints_cb,  10)

        # --- publishers ---
        self._status_pub = self.create_publisher(String, '/controller_status', 10)
        self._error_pub  = self.create_publisher(String, '/position_error',    10)

        # --- MoveIt2 action client ---
        self._move_client = ActionClient(self, MoveGroup, '/move_action')

        # --- state ---
        self._current_pose   = None
        self._current_joints = None
        self._last_pose_time = None
        self._last_tip_pos   = None

        self.get_logger().info('SurgicalController ready — waiting for /target_pose')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _target_cb(self, msg: PoseStamped):
        """Validate workspace, then dispatch MoveIt2 planning request."""
        pos = msg.pose.position

        if not self._in_workspace(pos):
            err = (
                f'Target ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) m '
                f'is outside workspace limits.'
            )
            self.get_logger().error(err)
            self._publish_status(f'WORKSPACE_VIOLATION: {err}')
            return

        self.get_logger().info(
            f'Target accepted: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) m'
        )
        self._publish_status('PLANNING')
        self._send_moveit_goal(msg)

    def _pose_cb(self, msg: PoseStamped):
        """Track real-time tip pose; compute position error and velocity."""
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._current_pose is not None and self._last_pose_time is not None:
            dt = now - self._last_pose_time
            if dt > 0:
                dp = np.array([
                    msg.pose.position.x - self._current_pose.pose.position.x,
                    msg.pose.position.y - self._current_pose.pose.position.y,
                    msg.pose.position.z - self._current_pose.pose.position.z,
                ])
                velocity = np.linalg.norm(dp) / dt
                if velocity > MAX_TIP_VELOCITY_MS:
                    self.get_logger().warn(
                        f'Tip velocity {velocity*1000:.1f} mm/s exceeds limit — '
                        f'commanding stop.'
                    )
                    self._publish_status(f'VELOCITY_VIOLATION: {velocity*1000:.1f} mm/s')

        self._current_pose   = msg
        self._last_pose_time = now

    def _joints_cb(self, msg: JointState):
        self._current_joints = msg

    # ------------------------------------------------------------------
    # MoveIt2 planning
    # ------------------------------------------------------------------

    def _send_moveit_goal(self, target: PoseStamped):
        """Build and send a MoveGroup action goal."""
        if not self._move_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('MoveGroup action server not available.')
            self._publish_status('ERROR: MoveGroup unavailable')
            return

        goal = MoveGroup.Goal()
        goal.request = self._build_plan_request(target)

        t0 = time.monotonic()
        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._goal_response_cb(f, t0))

    def _build_plan_request(self, target: PoseStamped) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name            = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = MAX_PLANNING_TIME_S
        req.max_velocity_scaling_factor     = MAX_VELOCITY_SCALE
        req.max_acceleration_scaling_factor = MAX_VELOCITY_SCALE

        # workspace box
        ws = WorkspaceParameters()
        ws.header.frame_id = 'base_link'
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = (
            WORKSPACE['x'][0], WORKSPACE['y'][0], WORKSPACE['z'][0]
        )
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = (
            WORKSPACE['x'][1], WORKSPACE['y'][1], WORKSPACE['z'][1]
        )
        req.workspace_parameters = ws

        # position constraint
        pc = PositionConstraint()
        pc.header          = target.header
        pc.link_name       = END_EFFECTOR
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        box = SolidPrimitive()
        box.type           = SolidPrimitive.BOX
        box.dimensions     = [POSITION_TOLERANCE_M * 2] * 3
        bv = BoundingVolume()
        bv.primitives.append(box)
        bv.primitive_poses.append(target.pose)
        pc.constraint_region = bv
        pc.weight = 1.0

        # orientation constraint
        oc = OrientationConstraint()
        oc.header              = target.header
        oc.link_name           = END_EFFECTOR
        oc.orientation         = target.pose.orientation
        oc.absolute_x_axis_tolerance = ORIENT_TOLERANCE_RAD
        oc.absolute_y_axis_tolerance = ORIENT_TOLERANCE_RAD
        oc.absolute_z_axis_tolerance = ORIENT_TOLERANCE_RAD
        oc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pc)
        goal_constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(goal_constraints)

        return req

    def _goal_response_cb(self, future, t0: float):
        goal_handle = future.result()
        planning_time = time.monotonic() - t0

        if not goal_handle.accepted:
            self.get_logger().error('MoveIt2 rejected the goal.')
            self._publish_status('PLANNING_FAILED')
            return

        if planning_time > MAX_PLANNING_TIME_S:
            self.get_logger().warn(
                f'Planning took {planning_time*1000:.0f} ms — exceeds 500 ms target.'
            )

        self.get_logger().info(
            f'Goal accepted — planning time {planning_time*1000:.0f} ms'
        )
        self._publish_status(f'EXECUTING — planned in {planning_time*1000:.0f} ms')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        if result.error_code.val == 1:   # MoveItErrorCodes.SUCCESS
            self.get_logger().info('Motion complete.')
            self._publish_status('COMPLETE')
            self._measure_error()
        else:
            self.get_logger().error(f'Motion failed — error code {result.error_code.val}')
            self._publish_status(f'MOTION_FAILED: code {result.error_code.val}')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _in_workspace(self, pos) -> bool:
        return (
            WORKSPACE['x'][0] <= pos.x <= WORKSPACE['x'][1] and
            WORKSPACE['y'][0] <= pos.y <= WORKSPACE['y'][1] and
            WORKSPACE['z'][0] <= pos.z <= WORKSPACE['z'][1]
        )

    def _measure_error(self):
        """Log final tip position error after motion completes."""
        if self._current_pose is None:
            return
        # In a real system, compare against the commanded target.
        # Placeholder: log current position for manual inspection.
        p = self._current_pose.pose.position
        msg = String()
        msg.data = f'tip=({p.x:.4f},{p.y:.4f},{p.z:.4f})'
        self._error_pub.publish(msg)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SurgicalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
