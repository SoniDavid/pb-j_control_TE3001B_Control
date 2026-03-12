#!/usr/bin/env python3
"""
Gazebo Mirror Node — Digital Twin Visualiser
─────────────────────────────────────────────
Subscribes to /odom (real encoder odometry from puzzlebot_hardware) and
teleports the Gazebo model to the same pose on every tick.

Gazebo becomes a pure visualisation layer — its physics engine is NOT used
for control. The real robot is the plant; Gazebo mirrors it.

Architecture:
  Real robot encoders
    → /motor_left/rpm, /motor_right/rpm
      → puzzlebot_hardware (odometry integration)
        → /odom
          → gazebo_mirror  ← THIS NODE
            → /gazebo/set_entity_state (Gazebo service)
              → Gazebo model teleports to match real pose
                → RViz / Gazebo GUI show real robot behaviour
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist


class GazeboMirror(Node):
    """Mirrors real /odom into the Gazebo simulation for visualisation."""

    def __init__(self):
        super().__init__('gazebo_mirror')

        self.declare_parameter('entity_name', 'puzzlebot')
        self.declare_parameter('reference_frame', '')   # '' = world frame
        self.declare_parameter('update_rate', 20.0)     # Hz cap (service calls are slow)

        self._entity   = self.get_parameter('entity_name').value
        self._ref_frame = self.get_parameter('reference_frame').value
        self._rate     = self.get_parameter('update_rate').value

        # Latest odometry to send
        self._latest_odom: Odometry | None = None
        self._dirty = False

        # Service client
        self._client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Subscribe to real odometry
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Timer to send updates at a capped rate (don't flood the service)
        self.create_timer(1.0 / self._rate, self._send)

        self.get_logger().info(
            f'[GazeboMirror] Waiting for /gazebo/set_entity_state service...')
        self._client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info(
            f'[GazeboMirror] Ready — mirroring /odom → Gazebo entity "{self._entity}"')

    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg
        self._dirty = True

    def _send(self):
        if not self._dirty or self._latest_odom is None:
            return
        if not self._client.service_is_ready():
            return

        self._dirty = False
        odom = self._latest_odom

        state = EntityState()
        state.name            = self._entity
        state.reference_frame = self._ref_frame

        # Copy pose from odometry
        state.pose = Pose()
        state.pose.position.x    = odom.pose.pose.position.x
        state.pose.position.y    = odom.pose.pose.position.y
        state.pose.position.z    = 0.05          # keep robot above ground
        state.pose.orientation   = odom.pose.pose.orientation

        # Copy body-frame velocity so Gazebo visuals are smooth
        state.twist = Twist()
        state.twist.linear.x  = odom.twist.twist.linear.x
        state.twist.angular.z = odom.twist.twist.angular.z

        req = SetEntityState.Request()
        req.state = state

        # Fire-and-forget async call — don't block the timer
        future = self._client.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn(
                    f'[GazeboMirror] set_entity_state failed: {result.status_message}',
                    throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().warn(
                f'[GazeboMirror] service call error: {e}',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
