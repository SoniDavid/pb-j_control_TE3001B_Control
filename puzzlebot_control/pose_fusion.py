#!/usr/bin/env python3
"""
Pose Fusion Node — Odometry + Gazebo Ground Truth
───────────────────────────────────────────────────
Implements a complementary filter that fuses two pose sources:

  Source 1: /odom  (integrated wheel odometry — low noise, drifts over time)
  Source 2: /gazebo/model_states  (Gazebo absolute ground truth — no drift)

Complementary filter update at each step:

  fused_k = fused_{k-1} + Δodom_k          ← integrate odometry delta (high-pass)
  fused_k += (1 − α) · (gt_k − fused_k)    ← pull toward GT   (low-pass correction)

  Simplifies to:  fused_k = α · odom_k + (1−α) · gt_k
  when both sources are absolute.

  α → 1.0  : trust odometry entirely (no drift correction)
  α → 0.0  : trust Gazebo GT entirely (may be jerky/noisy)
  Default α = 0.85  (smooth, drift-corrected in ~5–10 s)

Published topics
────────────────
  /pose_gt     (nav_msgs/Odometry) — Gazebo ground truth reformatted as Odometry
  /pose_fused  (nav_msgs/Odometry) — complementary filter fused estimate
  /fusion_state (std_msgs/String)  — JSON with per-axis errors and norms, for dashboard

The fused topic can be used in place of /odom by remapping:
  ros2 run puzzlebot_control pid_controller --ros-args -r /odom:=/pose_fused

Or enable 'remap_odom' parameter to have this node republish /odom directly.
"""

import rclpy
import numpy as np
import json
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def _wrap(a: float) -> float:
    return float(np.arctan2(np.sin(a), np.cos(a)))


class PoseFusion(Node):
    """Complementary filter: odometry (high-pass) + Gazebo GT (low-pass)."""

    def __init__(self):
        super().__init__('pose_fusion')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('alpha', 0.85)
        # Name of the Gazebo model to track (must match spawn entity name)
        self.declare_parameter('model_name', 'puzzlebot')
        # If True, republish /odom with the fused pose so controllers use it
        self.declare_parameter('remap_odom', False)
        self.declare_parameter('publish_rate', 50.0)

        self._alpha      = self.get_parameter('alpha').value
        self._model_name = self.get_parameter('model_name').value
        self._remap_odom = self.get_parameter('remap_odom').value
        rate             = self.get_parameter('publish_rate').value

        # ── Internal state ────────────────────────────────────────────
        # Odometry source
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_th: float = 0.0
        self._odom_v: float = 0.0
        self._odom_w: float = 0.0
        self._prev_odom_x: float | None = None
        self._prev_odom_y: float | None = None
        self._prev_odom_th: float | None = None
        self._odom_stamp = None

        # Gazebo ground truth source
        self._gt_x: float = 0.0
        self._gt_y: float = 0.0
        self._gt_th: float = 0.0
        self._gt_v: float = 0.0
        self._gt_w: float = 0.0
        self._gt_stamp = None
        self._gt_received: bool = False

        # Fused state (initialised from first odom)
        self._fused_x: float = 0.0
        self._fused_y: float = 0.0
        self._fused_th: float = 0.0
        self._fused_v: float = 0.0
        self._fused_w: float = 0.0
        self._initialised: bool = False

        # ── Publishers ────────────────────────────────────────────────
        self._gt_pub     = self.create_publisher(Odometry, '/pose_gt',     10)
        self._fused_pub  = self.create_publisher(Odometry, '/pose_fused',  10)
        self._state_pub  = self.create_publisher(String,   '/fusion_state', 10)

        if self._remap_odom:
            self._odom_remap_pub = self.create_publisher(Odometry, '/odom', 10)
            self.get_logger().warn(
                '[PoseFusion] remap_odom=True — will OVERRIDE /odom with fused pose. '
                'Make sure puzzlebot_sim / hardware is not also publishing /odom.')

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Odometry,    '/odom',                  self._odom_cb,  10)
        self.create_subscription(ModelStates, '/gazebo/model_states',   self._gt_cb,    10)

        # ── Timer ─────────────────────────────────────────────────────
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._update)

        self.get_logger().info(
            f'[PoseFusion] α={self._alpha}, model="{self._model_name}", '
            f'remap_odom={self._remap_odom}')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self._odom_th = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._odom_v = msg.twist.twist.linear.x
        self._odom_w = msg.twist.twist.angular.z
        self._odom_stamp = msg.header.stamp

    def _gt_cb(self, msg: ModelStates):
        """Extract the tracked model's pose from /gazebo/model_states."""
        try:
            idx = msg.name.index(self._model_name)
        except ValueError:
            # Model not yet spawned — skip silently
            return

        pose: Pose = msg.pose[idx]
        twist = msg.twist[idx]

        self._gt_x = pose.position.x
        self._gt_y = pose.position.y
        q = pose.orientation
        _, _, self._gt_th = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._gt_v = twist.linear.x
        self._gt_w = twist.angular.z
        self._gt_stamp = self.get_clock().now().to_msg()
        self._gt_received = True

    # ── Fusion update ─────────────────────────────────────────────────

    def _update(self):
        if self._odom_stamp is None:
            return  # no odom yet

        # ── Initialise fused state on first tick ──────────────────────
        if not self._initialised:
            self._fused_x  = self._odom_x
            self._fused_y  = self._odom_y
            self._fused_th = self._odom_th
            self._prev_odom_x  = self._odom_x
            self._prev_odom_y  = self._odom_y
            self._prev_odom_th = self._odom_th
            self._initialised = True
            return

        # ── Step 1: propagate with odometry delta (high-pass) ─────────
        delta_x  = self._odom_x  - self._prev_odom_x
        delta_y  = self._odom_y  - self._prev_odom_y
        delta_th = _wrap(self._odom_th - self._prev_odom_th)

        self._fused_x  += delta_x
        self._fused_y  += delta_y
        self._fused_th  = _wrap(self._fused_th + delta_th)

        self._prev_odom_x  = self._odom_x
        self._prev_odom_y  = self._odom_y
        self._prev_odom_th = self._odom_th

        # ── Step 2: pull toward GT correction (low-pass) ──────────────
        if self._gt_received:
            gain = 1.0 - self._alpha          # 0.15 at default α=0.85
            self._fused_x  += gain * (self._gt_x  - self._fused_x)
            self._fused_y  += gain * (self._gt_y  - self._fused_y)
            # Heading correction must respect wrap-around
            th_err = _wrap(self._gt_th - self._fused_th)
            self._fused_th = _wrap(self._fused_th + gain * th_err)

        # Velocity: complementary blend of odom and GT velocities
        if self._gt_received:
            self._fused_v = self._alpha * self._odom_v + (1.0 - self._alpha) * self._gt_v
            self._fused_w = self._alpha * self._odom_w + (1.0 - self._alpha) * self._gt_w
        else:
            self._fused_v = self._odom_v
            self._fused_w = self._odom_w

        # ── Publish ───────────────────────────────────────────────────
        stamp = self.get_clock().now().to_msg()

        if self._gt_received:
            self._gt_pub.publish(self._make_odom(
                self._gt_x, self._gt_y, self._gt_th,
                self._gt_v, self._gt_w, stamp))

        fused_msg = self._make_odom(
            self._fused_x, self._fused_y, self._fused_th,
            self._fused_v, self._fused_w, stamp)
        self._fused_pub.publish(fused_msg)

        if self._remap_odom:
            self._odom_remap_pub.publish(fused_msg)

        self._publish_state(stamp)

    # ── Helpers ───────────────────────────────────────────────────────

    def _make_odom(self, x, y, th, v, w, stamp) -> Odometry:
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, float(th))
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x  = float(v)
        msg.twist.twist.angular.z = float(w)
        return msg

    def _publish_state(self, stamp):
        # Errors: odom vs GT and fused vs GT
        if not self._gt_received:
            return

        odom_ex  = self._odom_x  - self._gt_x
        odom_ey  = self._odom_y  - self._gt_y
        odom_eth = _wrap(self._odom_th - self._gt_th)
        odom_dist = float(np.hypot(odom_ex, odom_ey))

        fused_ex  = self._fused_x  - self._gt_x
        fused_ey  = self._fused_y  - self._gt_y
        fused_eth = _wrap(self._fused_th - self._gt_th)
        fused_dist = float(np.hypot(fused_ex, fused_ey))

        state = {
            # Ground truth
            'gt_x':  round(self._gt_x,  5),
            'gt_y':  round(self._gt_y,  5),
            'gt_th': round(self._gt_th, 5),
            # Odometry vs GT error
            'odom_ex':   round(odom_ex,   5),
            'odom_ey':   round(odom_ey,   5),
            'odom_eth':  round(odom_eth,  5),
            'odom_dist': round(odom_dist, 5),
            # Fused vs GT error (should be smaller)
            'fused_ex':   round(fused_ex,   5),
            'fused_ey':   round(fused_ey,   5),
            'fused_eth':  round(fused_eth,  5),
            'fused_dist': round(fused_dist, 5),
            # Alpha used
            'alpha': self._alpha,
        }
        self._state_pub.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args)
    node = PoseFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
