#!/usr/bin/env python3
"""
Base Controller for PuzzleBot — v2
───────────────────────────────────
Handles: odom, goal/trajectory tracking, Lyapunov pub, phase data, sliding surface data.
Supports both single goal and multi-waypoint trajectory following.
"""
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, Float64MultiArray, String, Bool
from tf_transformations import euler_from_quaternion
import json


def wrap_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))

def sat(s, phi):
    if phi <= 0:
        return np.sign(s)
    return np.clip(s / phi, -1.0, 1.0)


class BaseController(Node):
    CONTROLLER_NAME = 'base'

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 3.0)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 1.5)
        self.declare_parameter('default_active', True)
        self.declare_parameter('waypoint_tolerance', 0.15)

        self.v_max = self.get_parameter('max_linear_vel').value
        self.w_max = self.get_parameter('max_angular_vel').value
        self.tol = self.get_parameter('goal_tolerance').value
        self.wp_tol = self.get_parameter('waypoint_tolerance').value
        rate = self.get_parameter('control_rate').value

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_odom = 0.0
        self.w_odom = 0.0

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_set = True
        self.arrived = False
        self.active = self.get_parameter('default_active').value

        # Trajectory waypoints
        self.waypoints = []      # list of (x, y)
        self.wp_index = 0
        self.trajectory_mode = False

        # Phase data history (for derivatives)
        self.prev_dist = 0.0
        self.prev_angle_err = 0.0
        self.dist_dot_filt = 0.0   # EMA low-pass filtered dist derivative
        self.angle_dot_filt = 0.0  # EMA low-pass filtered angle derivative

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lyapunov_pub = self.create_publisher(Float64, '/lyapunov', 10)
        self.ctrl_state_pub = self.create_publisher(String, '/controller_state', 10)
        self.arrived_pub = self.create_publisher(Bool, '/arrived', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)
        self.path_sub = self.create_subscription(Path, '/trajectory', self._path_cb, 10)
        self.switch_sub = self.create_subscription(String, '/switch_controller', self._switch_cb, 10)
        self.reset_sub = self.create_subscription(String, '/sim_reset', self._reset_cb, 10)

        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self._control_loop)
        self.ctrl_time = 0.0

        self.get_logger().info(
            f'[{self.CONTROLLER_NAME}] started (active={self.active})')

    # ── Callbacks ─────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.v_odom = msg.twist.twist.linear.x
        self.w_odom = msg.twist.twist.angular.z

    def _goal_cb(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_set = True
        self.arrived = False
        self.trajectory_mode = False
        self.waypoints = []
        self.on_new_goal()
        self.get_logger().info(f'[{self.CONTROLLER_NAME}] Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def _path_cb(self, msg: Path):
        """Receive a trajectory as nav_msgs/Path — list of waypoints."""
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(self.waypoints) > 0:
            self.wp_index = 0
            self.goal_x, self.goal_y = self.waypoints[0]
            self.trajectory_mode = True
            self.arrived = False
            self.on_new_goal()
            self.get_logger().info(
                f'[{self.CONTROLLER_NAME}] Trajectory: {len(self.waypoints)} waypoints')

    def _switch_cb(self, msg: String):
        requested = msg.data.strip()
        was_active = self.active
        self.active = (requested == self.CONTROLLER_NAME)
        if self.active and not was_active:
            self.on_new_goal()
            self.get_logger().info(f'[{self.CONTROLLER_NAME}] ACTIVATED')
        elif not self.active and was_active:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f'[{self.CONTROLLER_NAME}] deactivated')

    def _reset_cb(self, msg: String):
        self.ctrl_time = 0.0
        self.x = self.y = self.theta = 0.0
        self.v_odom = self.w_odom = 0.0
        self.arrived = False
        self.prev_dist = self.prev_angle_err = 0.0
        self.dist_dot_filt = self.angle_dot_filt = 0.0
        self.waypoints = []
        self.wp_index = 0
        self.trajectory_mode = False
        self.on_new_goal()
        self.cmd_pub.publish(Twist())
        self.get_logger().info(f'[{self.CONTROLLER_NAME}] RESET')

    # ── Errors ────────────────────────────────────────────────────────
    def get_errors(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = np.sqrt(dx*dx + dy*dy)
        angle_to_goal = np.arctan2(dy, dx)
        angle_err = wrap_angle(angle_to_goal - self.theta)
        return dist, angle_err, dx, dy

    # ── Waypoint advancement ──────────────────────────────────────────
    def _advance_waypoint(self, dist):
        if not self.trajectory_mode or len(self.waypoints) == 0:
            return
        tol = self.wp_tol if self.wp_index < len(self.waypoints) - 1 else self.tol
        if dist < tol and self.wp_index < len(self.waypoints) - 1:
            self.wp_index += 1
            self.goal_x, self.goal_y = self.waypoints[self.wp_index]
            self.on_new_goal()
            self.get_logger().info(
                f'[{self.CONTROLLER_NAME}] WP {self.wp_index}/{len(self.waypoints)}: '
                f'({self.goal_x:.2f}, {self.goal_y:.2f})')

    # ── Control loop ──────────────────────────────────────────────────
    def _control_loop(self):
        if not self.active:
            return

        # Once arrived, keep publishing zero and do nothing until a new goal arrives
        if self.arrived:
            self.cmd_pub.publish(Twist())
            return

        self.ctrl_time += self.dt
        dist, angle_err, dx, dy = self.get_errors()
        self._advance_waypoint(dist)
        # Re-evaluate errors after a possible waypoint advance so that the
        # arrival check and the control law both use the *current* goal.
        dist, angle_err, dx, dy = self.get_errors()

        # Phase derivatives — EMA low-pass filter (α=0.15 ≈ 15 Hz cutoff at 100 Hz)
        # Raw finite differences at 100 Hz amplify odometry noise; filter before using.
        raw_dist_dot = (dist - self.prev_dist) / self.dt if self.dt > 0 else 0.0
        raw_angle_dot = (angle_err - self.prev_angle_err) / self.dt if self.dt > 0 else 0.0
        _alpha = 0.15
        self.dist_dot_filt = _alpha * raw_dist_dot + (1.0 - _alpha) * self.dist_dot_filt
        self.angle_dot_filt = _alpha * raw_angle_dot + (1.0 - _alpha) * self.angle_dot_filt
        dist_dot = self.dist_dot_filt
        angle_dot = self.angle_dot_filt
        self.prev_dist = dist
        self.prev_angle_err = angle_err

        # Final waypoint arrival
        is_final = not self.trajectory_mode or self.wp_index >= len(self.waypoints) - 1
        if dist < self.tol and is_final:
            self.arrived = True
            self.cmd_pub.publish(Twist())
            self.arrived_pub.publish(Bool(data=True))
            V = self.compute_lyapunov(dist, angle_err, dx, dy)
            self.lyapunov_pub.publish(Float64(data=float(V)))
            self._publish_state(0, 0, V, dist, angle_err, dist_dot, angle_dot, 0, 0)
            return

        self.arrived = False
        self.arrived_pub.publish(Bool(data=False))

        # Compute control — subclass returns (v, w, V) or (v, w, V, extra)
        result = self.compute_control(dist, angle_err, dx, dy)
        if len(result) == 3:
            v, w, V = result
            s_v, s_w = 0.0, 0.0
        else:
            v, w, V, s_v, s_w = result

        v = np.clip(v, -self.v_max, self.v_max)
        w = np.clip(w, -self.w_max, self.w_max)
        # Smooth heading penalty: full speed when aligned, tapers via cosine,
        # floor at 15% so the robot can still translate to escape local oscillations.
        # Replaces the old hard cliff (if |angle_err|>0.8: v*=0.15) that caused
        # sawtooth oscillations in distance error.
        v_scale = max(0.15, float(np.cos(np.clip(abs(angle_err), 0.0, np.pi / 2))))
        v *= v_scale

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
        self.lyapunov_pub.publish(Float64(data=float(V)))
        self._publish_state(v, w, V, dist, angle_err, dist_dot, angle_dot, s_v, s_w)

    def _publish_state(self, v, w, V, dist, angle_err, dist_dot, angle_dot, s_v, s_w):
        state = {
            'ctrl': self.CONTROLLER_NAME,
            't': round(self.ctrl_time, 4),
            'x': round(self.x, 5), 'y': round(self.y, 5),
            'theta': round(self.theta, 5),
            'dist_err': round(dist, 5),
            'angle_err': round(angle_err, 5),
            'dist_dot': round(dist_dot, 5),
            'angle_dot': round(angle_dot, 5),
            'v': round(v, 5), 'w': round(w, 5),
            'V': round(V, 6),
            's_v': round(s_v, 5), 's_w': round(s_w, 5),
            'gx': round(self.goal_x, 3), 'gy': round(self.goal_y, 3),
            'wp': self.wp_index, 'wp_total': len(self.waypoints),
        }
        self.ctrl_state_pub.publish(String(data=json.dumps(state)))

    # ── Override in subclasses ────────────────────────────────────────
    def compute_control(self, dist, angle_err, dx, dy):
        raise NotImplementedError

    def compute_lyapunov(self, dist, angle_err, dx, dy):
        return 0.5 * dist**2 + 0.5 * angle_err**2

    def on_new_goal(self):
        pass
