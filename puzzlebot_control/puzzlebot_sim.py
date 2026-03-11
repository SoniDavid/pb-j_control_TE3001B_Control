#!/usr/bin/env python3
"""
PuzzleBot Nonholonomic Simulator
─────────────────────────────────
Differential-drive kinematics with Pfaffian constraint:
    ẏ cosθ − ẋ sinθ = 0

Accepts cmd_vel (Twist) and terrain perturbations.
Publishes: /odom (Odometry), /pose (Pose), /wl, /wr, /sim_state
"""
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, String
from geometry_msgs.msg import Twist, Pose, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import tf2_ros
import json


class PuzzleBotSim(Node):
    def __init__(self):
        super().__init__('puzzlebot_sim')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('mass', 0.5)
        self.declare_parameter('inertia', 0.01)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 3.0)
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)

        self.L = self.get_parameter('wheel_base').value
        self.r = self.get_parameter('wheel_radius').value
        self.m = self.get_parameter('mass').value
        self.J = self.get_parameter('inertia').value
        self.v_max = self.get_parameter('max_linear_vel').value
        self.w_max = self.get_parameter('max_angular_vel').value
        self.dt = self.get_parameter('dt').value

        # ── State ─────────────────────────────────────────────────────
        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.theta = self.get_parameter('initial_theta').value
        self.v = 0.0
        self.w = 0.0

        # Commanded velocities
        self.cmd_v = 0.0
        self.cmd_w = 0.0

        # Perturbation forces (from terrain node)
        self.perturb_v = 0.0
        self.perturb_w = 0.0

        # Damping (physical)
        self.d_v = 0.3   # linear damping [N·s/m]
        self.d_w = 0.1   # angular damping [N·m·s/rad]

        # ── Publishers ────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pose_pub = self.create_publisher(Pose, '/pose', 10)
        self.wl_pub = self.create_publisher(Float32, '/VelocityEncL', 10)
        self.wr_pub = self.create_publisher(Float32, '/VelocityEncR', 10)
        self.state_pub = self.create_publisher(String, '/sim_state', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Subscribers ───────────────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.perturb_sub = self.create_subscription(
            Vector3, '/terrain_perturbation', self.perturb_cb, 10)
        self.reset_sub = self.create_subscription(
            String, '/sim_reset', self.reset_cb, 10)

        # ── Timer ─────────────────────────────────────────────────────
        self.timer = self.create_timer(self.dt, self.step)
        self.sim_time = 0.0

        self.get_logger().info(
            f'PuzzleBot Sim started: L={self.L}m, r={self.r}m, dt={self.dt}s')

    # ── Callbacks ─────────────────────────────────────────────────────
    def cmd_vel_cb(self, msg: Twist):
        self.cmd_v = np.clip(msg.linear.x, -self.v_max, self.v_max)
        self.cmd_w = np.clip(msg.angular.z, -self.w_max, self.w_max)

    def perturb_cb(self, msg: Vector3):
        self.perturb_v = msg.x   # force perturbation along body x
        self.perturb_w = msg.z   # torque perturbation about body z

    def reset_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.x = data.get('x', 0.0)
            self.y = data.get('y', 0.0)
            self.theta = data.get('theta', 0.0)
        except Exception:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.sim_time = 0.0
        self.get_logger().info('Simulation reset.')

    # ── Physics Step ──────────────────────────────────────────────────
    def step(self):
        dt = self.dt

        # ── Dynamic model (second-order with mass/inertia) ────────────
        # F = m*a_v = F_cmd - d_v*v + F_perturb
        # τ = J*a_w = τ_cmd - d_w*w + τ_perturb
        #
        # We model cmd_vel as desired velocity → force = gain*(v_cmd - v)
        k_v = 5.0   # velocity tracking bandwidth
        k_w = 8.0

        F_cmd = k_v * (self.cmd_v - self.v)
        tau_cmd = k_w * (self.cmd_w - self.w)

        a_v = (F_cmd - self.d_v * self.v + self.perturb_v) / self.m
        a_w = (tau_cmd - self.d_w * self.w + self.perturb_w) / self.J

        # Integrate velocities
        self.v += a_v * dt
        self.w += a_w * dt

        # Saturate
        self.v = np.clip(self.v, -self.v_max, self.v_max)
        self.w = np.clip(self.w, -self.w_max, self.w_max)

        # ── Nonholonomic kinematics (midpoint integration) ────────────
        # Pfaffian constraint: ẏ cosθ − ẋ sinθ = 0
        avg_theta = self.theta + self.w * dt * 0.5
        self.x += self.v * np.cos(avg_theta) * dt
        self.y += self.v * np.sin(avg_theta) * dt
        self.theta += self.w * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.sim_time += dt

        # ── Wheel velocities ──────────────────────────────────────────
        wl = (2.0 * self.v - self.w * self.L) / (2.0 * self.r)
        wr = (2.0 * self.v + self.w * self.L) / (2.0 * self.r)

        # ── Publish ───────────────────────────────────────────────────
        self._publish_odom()
        self._publish_pose()
        self.wl_pub.publish(Float32(data=float(wl)))
        self.wr_pub.publish(Float32(data=float(wr)))
        self._publish_state()
        self._publish_tf()

    # ── Publishing helpers ────────────────────────────────────────────
    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.w
        self.odom_pub.publish(msg)

    def _publish_pose(self):
        msg = Pose()
        msg.position.x = self.x
        msg.position.y = self.y
        msg.orientation.z = self.theta
        self.pose_pub.publish(msg)

    def _publish_state(self):
        state = {
            't': round(self.sim_time, 4),
            'x': round(self.x, 6),
            'y': round(self.y, 6),
            'theta': round(self.theta, 6),
            'v': round(self.v, 6),
            'w': round(self.w, 6),
            'pv': round(self.perturb_v, 6),
            'pw': round(self.perturb_w, 6),
        }
        self.state_pub.publish(String(data=json.dumps(state)))

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzleBotSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
