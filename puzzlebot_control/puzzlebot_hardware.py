#!/usr/bin/env python3
"""
PuzzleBot Hardware Bridge
──────────────────────────
Drop-in replacement for puzzlebot_sim.py that drives real hardware via
Micro-ROS.  The controller-facing interface is identical to the simulator:

  SUBSCRIBES
    /cmd_vel              geometry_msgs/Twist   velocity command from any controller
    /sim_reset            std_msgs/String       optional reset (clears odometry)

  PUBLISHES (same as sim)
    /odom                 nav_msgs/Odometry     wheel-odometry
    /VelocityEncL         std_msgs/Float32      left  wheel RPM (signed)
    /VelocityEncR         std_msgs/Float32      right wheel RPM (signed)
    /pose                 geometry_msgs/Pose    2-D pose
  + TF  odom → base_link

  HARDWARE TOPICS (to/from micro-ROS node on ESP32)
    /motor_left/cmd_pwm   std_msgs/Int16   left  wheel PWM  -255…+255
    /motor_right/cmd_pwm  std_msgs/Int16   right wheel PWM  -255…+255
    /motor_left/rpm       std_msgs/Float32 left  wheel RPM  (signed feedback)
    /motor_right/rpm      std_msgs/Float32 right wheel RPM  (signed feedback)

Architecture
────────────
  cmd_vel (v, ω)  →  differential kinematics  →  RPM setpoints (left, right)
                   →  two incremental PID loops  →  PWM commands to ESP32
  RPM feedback    →  odometry integration         →  /odom, TF

Differential-drive kinematics
──────────────────────────────
  ω_L [rad/s] = (v − ω·L/2) / r
  ω_R [rad/s] = (v + ω·L/2) / r
  RPM = ω_wheel · 60 / (2π)
"""

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int16, String
from tf_transformations import quaternion_from_euler
import tf2_ros
import json


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class PIDWheel:
    """Incremental (velocity-form) PID for a single wheel."""

    def __init__(self, kp: float, ki: float, kd: float, rpm_max: float, ts: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.rpm_max = rpm_max
        self.ts = ts
        self._u_prev  = 0.0
        self._e_prev  = 0.0
        self._e_prev2 = 0.0
        self._dir_prev = None

    def reset(self):
        self._u_prev  = 0.0
        self._e_prev  = 0.0
        self._e_prev2 = 0.0
        self._dir_prev = None

    def update(self, ref_rpm: float, meas_rpm: float) -> int:
        """
        Returns Int16 PWM command in range −255…+255.
        ref_rpm  : desired wheel speed (signed, RPM)
        meas_rpm : measured wheel speed (signed, RPM)
        """
        fwd = ref_rpm >= 0.0
        ref_pct  = _clamp(abs(ref_rpm)  * 100.0 / self.rpm_max, 0.0, 105.0)
        meas_pct = _clamp(abs(meas_rpm) * 100.0 / self.rpm_max, 0.0, 130.0)

        stopped = ref_pct < 0.2

        # Direction reversal → reset integrator
        if self._dir_prev is not None and fwd != self._dir_prev:
            self.reset()
        self._dir_prev = fwd

        if stopped:
            self.reset()
            return 0

        e = ref_pct - meas_pct
        delta_u = (
            self.kp * (e - self._e_prev)
            + self.ki * self.ts * e
            + (self.kd / self.ts) * (e - 2.0 * self._e_prev + self._e_prev2)
        )
        u_pct = _clamp(self._u_prev + delta_u, 0.0, 100.0)
        duty  = int(_clamp(u_pct * 255.0 / 100.0 + 0.5, 1.0, 255.0))

        self._e_prev2 = self._e_prev
        self._e_prev  = e
        self._u_prev  = u_pct

        return duty if fwd else -duty


class PuzzleBotHardware(Node):
    """Hardware bridge between puzzlebot_control controllers and the ESP32."""

    def __init__(self):
        super().__init__('puzzlebot_hardware')

        # ── Robot parameters ───────────────────────────────────────
        self.declare_parameter('wheel_base',     0.19)
        self.declare_parameter('wheel_radius',   0.05)
        self.declare_parameter('rpm_max',        110.0)
        self.declare_parameter('sample_time',    0.05)    # seconds  (20 Hz)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 3.0)

        # PID gains (same defaults as pid_velocity_controller)
        self.declare_parameter('kp', 0.3)
        self.declare_parameter('ki', 0.8)
        self.declare_parameter('kd', 0.05)

        self.L    = self.get_parameter('wheel_base').value
        self.r    = self.get_parameter('wheel_radius').value
        self.ts   = self.get_parameter('sample_time').value
        self.v_max = self.get_parameter('max_linear_vel').value
        self.w_max = self.get_parameter('max_angular_vel').value
        rpm_max   = self.get_parameter('rpm_max').value
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value

        # ── PID controllers ────────────────────────────────────────
        self._pid_left  = PIDWheel(kp, ki, kd, rpm_max, self.ts)
        self._pid_right = PIDWheel(kp, ki, kd, rpm_max, self.ts)

        # ── Setpoints (from cmd_vel) ────────────────────────────────
        self._ref_rpm_left  = 0.0
        self._ref_rpm_right = 0.0

        # ── Measured RPM (from motor nodes) ────────────────────────
        self._rpm_left  = 0.0
        self._rpm_right = 0.0

        # ── Odometry state ─────────────────────────────────────────
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0

        # ── TF broadcaster ─────────────────────────────────────────
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Publishers ─────────────────────────────────────────────
        self._pub_odom  = self.create_publisher(Odometry, '/odom',         10)
        self._pub_pose  = self.create_publisher(Pose,     '/pose',         10)
        self._pub_wl    = self.create_publisher(Float32,  '/VelocityEncL', 10)
        self._pub_wr    = self.create_publisher(Float32,  '/VelocityEncR', 10)
        self._pub_pwm_l = self.create_publisher(Int16, 'motor_left/cmd_pwm',  10)
        self._pub_pwm_r = self.create_publisher(Int16, 'motor_right/cmd_pwm', 10)

        # ── Subscribers ────────────────────────────────────────────
        self.create_subscription(Twist,   '/cmd_vel',          self._cb_cmd_vel,    10)
        self.create_subscription(Float32, 'motor_left/rpm',    self._cb_rpm_left,   10)
        self.create_subscription(Float32, 'motor_right/rpm',   self._cb_rpm_right,  10)
        self.create_subscription(String,  '/sim_reset',        self._cb_reset,      10)

        # ── Control loop timer ─────────────────────────────────────
        self._timer = self.create_timer(self.ts, self._control_loop)

        self.get_logger().info(
            f'PuzzleBot Hardware Bridge started  '
            f'L={self.L} m  r={self.r} m  Ts={self.ts*1000:.0f} ms  '
            f'Kp={kp}  Ki={ki}  Kd={kd}')
        self.get_logger().info(
            '  Start micro-ROS agent:  '
            'ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0')

    # ──────────────────────────────────────────────────────────────
    #  Callbacks
    # ──────────────────────────────────────────────────────────────

    def _cb_cmd_vel(self, msg: Twist):
        v = _clamp(msg.linear.x,  -self.v_max, self.v_max)
        w = _clamp(msg.angular.z, -self.w_max, self.w_max)
        # Differential kinematics → wheel angular velocities [rad/s]
        omega_l = (v - w * self.L / 2.0) / self.r
        omega_r = (v + w * self.L / 2.0) / self.r
        # Convert to RPM
        self._ref_rpm_left  = omega_l * 60.0 / (2.0 * np.pi)
        self._ref_rpm_right = omega_r * 60.0 / (2.0 * np.pi)

    def _cb_rpm_left(self, msg: Float32):
        self._rpm_left = float(msg.data)

    def _cb_rpm_right(self, msg: Float32):
        self._rpm_right = float(msg.data)

    def _cb_reset(self, msg: String):
        try:
            d = json.loads(msg.data)
            self._x     = float(d.get('x',     0.0))
            self._y     = float(d.get('y',     0.0))
            self._theta = float(d.get('theta', 0.0))
        except Exception:
            self._x = self._y = self._theta = 0.0
        self._pid_left.reset()
        self._pid_right.reset()
        self._ref_rpm_left = self._ref_rpm_right = 0.0
        self.get_logger().info('Odometry reset.')

    # ──────────────────────────────────────────────────────────────
    #  Control loop (runs at 1/ts Hz)
    # ──────────────────────────────────────────────────────────────

    def _control_loop(self):
        # ── PID → PWM ───────────────────────────────────────────────
        pwm_l = self._pid_left.update(self._ref_rpm_left,  self._rpm_left)
        pwm_r = self._pid_right.update(self._ref_rpm_right, self._rpm_right)

        msg_l = Int16(); msg_l.data = pwm_l
        msg_r = Int16(); msg_r.data = pwm_r
        self._pub_pwm_l.publish(msg_l)
        self._pub_pwm_r.publish(msg_r)

        # ── Odometry from RPM feedback ──────────────────────────────
        # Convert measured RPM → wheel linear velocity [m/s]
        v_l = self._rpm_left  * (2.0 * np.pi / 60.0) * self.r
        v_r = self._rpm_right * (2.0 * np.pi / 60.0) * self.r

        v     = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / self.L

        dt = self.ts
        self._theta += omega * dt
        self._theta  = np.arctan2(np.sin(self._theta), np.cos(self._theta))
        self._x     += v * np.cos(self._theta) * dt
        self._y     += v * np.sin(self._theta) * dt

        # ── Publish /VelocityEncL, /VelocityEncR  (RPM, signed) ────
        wl_msg = Float32(); wl_msg.data = float(self._rpm_left)
        wr_msg = Float32(); wr_msg.data = float(self._rpm_right)
        self._pub_wl.publish(wl_msg)
        self._pub_wr.publish(wr_msg)

        # ── Publish /odom ───────────────────────────────────────────
        now = self.get_clock().now().to_msg()
        q   = quaternion_from_euler(0.0, 0.0, self._theta)

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega
        self._pub_odom.publish(odom)

        # ── Publish /pose ───────────────────────────────────────────
        pose = Pose()
        pose.position.x    = self._x
        pose.position.y    = self._y
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self._pub_pose.publish(pose)

        # ── Broadcast TF  odom → base_link ─────────────────────────
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.x    = q[0]
        tf.transform.rotation.y    = q[1]
        tf.transform.rotation.z    = q[2]
        tf.transform.rotation.w    = q[3]
        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzleBotHardware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PuzzleBot Hardware Bridge.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
