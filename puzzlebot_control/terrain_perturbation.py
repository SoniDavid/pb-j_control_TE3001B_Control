#!/usr/bin/env python3
"""
Terrain Perturbation Generator
───────────────────────────────
Publishes disturbance forces on /terrain_perturbation (Vector3):
  x → force perturbation on linear velocity
  z → torque perturbation on angular velocity

Supports: sinusoidal, step, Gaussian noise, mixed profiles.
Configurable via ROS2 parameters.
"""
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


class TerrainPerturbation(Node):
    def __init__(self):
        super().__init__('terrain_perturbation')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('enabled', True)
        self.declare_parameter('type', 'mixed')
        self.declare_parameter('amplitude_v', 0.05)
        self.declare_parameter('amplitude_w', 0.1)
        self.declare_parameter('frequency', 0.5)
        self.declare_parameter('step_interval', 5.0)
        self.declare_parameter('noise_sigma_v', 0.02)
        self.declare_parameter('noise_sigma_w', 0.04)
        self.declare_parameter('publish_rate', 100.0)

        self.enabled = self.get_parameter('enabled').value
        self.ptype = self.get_parameter('type').value
        self.amp_v = self.get_parameter('amplitude_v').value
        self.amp_w = self.get_parameter('amplitude_w').value
        self.freq = self.get_parameter('frequency').value
        self.step_int = self.get_parameter('step_interval').value
        self.sigma_v = self.get_parameter('noise_sigma_v').value
        self.sigma_w = self.get_parameter('noise_sigma_w').value
        rate = self.get_parameter('publish_rate').value

        # ── State ─────────────────────────────────────────────────────
        self.t = 0.0
        self.dt = 1.0 / rate
        self.last_step_time = 0.0
        self.step_val_v = 0.0
        self.step_val_w = 0.0

        # ── Publisher ─────────────────────────────────────────────────
        self.pub = self.create_publisher(Vector3, '/terrain_perturbation', 10)
        self.info_pub = self.create_publisher(String, '/terrain_info', 10)

        # ── Subscriber for dynamic control ────────────────────────────
        self.config_sub = self.create_subscription(
            String, '/terrain_config', self.config_cb, 10)

        # ── Timer ─────────────────────────────────────────────────────
        self.timer = self.create_timer(self.dt, self.publish_perturbation)

        self.get_logger().info(
            f'Terrain perturbation: type={self.ptype}, '
            f'amp_v={self.amp_v}, amp_w={self.amp_w}')

    def config_cb(self, msg: String):
        """Dynamically change perturbation type at runtime."""
        import json
        try:
            cfg = json.loads(msg.data)
            if 'type' in cfg:
                self.ptype = cfg['type']
            if 'enabled' in cfg:
                self.enabled = cfg['enabled']
            if 'amplitude_v' in cfg:
                self.amp_v = cfg['amplitude_v']
            if 'amplitude_w' in cfg:
                self.amp_w = cfg['amplitude_w']
            self.get_logger().info(f'Terrain config updated: {cfg}')
        except Exception as e:
            self.get_logger().warn(f'Bad terrain config: {e}')

    def publish_perturbation(self):
        self.t += self.dt
        msg = Vector3()

        if not self.enabled or self.ptype == 'none':
            msg.x = 0.0
            msg.z = 0.0
        else:
            fv, fw = 0.0, 0.0

            if self.ptype in ('sinusoidal', 'mixed'):
                fv += self.amp_v * np.sin(2.0 * np.pi * self.freq * self.t)
                fw += self.amp_w * np.sin(2.0 * np.pi * self.freq * 0.7 * self.t + 1.0)

            if self.ptype in ('step', 'mixed'):
                if self.t - self.last_step_time >= self.step_int:
                    self.step_val_v = np.random.uniform(-self.amp_v, self.amp_v)
                    self.step_val_w = np.random.uniform(-self.amp_w, self.amp_w)
                    self.last_step_time = self.t
                fv += self.step_val_v
                fw += self.step_val_w

            if self.ptype in ('noise', 'mixed'):
                fv += np.random.normal(0, self.sigma_v)
                fw += np.random.normal(0, self.sigma_w)

            msg.x = float(fv)
            msg.z = float(fw)

        self.pub.publish(msg)

        # Publish info at lower rate
        if int(self.t / self.dt) % 50 == 0:
            info = String()
            info.data = (f'{{"type":"{self.ptype}","enabled":{str(self.enabled).lower()},'
                         f'"fv":{msg.x:.4f},"fw":{msg.z:.4f}}}')
            self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainPerturbation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
