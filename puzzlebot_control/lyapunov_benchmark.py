#!/usr/bin/env python3
"""
Lyapunov Benchmark for PuzzleBot Controllers
──────────────────────────────────────────────
Subscribes to /lyapunov, /controller_state, /sim_state and logs
time-series data for each controller. After all controllers are
benchmarked, generates comparative plots:

  1. Lyapunov V(t) for all controllers
  2. Distance error e(t)
  3. Control effort (v, ω)
  4. Phase portrait (x, y trajectory)
  5. Convergence rate comparison (bar chart)
  6. Perturbation robustness

Usage:
  ros2 run puzzlebot_control lyapunov_benchmark

Or with the benchmark launch file which automatically cycles controllers.
"""
import rclpy
import numpy as np
import os
import json
import time as pytime
from pathlib import Path
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class LyapunovBenchmark(Node):
    def __init__(self):
        super().__init__('lyapunov_benchmark')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('duration', 30.0)
        self.declare_parameter('sample_rate', 50.0)
        self.declare_parameter('output_dir', '~/puzzlebot_benchmarks')
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 1.5)
        self.declare_parameter('auto_cycle', True)

        self.duration = self.get_parameter('duration').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value)
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.auto_cycle = self.get_parameter('auto_cycle').value

        Path(self.output_dir).mkdir(parents=True, exist_ok=True)

        # ── Controller sequence ───────────────────────────────────────
        self.controllers = ['PID', 'SMC', 'ISMC', 'CTC', 'Port-Hamiltonian']
        self.current_idx = 0
        self.running = False

        # ── Data storage ──────────────────────────────────────────────
        self.all_data = {}  # controller_name → dict of arrays
        self.current_data = None
        self.start_time = None

        # ── Subscribers ───────────────────────────────────────────────
        self.lyapunov_sub = self.create_subscription(
            Float64, '/lyapunov', self.lyapunov_cb, 10)
        self.ctrl_state_sub = self.create_subscription(
            String, '/controller_state', self.ctrl_state_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.perturb_sub = self.create_subscription(
            Vector3, '/terrain_perturbation', self.perturb_cb, 10)

        # ── Publishers ────────────────────────────────────────────────
        self.switch_pub = self.create_publisher(String, '/switch_controller', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.reset_pub = self.create_publisher(String, '/sim_reset', 10)
        self.cmd_stop = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── State ─────────────────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.lyapunov_val = 0.0
        self.perturb_v = 0.0
        self.perturb_w = 0.0

        # ── Timer ─────────────────────────────────────────────────────
        dt = 1.0 / self.sample_rate
        self.timer = self.create_timer(dt, self.sample_tick)
        self.init_timer = self.create_timer(2.0, self.start_benchmark)

        self.get_logger().info(
            f'Lyapunov Benchmark: {len(self.controllers)} controllers, '
            f'{self.duration}s each → {self.output_dir}')

    # ── Callbacks ─────────────────────────────────────────────────────
    def lyapunov_cb(self, msg: Float64):
        self.lyapunov_val = msg.data

    def ctrl_state_cb(self, msg: String):
        pass  # We log our own measurements for consistency

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def perturb_cb(self, msg: Vector3):
        self.perturb_v = msg.x
        self.perturb_w = msg.z

    # ── Benchmark orchestration ───────────────────────────────────────
    def start_benchmark(self):
        """Called once after 2s delay to allow nodes to spin up."""
        self.init_timer.cancel()
        if self.auto_cycle:
            self._start_controller(0)

    def _start_controller(self, idx):
        if idx >= len(self.controllers):
            self.get_logger().info('All controllers benchmarked!')
            self._generate_plots()
            return

        name = self.controllers[idx]
        self.current_idx = idx
        self.get_logger().info(f'═══ Starting benchmark: {name} ═══')

        # Reset simulation
        self.reset_pub.publish(String(data='{}'))
        pytime.sleep(0.5)

        # Stop any motion
        self.cmd_stop.publish(Twist())
        pytime.sleep(0.3)

        # Switch controller
        self.switch_pub.publish(String(data=name))
        pytime.sleep(0.3)

        # Publish goal
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        self.goal_pub.publish(goal)

        # Init data
        self.current_data = {
            'name': name,
            't': [], 'V': [], 'dist': [], 'angle_err': [],
            'v': [], 'w': [], 'x': [], 'y': [], 'theta': [],
            'pv': [], 'pw': [],  # perturbations
            'effort_v': [], 'effort_w': [],
        }
        self.start_time = self.get_clock().now()
        self.running = True

    # ── Sampling ──────────────────────────────────────────────────────
    def sample_tick(self):
        if not self.running or self.current_data is None:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        # Compute errors
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = np.sqrt(dx*dx + dy*dy)
        angle_to_goal = np.arctan2(dy, dx)
        angle_err = np.arctan2(
            np.sin(angle_to_goal - self.theta),
            np.cos(angle_to_goal - self.theta))

        d = self.current_data
        d['t'].append(elapsed)
        d['V'].append(self.lyapunov_val)
        d['dist'].append(dist)
        d['angle_err'].append(angle_err)
        d['v'].append(self.v)
        d['w'].append(self.w)
        d['x'].append(self.x)
        d['y'].append(self.y)
        d['theta'].append(self.theta)
        d['pv'].append(self.perturb_v)
        d['pw'].append(self.perturb_w)
        d['effort_v'].append(abs(self.v))
        d['effort_w'].append(abs(self.w))

        # Check if done
        if elapsed >= self.duration:
            self.running = False
            name = self.current_data['name']
            self.all_data[name] = self.current_data
            self.get_logger().info(
                f'═══ Finished: {name} — {len(d["t"])} samples ═══')

            # Stop robot
            self.cmd_stop.publish(Twist())

            # Next controller
            if self.auto_cycle:
                self.create_timer(1.0, lambda: self._start_controller(
                    self.current_idx + 1))

    # ── Plot Generation ───────────────────────────────────────────────
    def _generate_plots(self):
        self.get_logger().info('Generating benchmark plots...')

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            from matplotlib.gridspec import GridSpec
        except ImportError:
            self.get_logger().error(
                'matplotlib not found! pip install matplotlib')
            self._save_raw_data()
            return

        colors = {
            'PID': '#f97316',
            'SMC': '#06b6d4',
            'ISMC': '#8b5cf6',
            'CTC': '#10b981',
            'Port-Hamiltonian': '#ef4444',
        }

        # ══════════════════════════════════════════════════════════════
        # FIGURE 1: Master Comparison (6 subplots)
        # ══════════════════════════════════════════════════════════════
        fig = plt.figure(figsize=(20, 14))
        fig.patch.set_facecolor('#0d1117')
        gs = GridSpec(3, 3, figure=fig, hspace=0.35, wspace=0.3)

        axes_config = {
            'facecolor': '#161b22',
            'grid': True,
            'grid_alpha': 0.15,
        }

        def style_ax(ax, title, xlabel, ylabel):
            ax.set_facecolor('#161b22')
            ax.set_title(title, color='white', fontsize=13, fontweight='bold', pad=10)
            ax.set_xlabel(xlabel, color='#8b949e', fontsize=10)
            ax.set_ylabel(ylabel, color='#8b949e', fontsize=10)
            ax.tick_params(colors='#8b949e', labelsize=9)
            ax.grid(True, alpha=0.15, color='#30363d')
            for spine in ax.spines.values():
                spine.set_color('#30363d')

        # ── 1. Lyapunov V(t) ─────────────────────────────────────────
        ax1 = fig.add_subplot(gs[0, 0])
        for name, d in self.all_data.items():
            ax1.plot(d['t'], d['V'], color=colors.get(name, 'white'),
                     linewidth=1.5, label=name, alpha=0.9)
        style_ax(ax1, 'Lyapunov Function V(t)', 'Time [s]', 'V')
        ax1.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
                   labelcolor='white', loc='upper right')

        # ── 2. Lyapunov V(t) LOG SCALE ───────────────────────────────
        ax2 = fig.add_subplot(gs[0, 1])
        for name, d in self.all_data.items():
            V_arr = np.array(d['V'])
            V_pos = np.maximum(V_arr, 1e-10)
            ax2.semilogy(d['t'], V_pos, color=colors.get(name, 'white'),
                         linewidth=1.5, label=name, alpha=0.9)
        style_ax(ax2, 'Lyapunov V(t) — Log Scale', 'Time [s]', 'log V')
        ax2.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
                   labelcolor='white', loc='upper right')

        # ── 3. Distance Error ─────────────────────────────────────────
        ax3 = fig.add_subplot(gs[0, 2])
        for name, d in self.all_data.items():
            ax3.plot(d['t'], d['dist'], color=colors.get(name, 'white'),
                     linewidth=1.5, label=name, alpha=0.9)
        style_ax(ax3, 'Distance Error ‖e‖(t)', 'Time [s]', 'Distance [m]')
        ax3.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
                   labelcolor='white')

        # ── 4. XY Trajectories ────────────────────────────────────────
        ax4 = fig.add_subplot(gs[1, 0])
        for name, d in self.all_data.items():
            ax4.plot(d['x'], d['y'], color=colors.get(name, 'white'),
                     linewidth=1.5, label=name, alpha=0.8)
        ax4.plot(self.goal_x, self.goal_y, 'r*', markersize=15, zorder=10,
                 label='Goal')
        ax4.plot(0, 0, 'ws', markersize=8, zorder=10, label='Start')
        style_ax(ax4, 'XY Trajectory (Phase Portrait)', 'x [m]', 'y [m]')
        ax4.set_aspect('equal')
        ax4.legend(fontsize=7, facecolor='#161b22', edgecolor='#30363d',
                   labelcolor='white', loc='lower right')

        # ── 5. Control Effort ─────────────────────────────────────────
        ax5 = fig.add_subplot(gs[1, 1])
        for name, d in self.all_data.items():
            total_effort = np.cumsum(np.array(d['effort_v']) +
                                     0.1 * np.array(d['effort_w'])) / self.sample_rate
            ax5.plot(d['t'], total_effort, color=colors.get(name, 'white'),
                     linewidth=1.5, label=name, alpha=0.9)
        style_ax(ax5, 'Cumulative Control Effort', 'Time [s]',
                 '∫(|v| + 0.1|ω|) dt')
        ax5.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
                   labelcolor='white')

        # ── 6. V̇ (Lyapunov derivative) ───────────────────────────────
        ax6 = fig.add_subplot(gs[1, 2])
        for name, d in self.all_data.items():
            V_arr = np.array(d['V'])
            t_arr = np.array(d['t'])
            if len(V_arr) > 2:
                Vdot = np.gradient(V_arr, t_arr)
                # Smooth with moving average
                kernel = 5
                Vdot_smooth = np.convolve(Vdot, np.ones(kernel)/kernel, mode='same')
                ax6.plot(t_arr, Vdot_smooth, color=colors.get(name, 'white'),
                         linewidth=1.2, label=name, alpha=0.8)
        ax6.axhline(y=0, color='white', linewidth=0.5, alpha=0.3, linestyle='--')
        style_ax(ax6, 'V̇(t) — Lyapunov Derivative', 'Time [s]', 'dV/dt')
        ax6.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
                   labelcolor='white')

        # ── 7. Convergence rate bar chart ─────────────────────────────
        ax7 = fig.add_subplot(gs[2, 0])
        conv_times = {}
        for name, d in self.all_data.items():
            dist_arr = np.array(d['dist'])
            t_arr = np.array(d['t'])
            # Time to reach within 0.1m
            idx = np.where(dist_arr < 0.1)[0]
            conv_times[name] = t_arr[idx[0]] if len(idx) > 0 else self.duration

        names = list(conv_times.keys())
        times = [conv_times[n] for n in names]
        bars = ax7.bar(names, times,
                       color=[colors.get(n, 'white') for n in names],
                       alpha=0.85, edgecolor='white', linewidth=0.5)
        style_ax(ax7, 'Convergence Time (to 0.1m)', '', 'Time [s]')
        ax7.tick_params(axis='x', rotation=15)
        for bar, t in zip(bars, times):
            ax7.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                     f'{t:.1f}s', ha='center', va='bottom', color='white',
                     fontsize=9, fontweight='bold')

        # ── 8. Angular error ──────────────────────────────────────────
        ax8 = fig.add_subplot(gs[2, 1])
        for name, d in self.all_data.items():
            ax8.plot(d['t'], np.degrees(d['angle_err']),
                     color=colors.get(name, 'white'),
                     linewidth=1.2, label=name, alpha=0.8)
        style_ax(ax8, 'Heading Error θ_e(t)', 'Time [s]', 'Angle [deg]')
        ax8.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
                   labelcolor='white')

        # ── 9. Perturbation profile ───────────────────────────────────
        ax9 = fig.add_subplot(gs[2, 2])
        # Use data from first controller that has perturbation data
        for name, d in self.all_data.items():
            if len(d['pv']) > 0:
                ax9.plot(d['t'], d['pv'], color='#f97316',
                         linewidth=0.8, alpha=0.7, label='F_perturb (v)')
                ax9.plot(d['t'], d['pw'], color='#06b6d4',
                         linewidth=0.8, alpha=0.7, label='τ_perturb (ω)')
                break
        style_ax(ax9, 'Terrain Perturbations', 'Time [s]', 'Force / Torque')
        ax9.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
                   labelcolor='white')

        # ── Title ─────────────────────────────────────────────────────
        fig.suptitle(
            'PuzzleBot Nonholonomic Controller Benchmark\n'
            'Lyapunov Stability Analysis with Terrain Perturbations',
            color='white', fontsize=16, fontweight='bold', y=0.98)

        # ── Save ──────────────────────────────────────────────────────
        fig_path = os.path.join(self.output_dir, 'lyapunov_benchmark.png')
        fig.savefig(fig_path, dpi=150, bbox_inches='tight',
                    facecolor=fig.get_facecolor())
        plt.close(fig)
        self.get_logger().info(f'Plot saved: {fig_path}')

        # ══════════════════════════════════════════════════════════════
        # FIGURE 2: Per-controller detailed Lyapunov analysis
        # ══════════════════════════════════════════════════════════════
        fig2, axes = plt.subplots(2, len(self.all_data), figsize=(5*len(self.all_data), 8))
        fig2.patch.set_facecolor('#0d1117')

        if len(self.all_data) == 1:
            axes = axes.reshape(-1, 1)

        for i, (name, d) in enumerate(self.all_data.items()):
            c = colors.get(name, 'white')
            t = np.array(d['t'])
            V = np.array(d['V'])

            # Top: V(t) with shaded Ḣd ≤ 0 region
            ax = axes[0, i]
            ax.set_facecolor('#161b22')
            ax.fill_between(t, V, alpha=0.15, color=c)
            ax.plot(t, V, color=c, linewidth=2)
            ax.set_title(f'{name}\nV(t)', color='white', fontsize=11, fontweight='bold')
            ax.set_xlabel('Time [s]', color='#8b949e', fontsize=9)
            ax.set_ylabel('V', color='#8b949e', fontsize=9)
            ax.tick_params(colors='#8b949e', labelsize=8)
            ax.grid(True, alpha=0.1, color='#30363d')
            for spine in ax.spines.values():
                spine.set_color('#30363d')

            # Add annotation: final V value
            if len(V) > 0:
                ax.annotate(f'V_f = {V[-1]:.4f}',
                           xy=(t[-1], V[-1]), fontsize=8, color=c,
                           ha='right', va='bottom')

            # Bottom: dist and angle_err
            ax2 = axes[1, i]
            ax2.set_facecolor('#161b22')
            ax2.plot(t, d['dist'], color=c, linewidth=1.5, label='‖e‖')
            ax2.plot(t, np.abs(d['angle_err']), color=c, linewidth=1,
                     linestyle='--', alpha=0.6, label='|θ_e|')
            ax2.set_title(f'{name}\nErrors', color='white', fontsize=11, fontweight='bold')
            ax2.set_xlabel('Time [s]', color='#8b949e', fontsize=9)
            ax2.set_ylabel('Error', color='#8b949e', fontsize=9)
            ax2.tick_params(colors='#8b949e', labelsize=8)
            ax2.grid(True, alpha=0.1, color='#30363d')
            ax2.legend(fontsize=7, facecolor='#161b22', edgecolor='#30363d',
                       labelcolor='white')
            for spine in ax2.spines.values():
                spine.set_color('#30363d')

        fig2.suptitle('Per-Controller Lyapunov Analysis: Ḣd ≤ 0',
                       color='white', fontsize=14, fontweight='bold', y=1.02)
        fig2_path = os.path.join(self.output_dir, 'lyapunov_per_controller.png')
        fig2.savefig(fig2_path, dpi=150, bbox_inches='tight',
                     facecolor=fig2.get_facecolor())
        plt.close(fig2)
        self.get_logger().info(f'Plot saved: {fig2_path}')

        # ── Save raw data as JSON ─────────────────────────────────────
        self._save_raw_data()

        self.get_logger().info('══════════════════════════════════════')
        self.get_logger().info('  BENCHMARK COMPLETE')
        self.get_logger().info(f'  Results: {self.output_dir}')
        self.get_logger().info('══════════════════════════════════════')

    def _save_raw_data(self):
        for name, d in self.all_data.items():
            fname = name.lower().replace('-', '_').replace(' ', '_')
            path = os.path.join(self.output_dir, f'data_{fname}.json')
            # Convert numpy arrays to lists
            save_d = {}
            for k, v in d.items():
                if isinstance(v, list):
                    save_d[k] = [float(x) if isinstance(x, (float, np.floating))
                                 else x for x in v]
                else:
                    save_d[k] = v
            with open(path, 'w') as f:
                json.dump(save_d, f, indent=2)
            self.get_logger().info(f'Data saved: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = LyapunovBenchmark()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
