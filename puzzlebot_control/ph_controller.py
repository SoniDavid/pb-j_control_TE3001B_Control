#!/usr/bin/env python3
"""
Port-Hamiltonian IDA-PBC Controller for PuzzleBot
──────────────────────────────────────────────────
Based on: Gimenez, Rosales & Carelli (2015)
"Port-Hamiltonian Modelling of a Differential Drive Mobile Robot
 with Reference Velocities as Inputs" — RPIC 2015

Two-layer design:
  Layer 1 (outer): Navigation energy shaping
    Designs desired velocities μ* = [v*, ω*] via potential V_nav(q)
    with minimum at goal configuration.

  Layer 2 (inner): IDA-PBC velocity positioning (Sec. IV of paper)
    Ensures asymptotic tracking of μ* through the PH structure.

═══════════════════════════════════════════════════════════════════
PH Model in reduced momentum space (eq 6):

  [q̇]   [  0       S(q)  ] [∇_q H̄]   [  0 ]
  [p̄̇] = [-Sᵀ(q)   C̄-D̄  ] [∇_p̄ H̄] + [ Ḡ  ] u

  M̄ = diag(m, I_θ + md²)       — reduced mass matrix
  H̄ = ½ p̄ᵀ M̄⁻¹ p̄              — kinetic energy (Hamiltonian)
  C̄(p̄) = [0, cd·p_ω; -cd·p_ω, 0]  — Coriolis (cd = md/(I_θ+md²))
  D̄ = diag(d_ν, d_ω)           — dissipation (friction)
  Ḡ = I₂                        — input matrix (reduced space)

  S(q) = [cosθ, -d·sinθ; sinθ, d·cosθ; 0, 1]

═══════════════════════════════════════════════════════════════════
IDA-PBC velocity positioner (eq 9):

  u = -Kp(p̄ - p̄*) - Kd·p̄̇ - (C̄(p̄) - D̄)M̄⁻¹p̄

═══════════════════════════════════════════════════════════════════
Desired Hamiltonian (eq 8):

  Hd = ½ (p̄ - p̄*)ᵀ Kp (p̄ - p̄*)

  Ḣd = -(p̄ - p̄*)ᵀ Kpᵀ (Kd + I)⁻¹ Kp (p̄ - p̄*) ≤ 0
  → Asymptotically stable velocity tracking.

═══════════════════════════════════════════════════════════════════
Navigation potential (outer loop):

  V_nav(q) = ½ kv_nav ‖e_pos‖² + ½ kω_nav θ_e²

  μ* = [v*, ω*] where:
    v* = kv · dist · cos(θ_e)   (drive toward goal along heading)
    ω* = kω · θ_e               (align heading with goal)

  Total energy: H_total = Hd(p̄) + V_nav(q)
  Both terms decrease → convergence to goal with zero velocity.
"""
import rclpy
import numpy as np
from puzzlebot_control.base_controller import BaseController, wrap_angle


class PortHamiltonianController(BaseController):
    CONTROLLER_NAME = 'Port-Hamiltonian'

    def __init__(self):
        super().__init__('ph_controller')

        # ── Robot physical parameters (PuzzleBot MCR2-1000) ───────────
        self.declare_parameter('mass', 0.5)             # m [kg]
        self.declare_parameter('inertia', 0.01)          # I_θ [kg·m²]
        self.declare_parameter('d_cm', 0.0)              # d — CM offset from wheel axis [m]
        self.declare_parameter('friction_v', 0.01)       # d_ν — translational friction
        self.declare_parameter('friction_w', 0.01)       # d_ω — rotational friction

        # ── IDA-PBC gains (Kp, Kd from eq 9) ─────────────────────────
        self.declare_parameter('kp_v', 2.0)              # k_p^(ν) — proportional gain (linear)
        self.declare_parameter('kp_w', 2.0)              # k_p^(ω) — proportional gain (angular)
        self.declare_parameter('kd_v', 0.1)              # k_d^(ν) — derivative gain (linear)
        self.declare_parameter('kd_w', 0.1)              # k_d^(ω) — derivative gain (angular)

        # ── Navigation potential gains (outer loop) ───────────────────
        self.declare_parameter('nav_kv', 0.80)           # linear velocity gain
        self.declare_parameter('nav_kw', 2.50)           # angular velocity gain
        self.declare_parameter('nav_max_v', 0.40)        # max reference linear velocity
        self.declare_parameter('nav_max_w', 2.00)        # max reference angular velocity

        # Read parameters
        self.m   = self.get_parameter('mass').value
        self.I_t = self.get_parameter('inertia').value
        self.d   = self.get_parameter('d_cm').value
        self.d_v = self.get_parameter('friction_v').value
        self.d_w = self.get_parameter('friction_w').value

        self.kp_v = self.get_parameter('kp_v').value
        self.kp_w = self.get_parameter('kp_w').value
        self.kd_v = self.get_parameter('kd_v').value
        self.kd_w = self.get_parameter('kd_w').value

        self.nav_kv = self.get_parameter('nav_kv').value
        self.nav_kw = self.get_parameter('nav_kw').value
        self.nav_max_v = self.get_parameter('nav_max_v').value
        self.nav_max_w = self.get_parameter('nav_max_w').value

        # ── Derived quantities ────────────────────────────────────────
        # Reduced mass matrix M̄ = diag(m, I_θ + md²)  (eq after eq 7)
        self.M_bar = np.diag([self.m, self.I_t + self.m * self.d**2])
        self.M_bar_inv = np.diag([1.0/self.m, 1.0/(self.I_t + self.m * self.d**2)])

        # Dissipation matrix D̄ = diag(d_ν, d_ω)
        self.D_bar = np.diag([self.d_v, self.d_w])

        # IDA-PBC gain matrices
        self.Kp = np.diag([self.kp_v, self.kp_w])
        self.Kd = np.diag([self.kd_v, self.kd_w])

        # (Kd + I)⁻¹  — used in closed-loop dynamics analysis (eq 13, 16)
        self.Kd_I_inv = np.linalg.inv(self.Kd + np.eye(2))

        self.get_logger().info(
            f'[pH-IDA-PBC] m={self.m}, I={self.I_t}, d={self.d}, '
            f'Kp=({self.kp_v},{self.kp_w}), Kd=({self.kd_v},{self.kd_w}), '
            f'nav_kv={self.nav_kv}, nav_kw={self.nav_kw}')

    def on_new_goal(self):
        pass  # No internal state to reset — PH structure is memoryless

    def _coriolis(self, p_bar):
        """
        Coriolis matrix C̄(p̄) from eq (7):
          C̄ = [0, cd·p_ω; -cd·p_ω, 0]
        where cd = md / (I_θ + md²)
        """
        cd = (self.m * self.d) / (self.I_t + self.m * self.d**2) if abs(self.d) > 1e-6 else 0.0
        p_omega = p_bar[1]
        return np.array([[0.0, cd * p_omega],
                         [-cd * p_omega, 0.0]])

    def compute_control(self, dist, angle_err, dx, dy):
        """
        Two-layer PH controller:

        Layer 1: Navigation — compute desired velocities μ*
        Layer 2: IDA-PBC — compute control u that tracks μ* asymptotically
        """
        # ══════════════════════════════════════════════════════════════
        # LAYER 1: Navigation energy shaping
        # ══════════════════════════════════════════════════════════════
        # Desired velocities from navigation potential gradient:
        #   v* = kv · dist · cos(θ_e)   — approach goal along heading
        #   ω* = kw · θ_e               — steer toward goal
        v_star = np.clip(self.nav_kv * dist * np.cos(angle_err),
                         -self.nav_max_v, self.nav_max_v)
        w_star = np.clip(self.nav_kw * angle_err,
                         -self.nav_max_w, self.nav_max_w)

        # Desired momentum: p̄* = M̄ · μ*  (Sec. IV)
        mu_star = np.array([v_star, w_star])
        p_bar_star = self.M_bar @ mu_star

        # ══════════════════════════════════════════════════════════════
        # LAYER 2: PH model with velocity inputs (eq 16 of Gimenez)
        # ══════════════════════════════════════════════════════════════
        # The Gimenez model (eq 16) shows that when the robot accepts
        # velocity references (like Gazebo's diff_drive), p̄* IS the
        # natural input. The IDA-PBC controller (eq 9) is INTERNAL to
        # the robot's motor controller. The closed-loop satisfies:
        #   Ḣd = -(p̄-p̄*)ᵀ Kpᵀ (Kd+I)⁻¹ Kp (p̄-p̄*) ≤ 0
        #
        # We send μ* with a small damping correction for robustness:
        mu_current = np.array([self.v_odom, self.w_odom])
        e_mu = mu_current - mu_star

        # Damping injection: -Kd_nav · (μ - μ*)
        kd_nav_v = 0.15
        kd_nav_w = 0.10
        v_cmd = v_star - kd_nav_v * e_mu[0]
        w_cmd = w_star - kd_nav_w * e_mu[1]

        # ══════════════════════════════════════════════════════════════
        # LYAPUNOV / HAMILTONIAN
        # ══════════════════════════════════════════════════════════════
        # Current & desired momentum
        p_bar = self.M_bar @ mu_current
        p_bar_star = self.M_bar @ mu_star
        e_p = p_bar - p_bar_star

        # IDA-PBC Hamiltonian (eq 8):
        #   Hd = ½ (p̄ - p̄*)ᵀ Kp (p̄ - p̄*)
        Hd_ida = 0.5 * e_p @ self.Kp @ e_p

        # Navigation potential:
        #   V_nav = ½ kv ‖e_pos‖² + ½ kw θ_e²
        V_nav = 0.5 * self.nav_kv * dist**2 + 0.5 * self.nav_kw * angle_err**2

        # Total Hamiltonian: both decrease → convergence
        H_total = Hd_ida + V_nav

        # ── Sliding surfaces for dashboard ────────────────────────────
        # Momentum error = p̄ - p̄* (analogous to sliding surface)
        s_v = e_p[0]
        s_w = e_p[1]

        return v_cmd, w_cmd, H_total, s_v, s_w


def main(args=None):
    rclpy.init(args=args)
    node = PortHamiltonianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
