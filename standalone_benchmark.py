#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════
  PuzzleBot Controller Benchmark — Standalone (no ROS2 required)
═══════════════════════════════════════════════════════════════════════

Simulates the PuzzleBot differential drive with all 5 controllers,
terrain perturbations, and generates Lyapunov comparison plots.

Run:  python3 standalone_benchmark.py

Output: lyapunov_benchmark.png, lyapunov_per_controller.png
"""
import numpy as np
import os
import json

# ═══════════════════════════════════════════════════════════════════════
#  PuzzleBot Physical Parameters (MCR2-1000)
# ═══════════════════════════════════════════════════════════════════════
L_WHEEL = 0.19        # wheelbase [m]
R_WHEEL = 0.05        # wheel radius [m]
MASS    = 0.5         # effective mass [kg]
INERTIA = 0.01        # rotational inertia [kg·m²]
D_V     = 0.3         # linear damping
D_W     = 0.1         # angular damping
V_MAX   = 0.5
W_MAX   = 3.0
DT      = 0.01        # integration timestep [s]
T_FINAL = 30.0        # simulation duration per controller [s]
GOAL    = (2.0, 1.5)  # target waypoint


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))

def sat(s, phi):
    if phi <= 0:
        return np.sign(s)
    return np.clip(s / phi, -1.0, 1.0)


# ═══════════════════════════════════════════════════════════════════════
#  Terrain Perturbation
# ═══════════════════════════════════════════════════════════════════════
class TerrainPerturbation:
    """Mixed perturbation: sinusoidal + step + noise."""
    def __init__(self, amp_v=0.05, amp_w=0.1, freq=0.5,
                 step_int=5.0, sigma_v=0.02, sigma_w=0.04):
        self.amp_v = amp_v
        self.amp_w = amp_w
        self.freq = freq
        self.step_int = step_int
        self.sigma_v = sigma_v
        self.sigma_w = sigma_w
        self.last_step = 0.0
        self.step_v = 0.0
        self.step_w = 0.0

    def __call__(self, t):
        fv = self.amp_v * np.sin(2*np.pi*self.freq*t)
        fw = self.amp_w * np.sin(2*np.pi*self.freq*0.7*t + 1.0)
        if t - self.last_step >= self.step_int:
            self.step_v = np.random.uniform(-self.amp_v, self.amp_v)
            self.step_w = np.random.uniform(-self.amp_w, self.amp_w)
            self.last_step = t
        fv += self.step_v + np.random.normal(0, self.sigma_v)
        fw += self.step_w + np.random.normal(0, self.sigma_w)
        return fv, fw


# ═══════════════════════════════════════════════════════════════════════
#  Physics Simulator
# ═══════════════════════════════════════════════════════════════════════
class PuzzleBotSim:
    def __init__(self):
        self.reset()
        self.terrain = TerrainPerturbation()

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.terrain = TerrainPerturbation()

    def step(self, v_cmd, w_cmd, t):
        pv, pw = self.terrain(t)
        k_v, k_w = 5.0, 8.0
        F_cmd = k_v * (np.clip(v_cmd, -V_MAX, V_MAX) - self.v)
        tau_cmd = k_w * (np.clip(w_cmd, -W_MAX, W_MAX) - self.w)
        a_v = (F_cmd - D_V*self.v + pv) / MASS
        a_w = (tau_cmd - D_W*self.w + pw) / INERTIA
        self.v = np.clip(self.v + a_v*DT, -V_MAX, V_MAX)
        self.w = np.clip(self.w + a_w*DT, -W_MAX, W_MAX)
        avg_theta = self.theta + self.w*DT*0.5
        self.x += self.v * np.cos(avg_theta) * DT
        self.y += self.v * np.sin(avg_theta) * DT
        self.theta = wrap(self.theta + self.w*DT)
        return pv, pw


# ═══════════════════════════════════════════════════════════════════════
#  Controllers
# ═══════════════════════════════════════════════════════════════════════

def get_errors(sim, goal):
    dx = goal[0] - sim.x
    dy = goal[1] - sim.y
    dist = np.sqrt(dx*dx + dy*dy)
    ang = wrap(np.arctan2(dy, dx) - sim.theta)
    return dist, ang, dx, dy


class PIDCtrl:
    name = 'PID'
    def __init__(self):
        self.reset()
    def reset(self):
        self.i_d = 0; self.i_a = 0; self.p_d = 0; self.p_a = 0
    def __call__(self, sim, goal):
        d, a, _, _ = get_errors(sim, goal)
        self.i_d += d*DT; self.i_a += a*DT
        self.i_d = np.clip(self.i_d, -5, 5)
        self.i_a = np.clip(self.i_a, -5, 5)
        dd = (d - self.p_d)/DT; da = (a - self.p_a)/DT
        self.p_d = d; self.p_a = a
        v = 1.2*d + 0.05*self.i_d + 0.2*dd
        w = 3.0*a + 0.05*self.i_a + 0.35*da
        if abs(a) > 0.8: v *= 0.15
        V = 0.5*d**2 + 0.5*a**2
        return np.clip(v,-V_MAX,V_MAX), np.clip(w,-W_MAX,W_MAX), V


class SMCCtrl:
    name = 'SMC'
    def reset(self): pass
    def __call__(self, sim, goal):
        d, a, _, _ = get_errors(sim, goal)
        v = 1.5*d + 0.15*sat(d, 0.05)
        w = 3.5*a + 0.80*sat(a, 0.10)
        if abs(a) > 0.8: v *= 0.15
        V = 0.5*d**2 + 0.5*a**2
        return np.clip(v,-V_MAX,V_MAX), np.clip(w,-W_MAX,W_MAX), V


class ISMCCtrl:
    name = 'ISMC'
    def __init__(self):
        self.reset()
    def reset(self):
        self.i_d = 0; self.i_a = 0
    def __call__(self, sim, goal):
        d, a, _, _ = get_errors(sim, goal)
        self.i_d += d*DT; self.i_a += a*DT
        self.i_d = np.clip(self.i_d, -2, 2)
        self.i_a = np.clip(self.i_a, -2, 2)
        s_v = d + 0.3*self.i_d
        s_w = a + 0.8*self.i_a
        v = 1.5*s_v + 0.12*sat(s_v, 0.04)
        w = 3.0*s_w + 0.7*sat(s_w, 0.08)
        if abs(a) > 0.8: v *= 0.15
        # Lyapunov: use raw errors for fair comparison
        V = 0.5*d**2 + 0.5*a**2
        return np.clip(v,-V_MAX,V_MAX), np.clip(w,-W_MAX,W_MAX), V


class CTCCtrl:
    name = 'CTC'
    def __init__(self):
        self.reset()
    def reset(self):
        self.v_cmd = 0; self.w_cmd = 0
    def __call__(self, sim, goal):
        d, a, _, _ = get_errors(sim, goal)
        # Desired velocities (outer loop)
        v_des = 1.8*d*np.cos(a)
        w_des = 3.5*a
        v_err = v_des - sim.v
        w_err = w_des - sim.w
        # Computed torque: cancel dynamics + impose desired acceleration
        a_v = 3.0*v_err   # desired acceleration
        a_w = 5.0*w_err
        # Direct velocity output (bypass integration to avoid drift)
        v = v_des + 0.8*v_err
        w = w_des + 0.6*w_err
        if abs(a) > 0.8: v *= 0.15
        V = 0.5*d**2 + 0.5*a**2
        return np.clip(v,-V_MAX,V_MAX), np.clip(w,-W_MAX,W_MAX), V


class PHCtrl:
    """
    Port-Hamiltonian IDA-PBC controller (Gimenez, Rosales & Carelli 2015).

    PH model (eq 6): reduced momentum p̄ = [p_ν, p_ω]ᵀ
      M̄ = diag(m, I_θ + md²)
      C̄ = [0, cd·p_ω; -cd·p_ω, 0]  with cd = md/(I_θ+md²)
      D̄ = diag(d_ν, d_ω)

    IDA-PBC control (eq 9):
      u = -Kp(p̄ - p̄*) - Kd·p̄̇ - (C̄ - D̄)M̄⁻¹p̄

    Desired Hamiltonian (eq 8):
      Hd = ½(p̄ - p̄*)ᵀ Kp (p̄ - p̄*)
      Ḣd = -(p̄-p̄*)ᵀ Kpᵀ (Kd+I)⁻¹ Kp (p̄-p̄*) ≤ 0
    """
    name = 'Port-Hamiltonian'
    # Robot params
    m = MASS; I_t = INERTIA; d_cm = 0.0
    d_v = 0.01; d_w = 0.01
    # IDA-PBC gains (Kp, Kd from eq 9)
    kp_v = 2.0; kp_w = 2.0; kd_v = 0.1; kd_w = 0.1
    # Navigation gains (outer loop)
    nav_kv = 0.80; nav_kw = 2.50

    def __init__(self):
        self.reset()
        # Reduced mass M̄ = diag(m, I_θ + md²)
        self.M = np.diag([self.m, self.I_t + self.m*self.d_cm**2])
        self.M_inv = np.linalg.inv(self.M)
        self.Kp = np.diag([self.kp_v, self.kp_w])
        self.Kd = np.diag([self.kd_v, self.kd_w])
        self.D_bar = np.diag([self.d_v, self.d_w])
        self.Kd_I_inv = np.linalg.inv(self.Kd + np.eye(2))

    def reset(self):
        self.p_prev = np.zeros(2)
        self.p_dot = np.zeros(2)

    def _coriolis(self, p_bar):
        cd = (self.m*self.d_cm)/(self.I_t + self.m*self.d_cm**2) if abs(self.d_cm)>1e-6 else 0.0
        return np.array([[0, cd*p_bar[1]], [-cd*p_bar[1], 0]])

    def __call__(self, sim, goal):
        d, a_err, dx, dy = get_errors(sim, goal)

        # ── Layer 1: Navigation potential → desired velocities μ* ─────
        # V_nav(q) = ½ kv ‖e‖² + ½ kw θ_e²
        # μ* = -∇_μ V_nav along feasible directions
        v_star = np.clip(self.nav_kv * d * np.cos(a_err), -V_MAX, V_MAX)
        w_star = np.clip(self.nav_kw * a_err, -W_MAX, W_MAX)

        # ── Layer 2: Velocity reference = input to PH model (eq 16) ──
        # The Gimenez model (eq 16) shows that when the plant accepts
        # velocity references, p̄* IS the input. The IDA-PBC (eq 9)
        # is INTERNAL to the robot's velocity tracking loop.
        # The closed-loop gives: Ḣd ≤ 0 (asymptotic stability).
        #
        # We add a small damping correction for robustness:
        mu_star = np.array([v_star, w_star])
        mu = np.array([sim.v, sim.w])
        e_mu = mu - mu_star

        # Damping injection (energy extraction): -Kd_nav · e_μ
        kd_nav_v = 0.15; kd_nav_w = 0.10
        v = v_star - kd_nav_v * e_mu[0]
        w = w_star - kd_nav_w * e_mu[1]

        if abs(a_err) > 1.2: v *= 0.15

        # ── Hamiltonians ──────────────────────────────────────────────
        # IDA-PBC Hd (eq 8): ½(p̄-p̄*)ᵀ Kp (p̄-p̄*)
        p_bar = self.M @ mu
        p_star = self.M @ mu_star
        e_p = p_bar - p_star
        Hd = 0.5 * e_p @ self.Kp @ e_p

        # Navigation potential
        V_nav = 0.5 * self.nav_kv * d**2 + 0.5 * self.nav_kw * a_err**2

        V = Hd + V_nav

        return np.clip(v,-V_MAX,V_MAX), np.clip(w,-W_MAX,W_MAX), V


# ═══════════════════════════════════════════════════════════════════════
#  Run Benchmark
# ═══════════════════════════════════════════════════════════════════════
def run_benchmark():
    controllers = [PIDCtrl(), SMCCtrl(), ISMCCtrl(), CTCCtrl(), PHCtrl()]
    sim = PuzzleBotSim()
    N = int(T_FINAL / DT)

    all_data = {}

    for ctrl in controllers:
        print(f'Running {ctrl.name}...')
        sim.reset()
        ctrl.reset()
        np.random.seed(42)  # reproducible perturbations

        data = {k: np.zeros(N) for k in
                ['t','V','dist','angle_err','v','w','x','y','theta','pv','pw']}

        for i in range(N):
            t = i * DT
            v_cmd, w_cmd, V = ctrl(sim, GOAL)
            pv, pw = sim.step(v_cmd, w_cmd, t)
            d, a, _, _ = get_errors(sim, GOAL)

            data['t'][i] = t
            data['V'][i] = V
            data['dist'][i] = d
            data['angle_err'][i] = a
            data['v'][i] = sim.v
            data['w'][i] = sim.w
            data['x'][i] = sim.x
            data['y'][i] = sim.y
            data['theta'][i] = sim.theta
            data['pv'][i] = pv
            data['pw'][i] = pw

        all_data[ctrl.name] = data
        print(f'  Final dist={data["dist"][-1]:.4f}m, V={data["V"][-1]:.4f}')

    return all_data


# ═══════════════════════════════════════════════════════════════════════
#  Plot Generation
# ═══════════════════════════════════════════════════════════════════════
def generate_plots(all_data, output_dir='.'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    colors = {
        'PID': '#f97316', 'SMC': '#06b6d4', 'ISMC': '#8b5cf6',
        'CTC': '#10b981', 'Port-Hamiltonian': '#ef4444',
    }

    def style_ax(ax, title, xlabel, ylabel):
        ax.set_facecolor('#161b22')
        ax.set_title(title, color='white', fontsize=12, fontweight='bold', pad=10)
        ax.set_xlabel(xlabel, color='#8b949e', fontsize=10)
        ax.set_ylabel(ylabel, color='#8b949e', fontsize=10)
        ax.tick_params(colors='#8b949e', labelsize=9)
        ax.grid(True, alpha=0.15, color='#30363d')
        for spine in ax.spines.values():
            spine.set_color('#30363d')

    # ══════════════════════════════════════════════════════════════════
    # FIGURE 1: Master 3×3 Grid
    # ══════════════════════════════════════════════════════════════════
    fig = plt.figure(figsize=(22, 15))
    fig.patch.set_facecolor('#0d1117')
    gs = GridSpec(3, 3, figure=fig, hspace=0.38, wspace=0.32)

    # 1. Lyapunov V(t)
    ax = fig.add_subplot(gs[0, 0])
    for name, d in all_data.items():
        ax.plot(d['t'], d['V'], color=colors[name], linewidth=1.8,
                label=name, alpha=0.9)
    style_ax(ax, 'Lyapunov Function V(t)', 'Time [s]', 'V')
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white', loc='upper right')

    # 2. Lyapunov log scale
    ax = fig.add_subplot(gs[0, 1])
    for name, d in all_data.items():
        V_pos = np.maximum(d['V'], 1e-10)
        ax.semilogy(d['t'], V_pos, color=colors[name], linewidth=1.8,
                    label=name, alpha=0.9)
    style_ax(ax, 'V(t) — Log Scale', 'Time [s]', 'log₁₀ V')
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white', loc='upper right')

    # 3. Distance error
    ax = fig.add_subplot(gs[0, 2])
    for name, d in all_data.items():
        ax.plot(d['t'], d['dist'], color=colors[name], linewidth=1.8,
                label=name, alpha=0.9)
    style_ax(ax, 'Distance Error ‖e‖(t)', 'Time [s]', 'Distance [m]')
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    # 4. XY Trajectories
    ax = fig.add_subplot(gs[1, 0])
    for name, d in all_data.items():
        ax.plot(d['x'], d['y'], color=colors[name], linewidth=1.5,
                label=name, alpha=0.8)
    ax.plot(*GOAL, 'r*', markersize=18, zorder=10, label='Goal')
    ax.plot(0, 0, 'ws', markersize=10, zorder=10, label='Start')
    style_ax(ax, 'XY Trajectory (Phase Portrait)', 'x [m]', 'y [m]')
    ax.set_aspect('equal')
    ax.legend(fontsize=7, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white', loc='lower right')

    # 5. Cumulative control effort
    ax = fig.add_subplot(gs[1, 1])
    for name, d in all_data.items():
        effort = np.cumsum(np.abs(d['v']) + 0.1*np.abs(d['w'])) * DT
        ax.plot(d['t'], effort, color=colors[name], linewidth=1.8,
                label=name, alpha=0.9)
    style_ax(ax, 'Cumulative Control Effort', 'Time [s]', '∫(|v|+0.1|ω|) dt')
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    # 6. V̇ (Lyapunov derivative)
    ax = fig.add_subplot(gs[1, 2])
    for name, d in all_data.items():
        Vdot = np.gradient(d['V'], d['t'])
        k = 15
        Vdot_s = np.convolve(Vdot, np.ones(k)/k, mode='same')
        ax.plot(d['t'], Vdot_s, color=colors[name], linewidth=1.2,
                label=name, alpha=0.8)
    ax.axhline(0, color='white', linewidth=0.5, alpha=0.3, linestyle='--')
    style_ax(ax, 'V̇(t) — Lyapunov Derivative', 'Time [s]', 'dV/dt')
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    # 7. Convergence time bar chart
    ax = fig.add_subplot(gs[2, 0])
    conv = {}
    for name, d in all_data.items():
        idx = np.where(d['dist'] < 0.1)[0]
        conv[name] = d['t'][idx[0]] if len(idx) > 0 else T_FINAL
    names = list(conv.keys())
    times = [conv[n] for n in names]
    bars = ax.bar(range(len(names)), times,
                  color=[colors[n] for n in names], alpha=0.85,
                  edgecolor='white', linewidth=0.5)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=20, ha='right')
    style_ax(ax, 'Convergence Time (to 0.1 m)', '', 'Time [s]')
    for bar, t in zip(bars, times):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                f'{t:.1f}s', ha='center', va='bottom', color='white',
                fontsize=9, fontweight='bold')

    # 8. Heading error
    ax = fig.add_subplot(gs[2, 1])
    for name, d in all_data.items():
        ax.plot(d['t'], np.degrees(d['angle_err']), color=colors[name],
                linewidth=1.2, label=name, alpha=0.8)
    style_ax(ax, 'Heading Error θ_e(t)', 'Time [s]', 'Angle [°]')
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    # 9. Perturbation profile
    ax = fig.add_subplot(gs[2, 2])
    d0 = list(all_data.values())[0]
    ax.plot(d0['t'], d0['pv'], color='#f97316', linewidth=0.8, alpha=0.7,
            label='F_perturb (linear)')
    ax.plot(d0['t'], d0['pw'], color='#06b6d4', linewidth=0.8, alpha=0.7,
            label='τ_perturb (angular)')
    style_ax(ax, 'Terrain Perturbations', 'Time [s]', 'Force / Torque')
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    fig.suptitle(
        'PuzzleBot Nonholonomic Controller Benchmark\n'
        'Lyapunov Stability Analysis with Terrain Perturbations\n'
        f'Robot: L={L_WHEEL}m, r={R_WHEEL}m, m={MASS}kg | '
        f'Goal=({GOAL[0]}, {GOAL[1]}) | T={T_FINAL}s | '
        'Pfaffian: ẏcosθ − ẋsinθ = 0',
        color='white', fontsize=14, fontweight='bold', y=1.01)

    path1 = os.path.join(output_dir, 'lyapunov_benchmark.png')
    fig.savefig(path1, dpi=170, bbox_inches='tight',
                facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f'Saved: {path1}')

    # ══════════════════════════════════════════════════════════════════
    # FIGURE 2: Per-controller Lyapunov detail
    # ══════════════════════════════════════════════════════════════════
    nc = len(all_data)
    fig2, axes = plt.subplots(3, nc, figsize=(5*nc, 12))
    fig2.patch.set_facecolor('#0d1117')
    if nc == 1:
        axes = axes.reshape(-1, 1)

    for i, (name, d) in enumerate(all_data.items()):
        c = colors[name]
        t = d['t']
        V = d['V']

        # Row 0: V(t) with fill
        ax = axes[0, i]
        ax.set_facecolor('#161b22')
        ax.fill_between(t, V, alpha=0.12, color=c)
        ax.plot(t, V, color=c, linewidth=2)
        ax.set_title(f'{name}\nV(t) — Lyapunov', color='white', fontsize=11,
                     fontweight='bold')
        ax.set_xlabel('t [s]', color='#8b949e', fontsize=9)
        ax.tick_params(colors='#8b949e', labelsize=8)
        ax.grid(True, alpha=0.1, color='#30363d')
        for s in ax.spines.values(): s.set_color('#30363d')
        ax.annotate(f'V_f={V[-1]:.4f}', xy=(t[-1], V[-1]),
                    fontsize=8, color=c, ha='right')

        # Row 1: dist + |angle_err|
        ax = axes[1, i]
        ax.set_facecolor('#161b22')
        ax.plot(t, d['dist'], color=c, linewidth=1.5, label='‖e‖')
        ax.plot(t, np.abs(d['angle_err']), color=c, linewidth=1,
                linestyle='--', alpha=0.6, label='|θ_e|')
        ax.set_title(f'{name}\nErrors', color='white', fontsize=11,
                     fontweight='bold')
        ax.tick_params(colors='#8b949e', labelsize=8)
        ax.grid(True, alpha=0.1, color='#30363d')
        ax.legend(fontsize=7, facecolor='#161b22', edgecolor='#30363d',
                  labelcolor='white')
        for s in ax.spines.values(): s.set_color('#30363d')

        # Row 2: v(t) and w(t) control signals
        ax = axes[2, i]
        ax.set_facecolor('#161b22')
        ax.plot(t, d['v'], color=c, linewidth=1.2, label='v(t)')
        ax.plot(t, d['w'], color=c, linewidth=1, linestyle=':',
                alpha=0.7, label='ω(t)')
        ax.set_title(f'{name}\nControl Signals', color='white',
                     fontsize=11, fontweight='bold')
        ax.set_xlabel('t [s]', color='#8b949e', fontsize=9)
        ax.tick_params(colors='#8b949e', labelsize=8)
        ax.grid(True, alpha=0.1, color='#30363d')
        ax.legend(fontsize=7, facecolor='#161b22', edgecolor='#30363d',
                  labelcolor='white')
        for s in ax.spines.values(): s.set_color('#30363d')

    fig2.suptitle('Per-Controller Lyapunov Analysis: Ḣ_d ≤ 0\n'
                   'Port-Hamiltonian: Hd = ½pᵀM⁻¹p + ½wᵀLw (Ferguson et al.)',
                   color='white', fontsize=13, fontweight='bold', y=1.02)
    path2 = os.path.join(output_dir, 'lyapunov_per_controller.png')
    fig2.savefig(path2, dpi=170, bbox_inches='tight',
                 facecolor=fig2.get_facecolor())
    plt.close(fig2)
    print(f'Saved: {path2}')

    # ── Save raw JSON ─────────────────────────────────────────────────
    for name, d in all_data.items():
        fname = name.lower().replace('-', '_').replace(' ', '_')
        path = os.path.join(output_dir, f'data_{fname}.json')
        save_d = {k: v.tolist() if isinstance(v, np.ndarray) else v
                  for k, v in d.items()}
        with open(path, 'w') as f:
            json.dump(save_d, f)
    print(f'Raw data saved to {output_dir}/')

    # ── Print summary table ───────────────────────────────────────────
    print('\n' + '='*72)
    print(f'  {"Controller":<20} {"Conv. Time":>12} {"Final Dist":>12} {"Final V":>12}')
    print('-'*72)
    for name, d in all_data.items():
        idx = np.where(d['dist'] < 0.1)[0]
        ct = f'{d["t"][idx[0]]:.2f}s' if len(idx) > 0 else '>30s'
        print(f'  {name:<20} {ct:>12} {d["dist"][-1]:>12.4f} {d["V"][-1]:>12.4f}')
    print('='*72)


# ═══════════════════════════════════════════════════════════════════════
#  Robustness Sweep — Multiple perturbation amplitudes
# ═══════════════════════════════════════════════════════════════════════
PERTURB_SCALES = [0.0, 0.5, 1.0, 2.0, 4.0, 8.0]
PERTURB_LABELS = ['0×', '0.5×', '1×', '2×', '4×', '8×']

def make_controllers():
    return [PIDCtrl(), SMCCtrl(), ISMCCtrl(), CTCCtrl(), PHCtrl()]

def run_robustness_sweep():
    """Run every controller at every perturbation amplitude."""
    sim = PuzzleBotSim()
    N = int(T_FINAL / DT)
    sweep = {}   # ctrl_name → { scale → data_dict }

    for ctrl in make_controllers():
        sweep[ctrl.name] = {}
        for scale in PERTURB_SCALES:
            sim.reset()
            ctrl.reset()
            np.random.seed(42)

            # Scale both amplitude and noise
            sim.terrain = TerrainPerturbation(
                amp_v=0.05*scale, amp_w=0.1*scale,
                freq=0.5, step_int=5.0,
                sigma_v=0.02*scale, sigma_w=0.04*scale,
            )

            data = {k: np.zeros(N) for k in
                    ['t','V','dist','angle_err','v','w','x','y','theta',
                     'v_cmd','w_cmd','pv','pw']}

            prev_dist = 0.0
            prev_angle = 0.0

            for i in range(N):
                t = i * DT
                d_before, a_before, _, _ = get_errors(sim, GOAL)
                v_cmd, w_cmd, V = ctrl(sim, GOAL)
                pv, pw = sim.step(v_cmd, w_cmd, t)
                d, a, _, _ = get_errors(sim, GOAL)

                data['t'][i] = t
                data['V'][i] = V
                data['dist'][i] = d
                data['angle_err'][i] = a
                data['v'][i] = sim.v
                data['w'][i] = sim.w
                data['x'][i] = sim.x
                data['y'][i] = sim.y
                data['theta'][i] = sim.theta
                data['v_cmd'][i] = v_cmd
                data['w_cmd'][i] = w_cmd
                data['pv'][i] = pv
                data['pw'][i] = pw

            sweep[ctrl.name][scale] = data
            tag = f'{scale:.1f}×'
            print(f'  {ctrl.name:20s} perturb={tag:>5s}  '
                  f'dist={data["dist"][-1]:.4f}m  V={data["V"][-1]:.4f}')
    return sweep


# ═══════════════════════════════════════════════════════════════════════
#  Phase Diagrams + Robustness Plots
# ═══════════════════════════════════════════════════════════════════════
def generate_phase_robustness_plots(sweep, all_data, output_dir='.'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
    from matplotlib.collections import LineCollection
    import matplotlib.colors as mcolors

    colors = {
        'PID': '#f97316', 'SMC': '#06b6d4', 'ISMC': '#8b5cf6',
        'CTC': '#10b981', 'Port-Hamiltonian': '#ef4444',
    }
    ctrl_names = list(colors.keys())
    nc = len(ctrl_names)

    def style_ax(ax, title, xlabel, ylabel, fs_title=11):
        ax.set_facecolor('#161b22')
        ax.set_title(title, color='white', fontsize=fs_title,
                     fontweight='bold', pad=8)
        ax.set_xlabel(xlabel, color='#8b949e', fontsize=9)
        ax.set_ylabel(ylabel, color='#8b949e', fontsize=9)
        ax.tick_params(colors='#8b949e', labelsize=8)
        ax.grid(True, alpha=0.12, color='#30363d')
        for spine in ax.spines.values():
            spine.set_color('#30363d')

    def time_colored_line(ax, x, y, t, cmap, lw=1.2, alpha=0.85):
        """Draw a line whose colour fades from dark to bright along time."""
        points = np.column_stack([x, y]).reshape(-1, 1, 2)
        segs = np.concatenate([points[:-1], points[1:]], axis=1)
        norm = plt.Normalize(t.min(), t.max())
        lc = LineCollection(segs, cmap=cmap, norm=norm, linewidth=lw, alpha=alpha)
        lc.set_array(t)
        ax.add_collection(lc)
        ax.autoscale()
        return lc

    # ══════════════════════════════════════════════════════════════════
    #  FIGURE 3: Phase Portraits (baseline perturbation = 1×)
    #  Row 0 — (e_d, ė_d)        error distance phase plane
    #  Row 1 — (e_θ, ė_θ)        heading error phase plane
    #  Row 2 — (v, ω)            velocity-space phase portrait
    #  Row 3 — (x, y) with time-colouring
    # ══════════════════════════════════════════════════════════════════
    fig3 = plt.figure(figsize=(5*nc, 18))
    fig3.patch.set_facecolor('#0d1117')
    gs3 = GridSpec(4, nc, figure=fig3, hspace=0.40, wspace=0.32)

    for ci, name in enumerate(ctrl_names):
        d = all_data[name]
        c = colors[name]
        t = d['t']

        # Make a sequential colourmap from dark to bright for each ctrl
        cmap = mcolors.LinearSegmentedColormap.from_list(
            f'cm_{name}', ['#0d1117', c])

        dist = d['dist']
        ang  = d['angle_err']
        v_arr = d['v']
        w_arr = d['w']

        # Numerical derivatives (smoothed)
        k = 7
        dist_dot = np.convolve(np.gradient(dist, t), np.ones(k)/k, mode='same')
        ang_dot  = np.convolve(np.gradient(ang, t),  np.ones(k)/k, mode='same')

        # ── Row 0: Distance error phase plane ─────────────────────────
        ax = fig3.add_subplot(gs3[0, ci])
        time_colored_line(ax, dist, dist_dot, t, cmap, lw=1.4)
        ax.plot(dist[0], dist_dot[0], 'o', color='white', ms=5, zorder=5)
        ax.plot(dist[-1], dist_dot[-1], 's', color=c, ms=6, zorder=5)
        style_ax(ax, f'{name}\n(e_d , ė_d)', 'e_d [m]', 'ė_d [m/s]')
        # Mark origin
        ax.axhline(0, color='white', lw=0.3, alpha=0.3)
        ax.axvline(0, color='white', lw=0.3, alpha=0.3)

        # ── Row 1: Heading error phase plane ──────────────────────────
        ax = fig3.add_subplot(gs3[1, ci])
        time_colored_line(ax, np.degrees(ang), np.degrees(ang_dot), t, cmap, lw=1.4)
        ax.plot(np.degrees(ang[0]), np.degrees(ang_dot[0]),
                'o', color='white', ms=5, zorder=5)
        ax.plot(np.degrees(ang[-1]), np.degrees(ang_dot[-1]),
                's', color=c, ms=6, zorder=5)
        style_ax(ax, f'{name}\n(θ_e , θ̇_e)', 'θ_e [°]', 'θ̇_e [°/s]')
        ax.axhline(0, color='white', lw=0.3, alpha=0.3)
        ax.axvline(0, color='white', lw=0.3, alpha=0.3)

        # ── Row 2: Velocity phase portrait (v, ω) ────────────────────
        ax = fig3.add_subplot(gs3[2, ci])
        time_colored_line(ax, v_arr, w_arr, t, cmap, lw=1.4)
        ax.plot(v_arr[0], w_arr[0], 'o', color='white', ms=5, zorder=5)
        ax.plot(v_arr[-1], w_arr[-1], 's', color=c, ms=6, zorder=5)
        style_ax(ax, f'{name}\n(v , ω)', 'v [m/s]', 'ω [rad/s]')
        ax.axhline(0, color='white', lw=0.3, alpha=0.3)
        ax.axvline(0, color='white', lw=0.3, alpha=0.3)

        # ── Row 3: XY with time colouring ────────────────────────────
        ax = fig3.add_subplot(gs3[3, ci])
        lc = time_colored_line(ax, d['x'], d['y'], t, cmap, lw=2.0)
        ax.plot(0, 0, 'o', color='white', ms=6, zorder=5, label='Start')
        ax.plot(*GOAL, '*', color=c, ms=14, zorder=5, label='Goal')
        style_ax(ax, f'{name}\nXY Trajectory', 'x [m]', 'y [m]')
        ax.set_aspect('equal')
        # Colour bar
        cb = fig3.colorbar(lc, ax=ax, fraction=0.046, pad=0.04)
        cb.set_label('Time [s]', color='#8b949e', fontsize=8)
        cb.ax.tick_params(colors='#8b949e', labelsize=7)

    fig3.suptitle(
        'Phase Portraits — PuzzleBot Nonholonomic Controllers\n'
        'Time evolution: dark → bright  |  ○ start  ◼ end  |  '
        'Pfaffian: ẏcosθ − ẋsinθ = 0',
        color='white', fontsize=14, fontweight='bold', y=1.01)

    path3 = os.path.join(output_dir, 'phase_portraits.png')
    fig3.savefig(path3, dpi=170, bbox_inches='tight',
                 facecolor=fig3.get_facecolor())
    plt.close(fig3)
    print(f'Saved: {path3}')

    # ══════════════════════════════════════════════════════════════════
    #  FIGURE 4: Robustness Analysis
    #  Row 0 — XY trajectories at all perturbation levels (per ctrl)
    #  Row 1 — Distance error at all perturbation levels (per ctrl)
    #  Row 2 — Lyapunov V at all perturbation levels (per ctrl)
    # ══════════════════════════════════════════════════════════════════
    fig4 = plt.figure(figsize=(5*nc, 14))
    fig4.patch.set_facecolor('#0d1117')
    gs4 = GridSpec(3, nc, figure=fig4, hspace=0.40, wspace=0.32)

    # Perturbation level brightness mapping
    n_levels = len(PERTURB_SCALES)

    for ci, name in enumerate(ctrl_names):
        c_base = mcolors.to_rgb(colors[name])
        level_colors = []
        for j, sc in enumerate(PERTURB_SCALES):
            frac = j / max(n_levels - 1, 1)
            # Fade from white to saturated controller colour
            rc = tuple(1.0 - frac * (1.0 - ch) for ch in c_base)
            level_colors.append(rc)

        # ── Row 0: XY overlay ─────────────────────────────────────────
        ax = fig4.add_subplot(gs4[0, ci])
        for j, sc in enumerate(PERTURB_SCALES):
            d = sweep[name][sc]
            lbl = PERTURB_LABELS[j]
            lw = 2.0 if j == 2 else 1.0          # highlight 1× baseline
            ls = '-' if j >= 2 else '--'
            ax.plot(d['x'], d['y'], color=level_colors[j],
                    linewidth=lw, linestyle=ls, alpha=0.85, label=lbl)
        ax.plot(0, 0, 'o', color='white', ms=5, zorder=10)
        ax.plot(*GOAL, '*', color=colors[name], ms=12, zorder=10)
        style_ax(ax, f'{name}\nXY — Robustness', 'x [m]', 'y [m]')
        ax.set_aspect('equal')
        if ci == 0:
            ax.legend(fontsize=7, facecolor='#161b22', edgecolor='#30363d',
                      labelcolor='white', title='Perturb.', title_fontsize=7,
                      loc='lower right')

        # ── Row 1: Distance error ─────────────────────────────────────
        ax = fig4.add_subplot(gs4[1, ci])
        for j, sc in enumerate(PERTURB_SCALES):
            d = sweep[name][sc]
            lw = 2.0 if j == 2 else 1.0
            ls = '-' if j >= 2 else '--'
            ax.plot(d['t'], d['dist'], color=level_colors[j],
                    linewidth=lw, linestyle=ls, alpha=0.85,
                    label=PERTURB_LABELS[j])
        style_ax(ax, f'{name}\n‖e‖(t) — Robustness', 'Time [s]', 'Distance [m]')
        if ci == 0:
            ax.legend(fontsize=7, facecolor='#161b22', edgecolor='#30363d',
                      labelcolor='white', title='Perturb.', title_fontsize=7)

        # ── Row 2: Lyapunov ───────────────────────────────────────────
        ax = fig4.add_subplot(gs4[2, ci])
        for j, sc in enumerate(PERTURB_SCALES):
            d = sweep[name][sc]
            lw = 2.0 if j == 2 else 1.0
            ls = '-' if j >= 2 else '--'
            V_pos = np.maximum(d['V'], 1e-10)
            ax.semilogy(d['t'], V_pos, color=level_colors[j],
                        linewidth=lw, linestyle=ls, alpha=0.85,
                        label=PERTURB_LABELS[j])
        style_ax(ax, f'{name}\nV(t) — Robustness', 'Time [s]', 'log V')
        if ci == 0:
            ax.legend(fontsize=7, facecolor='#161b22', edgecolor='#30363d',
                      labelcolor='white', title='Perturb.', title_fontsize=7)

    fig4.suptitle(
        'Robustness Analysis — Perturbation Amplitude Sweep\n'
        f'Levels: {", ".join(PERTURB_LABELS)} × baseline  |  '
        f'Robot: L={L_WHEEL}m, r={R_WHEEL}m  |  '
        f'Goal=({GOAL[0]}, {GOAL[1]})',
        color='white', fontsize=14, fontweight='bold', y=1.01)

    path4 = os.path.join(output_dir, 'robustness_analysis.png')
    fig4.savefig(path4, dpi=170, bbox_inches='tight',
                 facecolor=fig4.get_facecolor())
    plt.close(fig4)
    print(f'Saved: {path4}')

    # ══════════════════════════════════════════════════════════════════
    #  FIGURE 5: Robustness Summary — heatmap + degradation curves
    # ══════════════════════════════════════════════════════════════════
    fig5 = plt.figure(figsize=(18, 10))
    fig5.patch.set_facecolor('#0d1117')
    gs5 = GridSpec(2, 3, figure=fig5, hspace=0.42, wspace=0.35)

    # ── 5a: Final distance heatmap ────────────────────────────────────
    ax = fig5.add_subplot(gs5[0, 0])
    matrix_dist = np.zeros((nc, n_levels))
    for ci, name in enumerate(ctrl_names):
        for j, sc in enumerate(PERTURB_SCALES):
            matrix_dist[ci, j] = sweep[name][sc]['dist'][-1]
    im = ax.imshow(matrix_dist, aspect='auto', cmap='inferno_r',
                   interpolation='nearest')
    ax.set_xticks(range(n_levels))
    ax.set_xticklabels(PERTURB_LABELS, color='#8b949e', fontsize=9)
    ax.set_yticks(range(nc))
    ax.set_yticklabels(ctrl_names, color='#8b949e', fontsize=9)
    for ci in range(nc):
        for j in range(n_levels):
            val = matrix_dist[ci, j]
            txt_color = 'white' if val > matrix_dist.max()*0.5 else 'black'
            ax.text(j, ci, f'{val:.3f}', ha='center', va='center',
                    fontsize=8, color=txt_color, fontweight='bold')
    style_ax(ax, 'Final Distance [m]', 'Perturbation Scale', '')
    cb = fig5.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cb.ax.tick_params(colors='#8b949e', labelsize=7)

    # ── 5b: Convergence time heatmap ──────────────────────────────────
    ax = fig5.add_subplot(gs5[0, 1])
    matrix_conv = np.zeros((nc, n_levels))
    for ci, name in enumerate(ctrl_names):
        for j, sc in enumerate(PERTURB_SCALES):
            d = sweep[name][sc]
            idx = np.where(d['dist'] < 0.1)[0]
            matrix_conv[ci, j] = d['t'][idx[0]] if len(idx) > 0 else T_FINAL
    im2 = ax.imshow(matrix_conv, aspect='auto', cmap='RdYlGn_r',
                    interpolation='nearest')
    ax.set_xticks(range(n_levels))
    ax.set_xticklabels(PERTURB_LABELS, color='#8b949e', fontsize=9)
    ax.set_yticks(range(nc))
    ax.set_yticklabels(ctrl_names, color='#8b949e', fontsize=9)
    for ci in range(nc):
        for j in range(n_levels):
            val = matrix_conv[ci, j]
            txt = f'{val:.1f}' if val < T_FINAL else '>30'
            txt_color = 'white' if val > matrix_conv.max()*0.5 else 'black'
            ax.text(j, ci, txt, ha='center', va='center',
                    fontsize=8, color=txt_color, fontweight='bold')
    style_ax(ax, 'Convergence Time [s] (to 0.1m)', 'Perturbation Scale', '')
    cb2 = fig5.colorbar(im2, ax=ax, fraction=0.046, pad=0.04)
    cb2.ax.tick_params(colors='#8b949e', labelsize=7)

    # ── 5c: Final Lyapunov heatmap ────────────────────────────────────
    ax = fig5.add_subplot(gs5[0, 2])
    matrix_V = np.zeros((nc, n_levels))
    for ci, name in enumerate(ctrl_names):
        for j, sc in enumerate(PERTURB_SCALES):
            matrix_V[ci, j] = sweep[name][sc]['V'][-1]
    im3 = ax.imshow(np.log10(np.maximum(matrix_V, 1e-6)), aspect='auto',
                    cmap='magma_r', interpolation='nearest')
    ax.set_xticks(range(n_levels))
    ax.set_xticklabels(PERTURB_LABELS, color='#8b949e', fontsize=9)
    ax.set_yticks(range(nc))
    ax.set_yticklabels(ctrl_names, color='#8b949e', fontsize=9)
    for ci in range(nc):
        for j in range(n_levels):
            val = matrix_V[ci, j]
            txt_color = 'white' if np.log10(max(val,1e-6)) < -1 else 'black'
            ax.text(j, ci, f'{val:.3f}', ha='center', va='center',
                    fontsize=8, color=txt_color, fontweight='bold')
    style_ax(ax, 'Final Lyapunov V', 'Perturbation Scale', '')
    cb3 = fig5.colorbar(im3, ax=ax, fraction=0.046, pad=0.04)
    cb3.set_label('log₁₀ V', color='#8b949e', fontsize=8)
    cb3.ax.tick_params(colors='#8b949e', labelsize=7)

    # ── 5d: Degradation curves (dist vs perturbation) ─────────────────
    ax = fig5.add_subplot(gs5[1, 0])
    for name in ctrl_names:
        finals = [sweep[name][sc]['dist'][-1] for sc in PERTURB_SCALES]
        ax.plot(PERTURB_SCALES, finals, 'o-', color=colors[name],
                linewidth=2, markersize=6, label=name, alpha=0.9)
    style_ax(ax, 'Robustness: Final Distance vs. Perturbation',
             'Perturbation Scale ×', 'Final ‖e‖ [m]')
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    # ── 5e: Degradation curves (conv time vs perturbation) ────────────
    ax = fig5.add_subplot(gs5[1, 1])
    for name in ctrl_names:
        conv_list = []
        for sc in PERTURB_SCALES:
            d = sweep[name][sc]
            idx = np.where(d['dist'] < 0.1)[0]
            conv_list.append(d['t'][idx[0]] if len(idx) > 0 else T_FINAL)
        ax.plot(PERTURB_SCALES, conv_list, 's-', color=colors[name],
                linewidth=2, markersize=6, label=name, alpha=0.9)
    style_ax(ax, 'Robustness: Convergence Time vs. Perturbation',
             'Perturbation Scale ×', 'Convergence Time [s]')
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    # ── 5f: Control effort vs perturbation ────────────────────────────
    ax = fig5.add_subplot(gs5[1, 2])
    for name in ctrl_names:
        efforts = []
        for sc in PERTURB_SCALES:
            d = sweep[name][sc]
            eff = np.sum(np.abs(d['v']) + 0.1*np.abs(d['w'])) * DT
            efforts.append(eff)
        ax.plot(PERTURB_SCALES, efforts, '^-', color=colors[name],
                linewidth=2, markersize=6, label=name, alpha=0.9)
    style_ax(ax, 'Robustness: Control Effort vs. Perturbation',
             'Perturbation Scale ×', '∫(|v|+0.1|ω|) dt')
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    fig5.suptitle(
        'Robustness Summary — PuzzleBot Controller Benchmark\n'
        'Port-Hamiltonian (Ferguson et al.): robust to M, D (Remark 11)  |  '
        'SMC/ISMC: invariant to matched disturbances',
        color='white', fontsize=13, fontweight='bold', y=1.02)

    path5 = os.path.join(output_dir, 'robustness_summary.png')
    fig5.savefig(path5, dpi=170, bbox_inches='tight',
                 facecolor=fig5.get_facecolor())
    plt.close(fig5)
    print(f'Saved: {path5}')

    # ══════════════════════════════════════════════════════════════════
    #  FIGURE 6: Phase portrait overlay — ALL controllers on one plane
    #  For visual comparison of attractor geometry
    # ══════════════════════════════════════════════════════════════════
    fig6 = plt.figure(figsize=(18, 12))
    fig6.patch.set_facecolor('#0d1117')
    gs6 = GridSpec(2, 3, figure=fig6, hspace=0.38, wspace=0.32)

    # 6a: (e_d, ė_d) overlay
    ax = fig6.add_subplot(gs6[0, 0])
    for name in ctrl_names:
        d = all_data[name]
        dist = d['dist']
        dist_dot = np.convolve(np.gradient(dist, d['t']),
                               np.ones(7)/7, mode='same')
        ax.plot(dist, dist_dot, color=colors[name], linewidth=1.2,
                label=name, alpha=0.75)
    ax.plot(0, 0, 'w+', ms=12, mew=2, zorder=10)
    style_ax(ax, 'Distance Phase Plane\n(e_d , ė_d) — All Controllers',
             'e_d [m]', 'ė_d [m/s]', fs_title=12)
    ax.axhline(0, color='white', lw=0.3, alpha=0.3)
    ax.axvline(0, color='white', lw=0.3, alpha=0.3)
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    # 6b: (θ_e, θ̇_e) overlay
    ax = fig6.add_subplot(gs6[0, 1])
    for name in ctrl_names:
        d = all_data[name]
        ang = d['angle_err']
        ang_dot = np.convolve(np.gradient(ang, d['t']),
                              np.ones(7)/7, mode='same')
        ax.plot(np.degrees(ang), np.degrees(ang_dot), color=colors[name],
                linewidth=1.2, label=name, alpha=0.75)
    ax.plot(0, 0, 'w+', ms=12, mew=2, zorder=10)
    style_ax(ax, 'Heading Phase Plane\n(θ_e , θ̇_e) — All Controllers',
             'θ_e [°]', 'θ̇_e [°/s]', fs_title=12)
    ax.axhline(0, color='white', lw=0.3, alpha=0.3)
    ax.axvline(0, color='white', lw=0.3, alpha=0.3)
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    # 6c: (v, ω) overlay
    ax = fig6.add_subplot(gs6[0, 2])
    for name in ctrl_names:
        d = all_data[name]
        ax.plot(d['v'], d['w'], color=colors[name], linewidth=1.2,
                label=name, alpha=0.75)
    ax.plot(0, 0, 'w+', ms=12, mew=2, zorder=10)
    style_ax(ax, 'Velocity Phase Plane\n(v , ω) — All Controllers',
             'v [m/s]', 'ω [rad/s]', fs_title=12)
    ax.axhline(0, color='white', lw=0.3, alpha=0.3)
    ax.axvline(0, color='white', lw=0.3, alpha=0.3)
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    # 6d: Phase portrait at max perturbation (8×) — distance plane
    ax = fig6.add_subplot(gs6[1, 0])
    sc_max = PERTURB_SCALES[-1]
    for name in ctrl_names:
        d = sweep[name][sc_max]
        dist = d['dist']
        dist_dot = np.convolve(np.gradient(dist, d['t']),
                               np.ones(7)/7, mode='same')
        ax.plot(dist, dist_dot, color=colors[name], linewidth=1.2,
                label=name, alpha=0.75)
    ax.plot(0, 0, 'w+', ms=12, mew=2, zorder=10)
    style_ax(ax, f'Distance Phase — {sc_max:.0f}× Perturbation\n(Stress Test)',
             'e_d [m]', 'ė_d [m/s]', fs_title=12)
    ax.axhline(0, color='white', lw=0.3, alpha=0.3)
    ax.axvline(0, color='white', lw=0.3, alpha=0.3)
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    # 6e: Phase portrait at max perturbation — heading plane
    ax = fig6.add_subplot(gs6[1, 1])
    for name in ctrl_names:
        d = sweep[name][sc_max]
        ang = d['angle_err']
        ang_dot = np.convolve(np.gradient(ang, d['t']),
                              np.ones(7)/7, mode='same')
        ax.plot(np.degrees(ang), np.degrees(ang_dot), color=colors[name],
                linewidth=1.2, label=name, alpha=0.75)
    ax.plot(0, 0, 'w+', ms=12, mew=2, zorder=10)
    style_ax(ax, f'Heading Phase — {sc_max:.0f}× Perturbation\n(Stress Test)',
             'θ_e [°]', 'θ̇_e [°/s]', fs_title=12)
    ax.axhline(0, color='white', lw=0.3, alpha=0.3)
    ax.axvline(0, color='white', lw=0.3, alpha=0.3)
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    # 6f: XY overlay at max perturbation
    ax = fig6.add_subplot(gs6[1, 2])
    for name in ctrl_names:
        d = sweep[name][sc_max]
        ax.plot(d['x'], d['y'], color=colors[name], linewidth=1.5,
                label=name, alpha=0.8)
    ax.plot(0, 0, 'wo', ms=7, zorder=10)
    ax.plot(*GOAL, 'r*', ms=14, zorder=10)
    style_ax(ax, f'XY Trajectory — {sc_max:.0f}× Perturbation\n(Stress Test)',
             'x [m]', 'y [m]', fs_title=12)
    ax.set_aspect('equal')
    ax.legend(fontsize=8, facecolor='#161b22', edgecolor='#30363d',
              labelcolor='white')

    fig6.suptitle(
        'Phase Diagrams — Robustness Comparison\n'
        'Top: 1× baseline perturbation  |  Bottom: 8× stress test  |  '
        '+ marks equilibrium (origin)',
        color='white', fontsize=14, fontweight='bold', y=1.01)

    path6 = os.path.join(output_dir, 'phase_robustness_comparison.png')
    fig6.savefig(path6, dpi=170, bbox_inches='tight',
                 facecolor=fig6.get_facecolor())
    plt.close(fig6)
    print(f'Saved: {path6}')

    # ── Print robustness summary table ────────────────────────────────
    print('\n' + '='*90)
    print('  ROBUSTNESS SUMMARY — Final distance [m] at each perturbation level')
    print('-'*90)
    header = f'  {"Controller":<20}'
    for lbl in PERTURB_LABELS:
        header += f' {lbl:>8}'
    print(header)
    print('-'*90)
    for name in ctrl_names:
        row = f'  {name:<20}'
        for sc in PERTURB_SCALES:
            val = sweep[name][sc]['dist'][-1]
            row += f' {val:>8.4f}'
        print(row)
    print('='*90)


# ═══════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    output_dir = os.path.expanduser('~/puzzlebot_benchmarks')
    os.makedirs(output_dir, exist_ok=True)

    print('╔══════════════════════════════════════════════════════════════╗')
    print('║  PuzzleBot Controller Benchmark (Standalone)                ║')
    print('║  5 Controllers × 30s × Mixed Perturbations                  ║')
    print('║  Pfaffian constraint: ẏcosθ − ẋsinθ = 0                    ║')
    print('╚══════════════════════════════════════════════════════════════╝')
    print()

    # Phase 1: Baseline benchmark
    print('── Phase 1: Baseline Benchmark ──────────────────────────────')
    all_data = run_benchmark()

    print('\n── Phase 2: Robustness Sweep (6 perturbation levels) ────────')
    sweep = run_robustness_sweep()

    print('\n── Phase 3: Generating Plots ────────────────────────────────')
    generate_plots(all_data, output_dir)
    generate_phase_robustness_plots(sweep, all_data, output_dir)

    print(f'\nAll results in: {output_dir}/')
    print('Done!')
