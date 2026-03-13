# PuzzleBot Nonholonomic Control Framework
| Nombre                         | Matrícula  |
|--------------------------------|-----------|
| David Alejandro Soni Cuevas    | A01571777 |
| Irvin David Ornelas Garcia     | A00839065 |
| Abraham de Jesús Maldonado Mata| A00838581 |
| David Gilberto Lomelí          | A01571193 |


Five controllers for differential-drive robots with terrain perturbations, Lyapunov stability analysis, phase portraits with sliding surface visualization, and custom trajectory tracking.

Based on: **Ferguson, Donaire, Renton & Middleton (2018)** — *A port-Hamiltonian approach to the control of nonholonomic systems* (arXiv:1801.06954v1).

## Prerequisites

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-gazebo-ros-pkgs \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO}-rviz2
pip3 install tf-transformations numpy matplotlib
```

## Build

```bash
mkdir -p ~/pb_ws/src
cp -r puzzlebot_control ~/pb_ws/src/
cd ~/pb_ws
colcon build --packages-select puzzlebot_control
source install/setup.bash
```

## Quick Start (3 terminals)

```bash
# Terminal 1 — Gazebo + controllers + dashboard
ros2 launch puzzlebot_control gazebo.launch.py

# Terminal 2 — Keyboard teleop
ros2 run puzzlebot_control teleop_keyboard

# Terminal 3 — Open browser
xdg-open http://localhost:8080
```

## Dashboard (localhost:8080) — 12 Live Charts

| Chart | What it shows |
|---|---|
| Lyapunov V(t) | V = ½‖e‖² + ½θ_e² decreasing to 0 |
| XY Trajectory | Robot path + goal. **Click to draw waypoints.** |
| Phase (e_d, ė_d) | Distance phase portrait. **White dashed = sliding surface s_v=0** |
| Distance Error | ‖e‖(t) convergence to 0 |
| Control v(t), ω(t) | Commanded velocities |
| Phase (θ_e, θ̇_e) | Heading phase portrait. **White dashed = sliding surface s_w=0** |
| V̇(t) | Lyapunov derivative — must stay ≤ 0 for stability |
| Sliding Surfaces s(t) | s_v(t), s_w(t) — converge to 0 for SMC/ISMC |
| Phase (v, ω) | Velocity-space attractor geometry |
| Heading Error | θ_e(t) in degrees |
| Perturbations | Terrain disturbance forces F_v, τ_w |
| Sliding Phase (s_v, ṡ_v) | Sliding surface phase portrait — convergence to origin |

All charts show **full history** (auto-downsampled for performance).

## Custom Trajectories

1. Click on XY chart to place waypoints → click **Send Drawn**
2. Or use presets: Circle, Figure-8, Square, Zigzag
3. Or publish from terminal:
```bash
ros2 topic pub /trajectory nav_msgs/Path "{header: {frame_id: 'odom'}, poses: [
  {pose: {position: {x: 1.0, y: 0.0}}},
  {pose: {position: {x: 2.0, y: 1.0}}},
  {pose: {position: {x: 0.0, y: 2.0}}}
]}" --once
```

## Keyboard Controls

```
↑/W Forward    ↓/S Backward    ←/A Left    →/D Right
SPACE Stop    G Goal    R Reset    P Perturbations    Q Quit
1 PID    2 SMC    3 ISMC    4 CTC    5 Port-Hamiltonian
```

## Reset (R key or dashboard button)

1. Calls Gazebo /reset_simulation → robot returns to origin
2. All controllers reset integrals and state
3. Dashboard charts clear completely

## Controllers

| Controller | Sliding Surface | Phase Portrait Shows |
|---|---|---|
| PID | — | Smooth spiral to origin |
| SMC | s_v=e_d, s_w=θ_e | Reaching → hits s=0 line → slides to origin |
| ISMC | σ=e+α∫e | Starts ON surface (no reaching phase) |
| CTC | — | Linearized dynamics spiral |
| Port-Hamiltonian | — | Energy-shaped convergence |

## Standalone Benchmark (no ROS2)

```bash
pip3 install numpy matplotlib
python3 standalone_benchmark.py
```
# pb-j_control
