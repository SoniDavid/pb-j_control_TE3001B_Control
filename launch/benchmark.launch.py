"""
PuzzleBot Benchmark Launch File
─────────────────────────────────
Runs the full Lyapunov benchmarking suite:
  - Physics simulator with perturbations
  - Benchmark node (auto-cycles through all 5 controllers)
  - Each controller runs for 30s, then switches

Generates comparison plots in ~/puzzlebot_benchmarks/

Usage:
  ros2 launch puzzlebot_control benchmark.launch.py
  ros2 launch puzzlebot_control benchmark.launch.py duration:=20
  ros2 launch puzzlebot_control benchmark.launch.py perturbation_type:=sinusoidal
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────
    duration_arg = DeclareLaunchArgument(
        'duration', default_value='30.0',
        description='Duration per controller [s]')

    perturb_type_arg = DeclareLaunchArgument(
        'perturbation_type', default_value='mixed',
        description='Perturbation: none, sinusoidal, step, noise, mixed')

    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='2.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='1.5')

    # ── Physics Simulator ─────────────────────────────────────────────
    sim_node = Node(
        package='puzzlebot_control',
        executable='puzzlebot_sim',
        name='puzzlebot_sim',
        output='screen',
        parameters=[{
            'wheel_base': 0.19,
            'wheel_radius': 0.05,
            'mass': 0.5,
            'inertia': 0.01,
            'max_linear_vel': 0.5,
            'max_angular_vel': 3.0,
            'dt': 0.01,
        }]
    )

    # ── Terrain Perturbation ──────────────────────────────────────────
    terrain_node = Node(
        package='puzzlebot_control',
        executable='terrain_perturb',
        name='terrain_perturbation',
        output='screen',
        parameters=[{
            'enabled': True,
            'type': 'mixed',
            'amplitude_v': 0.05,
            'amplitude_w': 0.1,
            'frequency': 0.5,
            'step_interval': 5.0,
            'noise_sigma_v': 0.02,
            'noise_sigma_w': 0.04,
        }]
    )

    # ── All 5 Controllers (benchmark node will switch between them) ───
    # We launch all so the benchmark node can publish /switch_controller
    # and the correct one responds. Each publishes on /cmd_vel.
    #
    # NOTE: In a production setup you'd use a controller_manager or
    # namespace isolation. For benchmarking, each controller checks
    # /switch_controller and only the active one publishes.

    common_params = {
        'goal_x': 2.0,
        'goal_y': 1.5,
        'max_linear_vel': 0.5,
        'max_angular_vel': 3.0,
        'control_rate': 100.0,
        'goal_tolerance': 0.05,
    }

    pid_node = Node(
        package='puzzlebot_control',
        executable='pid_controller',
        name='pid_controller',
        output='log',
        parameters=[{
            **common_params,
            'linear_kp': 0.80, 'linear_ki': 0.02, 'linear_kd': 0.15,
            'angular_kp': 2.50, 'angular_ki': 0.02, 'angular_kd': 0.30,
        }]
    )

    smc_node = Node(
        package='puzzlebot_control',
        executable='smc_controller',
        name='smc_controller',
        output='log',
        parameters=[{
            **common_params,
            'lambda_v': 1.50, 'lambda_w': 3.50,
            'eta_v': 0.15, 'eta_w': 0.80,
        }]
    )

    ismc_node = Node(
        package='puzzlebot_control',
        executable='ismc_controller',
        name='ismc_controller',
        output='log',
        parameters=[{
            **common_params,
            'alpha_v': 1.20, 'alpha_w': 2.50,
            'beta_v': 0.60, 'beta_w': 1.00,
            'eta_v': 0.10, 'eta_w': 0.60,
        }]
    )

    ctc_node = Node(
        package='puzzlebot_control',
        executable='ctc_controller',
        name='ctc_controller',
        output='log',
        parameters=[{
            **common_params,
            'kp_v': 2.50, 'kd_v': 1.20,
            'kp_w': 4.00, 'kd_w': 1.80,
        }]
    )

    ph_node = Node(
        package='puzzlebot_control',
        executable='ph_controller',
        name='ph_controller',
        output='log',
        parameters=[{
            **common_params,
            'L1': 1.00, 'L2': 8.00, 'L3': 0.01,
            'D_hat': 2.00, 'k_barrier': 0.50, 'angle_blend': 2.50,
        }]
    )

    # ── Benchmark Orchestrator ────────────────────────────────────────
    benchmark_node = Node(
        package='puzzlebot_control',
        executable='lyapunov_benchmark',
        name='lyapunov_benchmark',
        output='screen',
        parameters=[{
            'duration': 30.0,
            'sample_rate': 50.0,
            'output_dir': '~/puzzlebot_benchmarks',
            'goal_x': 2.0,
            'goal_y': 1.5,
            'auto_cycle': True,
        }]
    )

    return LaunchDescription([
        duration_arg,
        perturb_type_arg,
        goal_x_arg,
        goal_y_arg,
        sim_node,
        terrain_node,
        pid_node,
        smc_node,
        ismc_node,
        ctc_node,
        ph_node,
        benchmark_node,
    ])
