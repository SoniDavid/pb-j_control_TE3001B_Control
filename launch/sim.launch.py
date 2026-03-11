"""
PuzzleBot Simulation Launch File
─────────────────────────────────
Launches:
  - Physics simulator (with nonholonomic constraints)
  - Terrain perturbation generator
  - Selected controller (default: PID)
  - Optional: teleop keyboard (in separate terminal)

Usage:
  ros2 launch puzzlebot_control sim.launch.py
  ros2 launch puzzlebot_control sim.launch.py controller:=ph
  ros2 launch puzzlebot_control sim.launch.py controller:=smc perturbations:=false
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_control')
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'puzzlebot.urdf')

    # ── Launch arguments ──────────────────────────────────────────────
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='pid',
        description='Controller to use: pid, smc, ismc, ctc, ph')

    perturb_arg = DeclareLaunchArgument(
        'perturbations', default_value='true',
        description='Enable terrain perturbations')

    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='2.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='1.5')

    # ── Controller node name mapping ──────────────────────────────────
    controller_executables = {
        'pid': 'pid_controller',
        'smc': 'smc_controller',
        'ismc': 'ismc_controller',
        'ctc': 'ctc_controller',
        'ph': 'ph_controller',
    }

    # ── Nodes ─────────────────────────────────────────────────────────

    # Physics simulator
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
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_theta': 0.0,
        }]
    )

    # Terrain perturbation
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
            'publish_rate': 100.0,
        }]
    )

    # Robot state publisher (for RViz)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}] if os.path.exists(urdf_file) else [],
        condition=None,
    )

    # ── Build controller nodes for each type ──────────────────────────
    # We launch all controllers but only one will be active
    # (the others stay idle until switched via /switch_controller)

    pid_node = Node(
        package='puzzlebot_control',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[{
            'goal_x': 2.0, 'goal_y': 1.5,
            'linear_kp': 0.80, 'linear_ki': 0.02, 'linear_kd': 0.15,
            'angular_kp': 2.50, 'angular_ki': 0.02, 'angular_kd': 0.30,
        }]
    )

    smc_node = Node(
        package='puzzlebot_control',
        executable='smc_controller',
        name='smc_controller',
        output='screen',
        parameters=[{
            'goal_x': 2.0, 'goal_y': 1.5,
            'lambda_v': 1.50, 'lambda_w': 3.50,
            'eta_v': 0.15, 'eta_w': 0.80,
            'phi_v': 0.05, 'phi_w': 0.10,
        }]
    )

    ismc_node = Node(
        package='puzzlebot_control',
        executable='ismc_controller',
        name='ismc_controller',
        output='screen',
        parameters=[{
            'goal_x': 2.0, 'goal_y': 1.5,
            'alpha_v': 1.20, 'alpha_w': 2.50,
            'beta_v': 0.60, 'beta_w': 1.00,
            'eta_v': 0.10, 'eta_w': 0.60,
        }]
    )

    ctc_node = Node(
        package='puzzlebot_control',
        executable='ctc_controller',
        name='ctc_controller',
        output='screen',
        parameters=[{
            'goal_x': 2.0, 'goal_y': 1.5,
            'kp_v': 2.50, 'kd_v': 1.20,
            'kp_w': 4.00, 'kd_w': 1.80,
            'mass': 0.50, 'inertia': 0.01,
        }]
    )

    ph_node = Node(
        package='puzzlebot_control',
        executable='ph_controller',
        name='ph_controller',
        output='screen',
        parameters=[{
            'goal_x': 2.0, 'goal_y': 1.5,
            'L1': 1.00, 'L2': 8.00, 'L3': 0.01,
            'D_hat': 2.00, 'k_barrier': 0.50,
            'angle_blend': 2.50,
        }]
    )

    return LaunchDescription([
        controller_arg,
        perturb_arg,
        goal_x_arg,
        goal_y_arg,
        sim_node,
        terrain_node,
        # Launch the desired controller — by default PID
        # For benchmarking, use benchmark.launch.py instead
        pid_node,
    ])
