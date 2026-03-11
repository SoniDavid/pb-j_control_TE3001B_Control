"""
PuzzleBot Gazebo Benchmark
───────────────────────────
Launches Gazebo with terrain world, spawns PuzzleBot, starts all controllers
and the Lyapunov benchmark node that cycles through them.

Usage:
  ros2 launch puzzlebot_control gazebo_benchmark.launch.py
  ros2 launch puzzlebot_control gazebo_benchmark.launch.py duration:=20
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_control')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'puzzlebot_gazebo.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'terrain.world')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    common = {
        'goal_x': 2.0, 'goal_y': 1.5,
        'max_linear_vel': 0.5, 'max_angular_vel': 3.0,
        'control_rate': 50.0, 'goal_tolerance': 0.05,
    }

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='30.0'),

        # Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
                 world_file],
            output='screen',
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'puzzlebot',
                       '-x', '0', '-y', '0', '-z', '0.05'],
            output='screen',
        ),

        # Terrain perturbation (software layer)
        Node(
            package='puzzlebot_control', executable='terrain_perturb',
            name='terrain_perturbation', output='log',
            parameters=[{'enabled': True, 'type': 'mixed',
                         'amplitude_v': 0.03, 'amplitude_w': 0.06}],
        ),

        # All 5 controllers
        Node(package='puzzlebot_control', executable='pid_controller',
             name='pid_controller', output='log',
             parameters=[{**common,
                          'linear_kp': 1.2, 'linear_ki': 0.05, 'linear_kd': 0.2,
                          'angular_kp': 3.0, 'angular_ki': 0.05, 'angular_kd': 0.35}]),
        Node(package='puzzlebot_control', executable='smc_controller',
             name='smc_controller', output='log',
             parameters=[{**common,
                          'lambda_v': 1.5, 'lambda_w': 3.5,
                          'eta_v': 0.15, 'eta_w': 0.8}]),
        Node(package='puzzlebot_control', executable='ismc_controller',
             name='ismc_controller', output='log',
             parameters=[{**common,
                          'alpha_v': 0.3, 'alpha_w': 0.8,
                          'beta_v': 1.5, 'beta_w': 3.0,
                          'eta_v': 0.12, 'eta_w': 0.7}]),
        Node(package='puzzlebot_control', executable='ctc_controller',
             name='ctc_controller', output='log',
             parameters=[{**common,
                          'kp_v': 1.8, 'kd_v': 0.8,
                          'kp_w': 3.5, 'kd_w': 0.6}]),
        Node(package='puzzlebot_control', executable='ph_controller',
             name='ph_controller', output='log',
             parameters=[{**common,
                          'L1': 1.0, 'L2': 8.0, 'L3': 0.01,
                          'D_hat': 2.0, 'k_barrier': 0.5, 'angle_blend': 2.5}]),

        # Benchmark orchestrator
        Node(
            package='puzzlebot_control', executable='lyapunov_benchmark',
            name='lyapunov_benchmark', output='screen',
            parameters=[{
                'duration': 30.0, 'sample_rate': 50.0,
                'output_dir': '~/puzzlebot_benchmarks',
                'goal_x': 2.0, 'goal_y': 1.5, 'auto_cycle': True,
            }],
        ),
    ])
