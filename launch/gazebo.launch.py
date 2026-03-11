"""
PuzzleBot Gazebo Simulation
────────────────────────────
Launches Gazebo with terrain world, spawns PuzzleBot, and starts a controller.

Usage:
  ros2 launch puzzlebot_control gazebo.launch.py
  ros2 launch puzzlebot_control gazebo.launch.py controller:=ph
  ros2 launch puzzlebot_control gazebo.launch.py controller:=smc world:=empty

Then in another terminal:
  ros2 run puzzlebot_control teleop_keyboard

Or set a goal:
  ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 1.5}}}" --once
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_control')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'puzzlebot_gazebo.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'terrain.world')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'gazebo.rviz')

    # ── Arguments ─────────────────────────────────────────────────────
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='pid',
        description='Controller: pid, smc, ismc, ctc, ph')

    world_arg = DeclareLaunchArgument(
        'world', default_value='terrain',
        description='World: terrain (with obstacles) or empty')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo GUI')

    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='2.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='1.5')

    # ── Process URDF via xacro ────────────────────────────────────────
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ── Gazebo Server + Client ────────────────────────────────────────
    gazebo_server = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen',
    )

    # ── Spawn Robot ───────────────────────────────────────────────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'puzzlebot',
            '-x', '0.0', '-y', '0.0', '-z', '0.05',
        ],
        output='screen',
    )

    # ── Robot State Publisher ─────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ── Terrain Perturbation (software-side, complements physical terrain) ──
    terrain_node = Node(
        package='puzzlebot_control',
        executable='terrain_perturb',
        name='terrain_perturbation',
        output='screen',
        parameters=[{
            'enabled': True,
            'type': 'mixed',
            'amplitude_v': 0.03,
            'amplitude_w': 0.06,
            'frequency': 0.5,
            'noise_sigma_v': 0.01,
            'noise_sigma_w': 0.02,
        }],
    )

    # ── Controller common parameters ──────────────────────────────────
    common = {
        'goal_x': 2.0, 'goal_y': 1.5,
        'max_linear_vel': 0.5, 'max_angular_vel': 3.0,
        'control_rate': 50.0, 'goal_tolerance': 0.05,
    }

    pid_node = Node(
        package='puzzlebot_control',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[common],
    )

    smc_node = Node(
        package='puzzlebot_control',
        executable='smc_controller',
        name='smc_controller',
        output='screen',
        parameters=[{**common, 'default_active': False}],
    )

    ismc_node = Node(
        package='puzzlebot_control',
        executable='ismc_controller',
        name='ismc_controller',
        output='screen',
        parameters=[{**common, 'default_active': False}],
    )

    ctc_node = Node(
        package='puzzlebot_control',
        executable='ctc_controller',
        name='ctc_controller',
        output='screen',
        parameters=[{**common, 'default_active': False}],
    )

    ph_node = Node(
        package='puzzlebot_control',
        executable='ph_controller',
        name='ph_controller',
        output='screen',
        parameters=[{**common, 'default_active': False}],
    )

    # Map controller name to node
    controller_nodes = {
        'pid': pid_node,
        'smc': smc_node,
        'ismc': ismc_node,
        'ctc': ctc_node,
        'ph': ph_node,
    }

    # ── RViz ──────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file] if os.path.exists(rviz_file) else [],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ── Build launch description ──────────────────────────────────────
    ld = LaunchDescription()

    # Arguments
    ld.add_action(controller_arg)
    ld.add_action(world_arg)
    ld.add_action(rviz_arg)
    ld.add_action(gui_arg)
    ld.add_action(goal_x_arg)
    ld.add_action(goal_y_arg)

    # Core
    ld.add_action(robot_state_pub)
    ld.add_action(gazebo_server)
    ld.add_action(spawn_robot)

    # Perturbation
    ld.add_action(terrain_node)

    # Default controller (PID) — switch via teleop keys 1-5
    # All 5 controllers run simultaneously but only the active one publishes.
    # Press 1-5 in teleop to switch. PID is active on startup.
    ld.add_action(pid_node)
    ld.add_action(smc_node)
    ld.add_action(ismc_node)
    ld.add_action(ctc_node)
    ld.add_action(ph_node)

    # Live dashboard at http://localhost:8080
    ld.add_action(Node(
        package='puzzlebot_control',
        executable='dashboard',
        name='dashboard',
        output='screen',
        parameters=[{'port': 8080}],
    ))

    # RViz
    ld.add_action(rviz_node)

    return ld
