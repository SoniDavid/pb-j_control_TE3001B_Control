"""
PuzzleBot Hardware Launch
──────────────────────────
Launches everything needed to run the real PuzzleBot hardware:
  • puzzlebot_hardware  — hardware bridge (cmd_vel ↔ motor PWM + encoder odometry)
  • robot_state_publisher — URDF/TF for RViz
  • All 5 controllers     — same as simulation (only active one publishes)
  • dashboard             — live web dashboard at http://localhost:8080
  • rviz2                 — optional visualisation
  • teleop_keyboard       — optional keyboard teleop

NOTE: Does NOT launch Gazebo. For simulation use gazebo.launch.py or sim.launch.py.
The hardware bridge is the only plant node — it owns /odom and /cmd_vel.

Usage
─────
  # In a separate terminal — start the Micro-ROS agent FIRST:
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

  # Then launch:
  ros2 launch puzzlebot_control hardware.launch.py

  # With option overrides:
  ros2 launch puzzlebot_control hardware.launch.py controller:=smc goal_x:=1.5 goal_y:=1.0

Arguments
─────────
  controller  : pid | smc | ismc | ctc | ph   (initial active controller)
  serial_port : /dev/ttyUSB0 (port used by the Micro-ROS agent message)
  goal_x      : 2.0
  goal_y      : 1.5
  wheel_base  : 0.19
  wheel_radius: 0.05
  rpm_max     : 110.0
  rviz        : true | false
  teleop      : false | true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    pkg_share = FindPackageShare('puzzlebot_control')

    # ── Arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('controller',   default_value='pid',
            description='Initial active controller: pid smc ismc ctc ph'),
        DeclareLaunchArgument('serial_port',  default_value='/dev/ttyUSB0',
            description='Serial port of the ESP32 (shown in hint message only)'),
        DeclareLaunchArgument('goal_x',       default_value='2.0'),
        DeclareLaunchArgument('goal_y',       default_value='1.5'),
        DeclareLaunchArgument('wheel_base',   default_value='0.19'),
        DeclareLaunchArgument('wheel_radius', default_value='0.05'),
        DeclareLaunchArgument('rpm_max',      default_value='110.0'),
        DeclareLaunchArgument('rviz',         default_value='true'),
        DeclareLaunchArgument('teleop',       default_value='false'),
    ]

    # ── Robot description (URDF for TF/RViz only — no physics) ───────
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'puzzlebot_gazebo.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ── Hardware bridge ────────────────────────────────────────────────
    hardware_node = Node(
        package='puzzlebot_control',
        executable='puzzlebot_hardware',
        name='puzzlebot_hardware',
        output='screen',
        parameters=[{
            'wheel_base':     LaunchConfiguration('wheel_base'),
            'wheel_radius':   LaunchConfiguration('wheel_radius'),
            'rpm_max':        LaunchConfiguration('rpm_max'),
            'max_linear_vel':  0.5,
            'max_angular_vel': 3.0,
            'sample_time':     0.05,
        }],
    )

    # ── Common controller parameters (identical to sim) ────────────────
    common = {
        'max_linear_vel':  0.5,
        'max_angular_vel': 3.0,
        'control_rate':   50.0,
        'goal_tolerance':  0.05,
    }
    common_with_goal = {
        **common,
        'goal_x': LaunchConfiguration('goal_x'),
        'goal_y': LaunchConfiguration('goal_y'),
    }

    pid_node = Node(
        package='puzzlebot_control', executable='pid_controller',
        name='pid_controller',  output='screen',
        parameters=[{**common, 'goal_x': LaunchConfiguration('goal_x'),
                                'goal_y': LaunchConfiguration('goal_y'),
                                'default_active': True}],
    )
    smc_node = Node(
        package='puzzlebot_control', executable='smc_controller',
        name='smc_controller',  output='screen',
        parameters=[{**common, 'goal_x': LaunchConfiguration('goal_x'),
                                'goal_y': LaunchConfiguration('goal_y'),
                                'default_active': False}],
    )
    ismc_node = Node(
        package='puzzlebot_control', executable='ismc_controller',
        name='ismc_controller', output='screen',
        parameters=[{**common, 'goal_x': LaunchConfiguration('goal_x'),
                                'goal_y': LaunchConfiguration('goal_y'),
                                'default_active': False}],
    )
    ctc_node = Node(
        package='puzzlebot_control', executable='ctc_controller',
        name='ctc_controller',  output='screen',
        parameters=[{**common, 'goal_x': LaunchConfiguration('goal_x'),
                                'goal_y': LaunchConfiguration('goal_y'),
                                'default_active': False}],
    )
    ph_node = Node(
        package='puzzlebot_control', executable='ph_controller',
        name='ph_controller',   output='screen',
        parameters=[{**common, 'goal_x': LaunchConfiguration('goal_x'),
                                'goal_y': LaunchConfiguration('goal_y'),
                                'default_active': False}],
    )

    # ── Dashboard ──────────────────────────────────────────────────────
    dashboard_node = Node(
        package='puzzlebot_control',
        executable='dashboard',
        name='dashboard',
        output='screen',
        parameters=[{'port': 8080}],
    )

    # ── RViz ──────────────────────────────────────────────────────────
    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'gazebo.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ── Teleop (optional) ─────────────────────────────────────────────
    teleop_node = Node(
        package='puzzlebot_control',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        condition=IfCondition(LaunchConfiguration('teleop')),
    )

    # ── Assemble ───────────────────────────────────────────────────────
    ld = LaunchDescription()
    for a in args:
        ld.add_action(a)

    ld.add_action(robot_state_pub)
    ld.add_action(hardware_node)

    ld.add_action(pid_node)
    ld.add_action(smc_node)
    ld.add_action(ismc_node)
    ld.add_action(ctc_node)
    ld.add_action(ph_node)

    ld.add_action(dashboard_node)
    ld.add_action(rviz_node)
    ld.add_action(teleop_node)

    return ld
