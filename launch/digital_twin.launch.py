"""
PuzzleBot Digital Twin Launch
──────────────────────────────
Runs the real robot AND mirrors its pose into Gazebo simultaneously.

  Real robot (ESP32 + encoders)
    → puzzlebot_hardware  →  /odom  →  controllers  →  /cmd_vel  →  real motors
                                   ↓
                              gazebo_mirror  →  /gazebo/set_entity_state
                                   ↓
                              Gazebo GUI  (pure visualisation, no physics control)

Gazebo is NOT the plant here. It is a live 3-D display of where the real
robot actually is, updated from encoder odometry at 20 Hz.

Usage
─────
  # Terminal 1 — start Micro-ROS agent first:
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

  # Terminal 2 — launch everything:
  ros2 launch puzzlebot_control digital_twin.launch.py

  # Optional overrides:
  ros2 launch puzzlebot_control digital_twin.launch.py controller:=smc gui:=false

Arguments
─────────
  controller   : pid | smc | ismc | ctc | ph   (default: pid)
  goal_x/goal_y: initial goal position          (default: 2.0 / 1.5)
  gui          : true | false — show Gazebo GUI (default: true)
  rviz         : true | false                   (default: true)
  teleop       : true | false                   (default: false)
  rpm_max      : maximum wheel RPM              (default: 110.0)
  mirror_rate  : Hz at which gazebo_mirror polls (default: 20.0)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = FindPackageShare('puzzlebot_control')
    pkg_dir   = get_package_share_directory('puzzlebot_control')

    # ── Arguments ────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('controller',   default_value='pid'),
        DeclareLaunchArgument('goal_x',       default_value='2.0'),
        DeclareLaunchArgument('goal_y',       default_value='1.5'),
        DeclareLaunchArgument('wheel_base',   default_value='0.19'),
        DeclareLaunchArgument('wheel_radius', default_value='0.05'),
        DeclareLaunchArgument('rpm_max',      default_value='110.0'),
        DeclareLaunchArgument('mirror_rate',  default_value='20.0',
            description='Hz rate for Gazebo mirror updates'),
        DeclareLaunchArgument('gui',   default_value='true',
            description='Show Gazebo GUI'),
        DeclareLaunchArgument('rviz',  default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false'),
    ]

    # ── Gazebo (visualisation only — robot will be teleported, not simulated) ──
    world_file = os.path.join(pkg_dir, 'worlds', 'terrain.world')

    gazebo_server = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen',
    )

    # Spawn a static copy of the robot into Gazebo (starting pose 0,0)
    # The gazebo_mirror node will move it from here on.
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

    # ── Robot description (TF / RViz) ─────────────────────────────────
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'puzzlebot_gazebo.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ── Hardware bridge — the REAL plant ──────────────────────────────
    # Publishes /odom from encoder RPM, drives motors via /motor_*/ PWM topics.
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

    # ── Gazebo mirror — the digital twin bridge ───────────────────────
    # Reads /odom (from real encoders) → teleports Gazebo model each tick.
    mirror_node = Node(
        package='puzzlebot_control',
        executable='gazebo_mirror',
        name='gazebo_mirror',
        output='screen',
        parameters=[{
            'entity_name':     'puzzlebot',
            'reference_frame': '',           # world frame
            'update_rate':     LaunchConfiguration('mirror_rate'),
        }],
    )

    # ── Controllers (identical configuration to sim/hardware launches) ─
    common = {
        'max_linear_vel':  0.5,
        'max_angular_vel': 3.0,
        'control_rate':    50.0,
        'goal_tolerance':  0.05,
    }

    def ctrl_node(executable, active):
        return Node(
            package='puzzlebot_control', executable=executable,
            name=executable, output='screen',
            parameters=[{**common,
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'default_active': active}],
        )

    # ── Dashboard ─────────────────────────────────────────────────────
    dashboard_node = Node(
        package='puzzlebot_control', executable='dashboard',
        name='dashboard', output='screen',
        parameters=[{'port': 8080}],
    )

    # ── RViz ──────────────────────────────────────────────────────────
    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'gazebo.rviz'])
    rviz_node = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ── Teleop (optional) ─────────────────────────────────────────────
    teleop_node = Node(
        package='puzzlebot_control', executable='teleop_keyboard',
        name='teleop_keyboard', output='screen', prefix='xterm -e',
        condition=IfCondition(LaunchConfiguration('teleop')),
    )

    # ── Assemble ──────────────────────────────────────────────────────
    ld = LaunchDescription()
    for a in args:
        ld.add_action(a)

    # 1. Gazebo (visualisation)
    ld.add_action(gazebo_server)
    ld.add_action(robot_state_pub)
    ld.add_action(spawn_robot)

    # 2. Real hardware
    ld.add_action(hardware_node)

    # 3. Digital twin bridge
    ld.add_action(mirror_node)

    # 4. Controllers (all 5, only active one publishes)
    ld.add_action(ctrl_node('pid_controller',  True))
    ld.add_action(ctrl_node('smc_controller',  False))
    ld.add_action(ctrl_node('ismc_controller', False))
    ld.add_action(ctrl_node('ctc_controller',  False))
    ld.add_action(ctrl_node('ph_controller',   False))

    # 5. Monitoring
    ld.add_action(dashboard_node)
    ld.add_action(rviz_node)
    ld.add_action(teleop_node)

    return ld
