"""urban_world.launch.py — gazebo (urban world) + sedan spawn + bridge.
v1 launches untouched.

  ros2 launch robot_bringup urban_world.launch.py            # urban_course
  ros2 launch robot_bringup urban_world.launch.py world:=urban_2x2
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    world = context.launch_configurations['world']
    ws = os.path.expanduser('~/vlm_av_ws')
    map_path = os.path.join(ws, 'maps', f'{world}.json')

    pkg_sedan = get_package_share_directory('sedan_description')
    pkg_urban = get_package_share_directory('urban_gazebo')
    pkg_bringup = get_package_share_directory('robot_bringup')

    world_file = os.path.join(pkg_urban, 'worlds', f'{world}.world.sdf')
    urdf_file = os.path.join(pkg_sedan, 'urdf', 'sedan.urdf.xacro')
    bridge_cfg = os.path.join(pkg_bringup, 'config', 'bridge_sedan.yaml')

    # spawn pose from the map (generator emits it)
    spawn = {'x': 10.0, 'y': -1.75, 'yaw': 0.0}
    try:
        with open(map_path) as f:
            spawn = json.load(f).get('spawn', spawn)
    except OSError:
        pass

    robot_description = ParameterValue(Command(['xacro ', urdf_file]),
                                       value_type=str)

    # model:// resolution for the course-demo sign boards: urban_gazebo/models
    # (sign_*_urban) + robotcar_gazebo/models/signs (the v1 meshes/textures the
    # urban models reference). Path entries must be the immediate PARENT of
    # each model.config dir. Prepend, preserving anything already set.
    ign_resource_path = os.pathsep.join([
        os.path.join(pkg_urban, 'models'),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])

    return [
        # 1. Gazebo — server+run; NEVER --headless-rendering (WSL freeze)
        ExecuteProcess(
            cmd=['ign', 'gazebo','-r', world_file],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1',
                            'IGN_GAZEBO_RESOURCE_PATH': ign_resource_path},
            output='screen'),

        # 2. robot_state_publisher
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': True}]),

        # 3. bridge after Gazebo loads (CLAUDE.md ordering). The joint_state
        # topic embeds the world name, so it is bridged as a positional arg
        # built from `world` (the yaml holds only world-independent topics —
        # a hardcoded urban_2x2 there broke wheel TFs on urban_course).
        # TWO separate parameter_bridge nodes: combining a CLI positional
        # topic arg with a config_file param on ONE node silently drops every
        # config_file bridge (odom/imu/camera/cmd_vel/clock all vanished,
        # confirmed by log diff 2026-07-16 — no error, just missing). Each
        # mechanism works fine alone, so run them as separate nodes.
        TimerAction(period=3.0, actions=[
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='gz_ros_bridge_joint_state',
                 arguments=[f'/world/{world}/model/sedan/joint_state@'
                            'sensor_msgs/msg/JointState[ignition.msgs.Model'],
                 remappings=[(f'/world/{world}/model/sedan/joint_state',
                              '/joint_states')],
                 output='screen'),
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='gz_ros_bridge',
                 parameters=[{'config_file': bridge_cfg}],
                 output='screen')]),

        # 4. spawn sedan (world name explicit — multi-world gotcha)
        TimerAction(period=6.0, actions=[
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'sedan', '-world', world,
                            '-topic', 'robot_description',
                            '-x', str(spawn['x']), '-y', str(spawn['y']),
                            '-z', '0.45', '-Y', str(spawn['yaw'])],
                 output='screen')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='urban_course'),
        OpaqueFunction(function=setup),
    ])