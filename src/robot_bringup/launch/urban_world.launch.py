"""urban_world.launch.py — gazebo (urban world) + sedan spawn + bridge + EKF + RViz.
v1 launches untouched.

  ros2 launch robot_bringup urban_world.launch.py            # urban_course
  ros2 launch robot_bringup urban_world.launch.py world:=urban_2x2
  ros2 launch robot_bringup urban_world.launch.py rviz:=false # headless
  ros2 launch robot_bringup urban_world.launch.py ekf:=false  # no odom TF (run your own)
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    world = context.launch_configurations['world']
    ws = os.path.expanduser('~/vlm_av_ws')
    map_path = os.path.join(ws, 'maps', f'{world}.json')

    pkg_sedan = get_package_share_directory('sedan_description')
    pkg_urban = get_package_share_directory('urban_gazebo')
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_loc = get_package_share_directory('robotcar_localization')

    world_file = os.path.join(pkg_urban, 'worlds', f'{world}.world.sdf')
    urdf_file = os.path.join(pkg_sedan, 'urdf', 'sedan.urdf.xacro')
    bridge_cfg = os.path.join(pkg_bringup, 'config', 'bridge_sedan.yaml')
    rviz_cfg = os.path.join(pkg_bringup, 'config', 'av_stack.rviz')
    ekf_cfg = os.path.join(pkg_loc, 'config', 'ekf_sedan.yaml')

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

        # 5. EKF (robot_localization) — the ONLY publisher of the odom -> base_link
        # TF. Without it RViz's `odom` fixed frame has no transform and nothing
        # renders. Fuses /odom (wheel twist) + /imu from the bridge; publishes
        # /odom_ekf (remapped from odometry/filtered) for lane_node / local_planner.
        # Started at 9 s: ign gazebo's /clock stutters (jumps back) for the first
        # several seconds of world init, and robot_localization spams "jump back
        # in time / clearing TF buffer" and drops updates if it comes up into
        # that. By 9 s the sim clock is monotonic and the sedan (spawn at 6 s) is
        # already publishing /odom.
        TimerAction(period=9.0, actions=[
            Node(package='robot_localization', executable='ekf_node',
                 name='ekf_filter_node', output='screen',
                 parameters=[ekf_cfg, {'use_sim_time': True}],
                 remappings=[('odometry/filtered', '/odom_ekf')],
                 condition=IfCondition(LaunchConfiguration('ekf')))]),

        # 6. RViz — last, so robot_description (latched), the odom TF, and the
        # sensor topics are all up by the time it subscribes. /lane/debug_image
        # and /lane/debug_markers are BEST_EFFORT (see lane_node.cpp); the shipped
        # config sets those two displays to Best Effort so they actually render.
        # The config also carries Odometry displays for /odom_ekf (orange, the
        # fused estimate) and /odom_truth (white, the Gazebo ground truth) so the
        # EKF drift is visible at a glance.
        TimerAction(period=13.0, actions=[
            Node(package='rviz2', executable='rviz2', name='rviz2',
                 arguments=['-d', rviz_cfg],
                 parameters=[{'use_sim_time': True}],
                 condition=IfCondition(LaunchConfiguration('rviz')),
                 output='screen')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='urban_course'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='launch RViz2 with the av_stack config'),
        DeclareLaunchArgument('ekf', default_value='true',
                              description='launch the robot_localization EKF (odom->base_link TF, /odom_ekf)'),
        OpaqueFunction(function=setup),
    ])