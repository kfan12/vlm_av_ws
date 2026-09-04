"""av_stack.launch.py - the full stack in one command.

Brings up urban_world (gazebo + sedan + bridge + EKF + odom_compare + RViz) and
layers the perception / planning / control nodes on top. This launch is the
single place that reads the map spawn pose and fans it out as map_origin_* to
every map consumer - lane_node and local_planner both transform the world-frame
map into the spawn-zeroed odom frame, and a mismatch here puts the planned path
metres off the vehicle.

  ros2 launch robot_bringup av_stack.launch.py
  ros2 launch robot_bringup av_stack.launch.py world:=urban_2x2
  ros2 launch robot_bringup av_stack.launch.py rviz:=false
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    world = context.launch_configurations['world']
    ws = os.path.expanduser('~/vlm_av_ws')
    map_path = os.path.join(ws, 'maps', f'{world}.json')

    pkg_bringup = get_package_share_directory('robot_bringup')
    world_launch = os.path.join(pkg_bringup, 'launch', 'urban_world.launch.py')

    # Spawn pose = origin of the odom frame. Same read as urban_world.launch.py;
    # kept here too so the map_origin_* fan-out has a single owner.
    spawn = {'x': 10.0, 'y': -1.75, 'yaw': 0.0}
    try:
        with open(map_path) as f:
            spawn = {**spawn, **json.load(f).get('spawn', {})}
    except OSError:
        pass

    map_origin = {
        'use_sim_time': True,
        'map_path': map_path,
        'map_origin_x': float(spawn['x']),
        'map_origin_y': float(spawn['y']),
        'map_origin_yaw': float(spawn['yaw']),
    }

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={
                'world': world,
                'rviz': LaunchConfiguration('rviz'),
                'ekf': LaunchConfiguration('ekf'),
            }.items()),

        # Perception + planning + control. Started after EKF (urban_world starts
        # it at 9 s) so /odom_ekf is already flowing.
        TimerAction(period=12.0, actions=[
            Node(package='av_perception_cpp', executable='lane_node',
                 name='lane_node', output='screen',
                 parameters=[map_origin]),
            Node(package='av_behavior_cpp', executable='local_planner',
                 name='local_planner', output='screen',
                 parameters=[map_origin]),
            Node(package='av_behavior_cpp', executable='mpc_tracker_v2',
                 name='mpc_tracker_v2', output='screen',
                 parameters=[{'use_sim_time': True}]),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='urban_course'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ekf', default_value='true'),
        OpaqueFunction(function=setup),
    ])
