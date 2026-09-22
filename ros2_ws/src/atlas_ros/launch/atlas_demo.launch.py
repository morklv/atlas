"""Launch the ATLAS Gazebo scenario, ROS bridge, and route follower."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    share = get_package_share_directory('atlas_ros')
    world = join(share, 'worlds', 'atlas_terrain.sdf')
    bridge = join(share, 'config', 'bridge.yaml')
    return LaunchDescription([
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', world], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['--ros-args', '-p', f'config_file:={bridge}'], output='screen'),
        Node(package='atlas_ros', executable='route_follower_node', output='screen'),
        Node(package='atlas_ros', executable='demo_route_node', output='screen'),
    ])
