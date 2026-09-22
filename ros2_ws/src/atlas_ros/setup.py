from glob import glob
from setuptools import setup

setup(name='atlas_ros',version='0.2.0',packages=['atlas_ros'],data_files=[
    ('share/ament_index/resource_index/packages',['resource/atlas_ros']),
    ('share/atlas_ros',['package.xml']),
    ('share/atlas_ros/launch',glob('launch/*.launch.py')),
    ('share/atlas_ros/worlds',glob('worlds/*.sdf')),
    ('share/atlas_ros/config',glob('config/*.yaml')),
],install_requires=['setuptools'],entry_points={'console_scripts':[
    'terrain_fusion_node=atlas_ros.terrain_fusion_node:main',
    'route_follower_node=atlas_ros.route_follower_node:main',
    'demo_route_node=atlas_ros.demo_route_node:main',
]})
