"""Launch file for graspnet demo with bundled model and example_data."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('graspnet')

    checkpoint_path = os.path.join(share_dir, 'model', 'checkpoint-rs.tar')
    data_dir = os.path.join(share_dir, 'example_data')

    checkpoint_arg = DeclareLaunchArgument(
        'checkpoint_path',
        default_value=checkpoint_path,
        description='Path to model checkpoint'
    )
    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value=data_dir,
        description='Path to example data directory'
    )
    num_point_arg = DeclareLaunchArgument(
        'num_point',
        default_value='20000',
        description='Number of points to sample'
    )
    num_view_arg = DeclareLaunchArgument(
        'num_view',
        default_value='300',
        description='Number of grasp views'
    )
    collision_thresh_arg = DeclareLaunchArgument(
        'collision_thresh',
        default_value='0.01',
        description='Collision detection threshold'
    )
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.01',
        description='Voxel size for collision detection'
    )

    demo_node = Node(
        package='graspnet',
        executable='graspnet_demo',
        name='graspnet_demo',
        output='screen',
        arguments=[
            '--checkpoint_path', LaunchConfiguration('checkpoint_path'),
            '--data_dir', LaunchConfiguration('data_dir'),
            '--num_point', LaunchConfiguration('num_point'),
            '--num_view', LaunchConfiguration('num_view'),
            '--collision_thresh', LaunchConfiguration('collision_thresh'),
            '--voxel_size', LaunchConfiguration('voxel_size'),
        ],
    )

    return LaunchDescription([
        checkpoint_arg,
        data_dir_arg,
        num_point_arg,
        num_view_arg,
        collision_thresh_arg,
        voxel_size_arg,
        demo_node,
    ])