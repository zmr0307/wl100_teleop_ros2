import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config', 'unilidar_l2.yaml')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    return LaunchDescription([
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[
                default_config_path
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', default_rviz_config_path],
            prefix='nice',
        )
    ])
