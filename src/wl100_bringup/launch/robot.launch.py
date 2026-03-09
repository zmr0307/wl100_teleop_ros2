# WL100 Bringup — 一键启动全部节点
# TODO: 明天完善具体实现

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- 1. 机器人描述 (URDF + TF 树) ----
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('wl100_description'),
                'launch', 'display.launch.py'
            )
        )
    )

    # ---- 2. 底盘串口通信节点 ----
    chassis_node = Node(
        package='wl100_teleop',
        executable='serial_node',
        name='wl100_serial_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('wl100_bringup'),
                'config', 'chassis_params.yaml'
            )
        ],
    )

    # ---- 3. 雷达节点 ----
    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('wl100_bringup'),
                'config', 'lidar_params.yaml'
            )
        ],
    )

    return LaunchDescription([
        description_launch,
        chassis_node,
        lidar_node,
    ])
