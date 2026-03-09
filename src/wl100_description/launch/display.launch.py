# WL100 Description — robot_state_publisher 启动文件
# TODO: 明天完善具体实现

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # 获取 URDF 文件路径
    pkg_description = get_package_share_directory('wl100_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'wl100.urdf.xacro')

    # 解析 xacro → URDF
    robot_description = xacro.process_file(xacro_file).toxml()

    # robot_state_publisher 节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
