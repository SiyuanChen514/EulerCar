#!/usr/bin/env python3

"""
Launch file for autonomous explorer and servo nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for autonomous explorer."""
    
    # 获取配置文件目录
    src_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_file = os.path.join(src_dir, 'config', 'robot_params.yaml')

    # 声明launch参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the robot configuration file'
    )
    
    # Explorer node
    explorer_node = Node(
        package='autonomous_explorer',
        executable='explorer_node',
        name='autonomous_explorer_node',
        output='screen',
        parameters=[{
            LaunchConfiguration('config_file')
        }]
    )

    # Servo node
    servo_node = Node(
        package='servo_response',
        executable='servo_node',
        name='servo_response_node',
        output='screen',
        parameters=[{
            LaunchConfiguration('config_file')
        }]
    )
    
    return LaunchDescription([
        config_file_arg,
        explorer_node,
        servo_node
    ])
