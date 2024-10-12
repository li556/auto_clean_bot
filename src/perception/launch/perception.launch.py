import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
     # 声明日志级别参数
    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    DeclareLaunchArgument(
        'config_file',
        default_value='config/perception_config.yaml',
        description='Path to the YAML configuration file'
    )
    perception_node = Node(
            package='perception',
            # namespace='test_control',
            executable='perception_node',
            output='screen',    # show print info in terminal
            parameters=[],
            arguments=['--ros-args', '--log-level', log_level]
    )
    return LaunchDescription([
        declare_log_level,
        perception_node
    ])