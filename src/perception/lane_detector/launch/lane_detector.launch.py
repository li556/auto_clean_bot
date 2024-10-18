import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
# from launch.substitutions import FindPackageShare

def generate_launch_description():
     # 声明日志级别参数
    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    # 获取配置文件路径
    lane_detector_pkg_dir = get_package_share_directory('lane_detector')
    params_file_path = \
        Path(lane_detector_pkg_dir) / 'config' / 'lane_detector_config.yaml'
    print(params_file_path)

    lane_detector_node = Node(
            package='lane_detector',
            # namespace='test_control',
            executable='lane_detector_node',
            output='screen',    # show print info in terminal
            parameters=[params_file_path],
            arguments=['--ros-args', '--log-level', log_level]
    )
    return LaunchDescription([
        declare_log_level,
        lane_detector_node
    ])