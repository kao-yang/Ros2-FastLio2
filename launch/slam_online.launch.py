from launch import LaunchDescription
from launch_ros.actions import Node
import os

current_file_path = os.path.abspath(__file__)
parent_directory = os.path.dirname(current_file_path)
parent_directory = os.path.dirname(parent_directory)
rviz_file = parent_directory + "/rviz_cfg/slam.rviz"

def generate_launch_description():    
    return LaunchDescription([
        Node(
            package='ros2_fast_lio2',
            executable='slam_online',
            name='slam_online',
            arguments=['-config_dir', 'rs16/'],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
        )
    ])