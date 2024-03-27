from launch import LaunchDescription
from launch_ros.actions import Node

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
            output='screen'
        )
    ])