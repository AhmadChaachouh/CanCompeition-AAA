from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pepsi_detector',
            executable='pepsi_publisher.py',
            name='pepsi_publisher',
            output='screen',
            parameters=[]  # If you have any parameters
        ),
        Node(
            package='pepsi_detector',
            executable='pepsi_subscriber',
            name='pepsi_subscriber',
            output='screen'
        ),
    ])

