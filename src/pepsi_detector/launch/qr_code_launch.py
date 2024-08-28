from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pepsi_detector',
            executable='qr_code_detector.py',
            name='qr_code_detector',
            output='screen',
            parameters=[]  # If you have any parameters
        ),
        Node(
            package='pepsi_detector',
            executable='qr_code_subscriber',
            name='qr_code_subscriber',
            output='screen'
        ),
    ])