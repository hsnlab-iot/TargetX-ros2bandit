from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mock_control_pkg',
            executable='control',
            name='simulated_control',
            output='screen'
        ),
    ])