from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mock_robot_pkg',
            executable='robot',
            name='simulated_rover',
            output='screen'
        ),
    ])