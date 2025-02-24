from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello_task',
            executable='sine_wave_publisher',
            name='sine_wave_publisher',
        ),
        Node(
            package='hello_task',
            executable='sine_wave_subscriber',
            name='sine_wave_subscriber',
        )
    ])
