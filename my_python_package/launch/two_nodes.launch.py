from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package='package1',
            executable='node1',
            name='node1_name'
        ),
        Node(
            package='package2',
            executable='node2',
            name='node2_name'
        )
    ]

    )