from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera1394",
            executable="camera1394_node",
            name="camera1394_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"guid": "0"}
            ]
        )
    ])
