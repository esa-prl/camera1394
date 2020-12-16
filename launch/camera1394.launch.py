import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('camera1394'),
         'config',
         'marta.yaml'
    )
    
    # Launch declarations
    return LaunchDescription([
        Node(
            package="camera1394",
            executable="camera1394_node",
            name="camera1394_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                config
            ],
        )
    ])
