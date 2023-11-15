from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
import socket


def generate_launch_description():
    tool_frame_name = "probe"
    base_frame_name = "base_link"
    rate = 1000
    topic = '/tool0_pose'
    return LaunchDescription(
        [
            Node(
                package="tf_publisher",
                executable="tf_publisher",
                parameters=[
                    {"ee_frame_name": tool_frame_name, "base_frame_name": base_frame_name, 'rate': str(rate), 'topic':topic}
                ],
            )
        ]
    )
