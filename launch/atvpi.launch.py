from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='atvpi',
            namespace='sub_node',
            executable='sub',
            name='sub_node'
        ),
        Node(
            package='atvpi',
            namespace='gamepad_websocket_node',
            executable='gamepad_websocket_node',
            name='gamepad_websocket_node'
        ),
    ])