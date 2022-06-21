from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    virtual_bot_node = Node(
        package="cpp_topics",
        executable="new",
        parameters=[
            {"tf_prefix": "r2"},
            {"x_pos":1.8},
            {"y_pos":0.7},
            {"a_pos":1.56}
        ]
    )

    ld.add_action(virtual_bot_node)

    return ld