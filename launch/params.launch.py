from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    virtual_bot_node = Node(
        package="cpp_topics",
        executable="new",
        parameters=[
            {"tf_prefix": "r1"},
            {"x_pos":1.8},
            {"y_pos":-0.7},
            {"a_pos":1.56}
        ]
    )

    virtual_bot_node_2 = Node(
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
    ld.add_action(virtual_bot_node_2)

    return ld