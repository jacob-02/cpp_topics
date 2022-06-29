from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    virtual_bot_node = Node(
        package="cpp_topics",
        executable="new",
        parameters=[
            {"tf_prefix": "r1"},
            {"x_pos": 0.0},
            {"y_pos": 0.0},
            {"a_pos": 0.0}
        ]
    )

    qr_node = Node(
        package="cpp_topics",
        executable="qr_code",
        parameters=[
            {"tf_prefix": "r1"}
        ]
    )

    square_node = Node(
        package="cpp_topics",
        executable="square",
        parameters=[
            {"tf_prefix": "r1"}
        ]
    )

    goal_node = Node(
        package="cpp_topics",
        executable="goal",
        parameters=[
            {"tf_prefix": "r1"},
            {"x_goal": 10},
            {"y_goal": 10}
        ]
    )

    autotune_node = Node(
        package="cpp_topics",
        executable="autotune",
        parameters=[
            {"tf_prefix": "r1"}
        ]
    )
    # rotate_node = Node(
    #     package="cpp_topics",
    #     executable="rotate",
    #     parameters=[
    #         {"tf_prefix": "r1"}
    #     ]
    # )

    ld.add_action(virtual_bot_node)
    # ld.add_action(rotate_node)
    ld.add_action(goal_node)
    # ld.add_action(square_node)
    ld.add_action(qr_node)
    ld.add_action(autotune_node)

    return ld
