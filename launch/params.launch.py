from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # base_path = os.path.realpath(get_package_share_directory('package'))
    # rviz_path = base_path+'/config.config.rviz'

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

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d',str(rviz_path)]
    # )

    ld.add_action(virtual_bot_node)

    return ld