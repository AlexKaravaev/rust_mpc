from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()


    # mpc_node = Node(
    #     package='rust_mpc',
    #     executable='load_raceline.py',
    #     name='load_raceline',
    #     output='screen'
    # )
    mpc_node = Node(
        package='rust_mpc',
        executable='mpc_node',
        name='mpc_node',
        output='screen'
    )

    # finalize
    ld.add_action(mpc_node)
    # ld.add_action(raceline_node)

    return ld