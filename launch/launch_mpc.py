import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('rust_mpc'),
        'config',
        'sim.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))

    map_name = config_dict['rust_mpc']['ros__parameters']['map'] + "_map"
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config, {'map_path': os.path.join(get_package_share_directory('rust_mpc'), 'data', map_name)}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('rust_mpc'), 'launch', 'mpc.rviz')]
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[
            {'yaml_filename': os.path.join(get_package_share_directory('rust_mpc'), 'data', map_name + '.yaml')},
            {'topic': 'map'},
            {'frame_id': 'map'},
            {'output': 'screen'},
            {'use_sim_time': True}]
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', os.path.join(get_package_share_directory('rust_mpc'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )

    mpc_node = Node(
        package='rust_mpc',
        executable='mpc_node',
        name='mpc_node',
        output='screen'
    )

    # Timers needed because map sometimes is not loading
    ld.add_action(rviz_node)
    ld.add_action(TimerAction(
        period=0.,
        actions=[
            bridge_node,
            map_server_node

        ]
    ),
    )
    ld.add_action(TimerAction(
        period=2.,
        actions=[
            nav_lifecycle_node,
            ego_robot_publisher,
            mpc_node
        ]
    ),
    )

    return ld
