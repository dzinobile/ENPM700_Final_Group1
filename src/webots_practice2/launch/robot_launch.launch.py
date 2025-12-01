#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots TurtleBot3 Burger driver."""


import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    num_bots = 1

    package_dir = get_package_share_directory('webots_ros2_turtlebot')
    my_dir = get_package_share_directory('webots_practice2')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([my_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    robot_state_publishers = []
    footprint_publishers = []
    turtlebot_drivers = []
    waiting_nodes = []
    
    for i in range(0, num_bots+1):

        robot_state_publisher = Node(
            namespace=f'robot{i}',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>'
            }],
        )
        robot_state_publishers.append(robot_state_publisher)

        footprint_publisher = Node(
            namespace=f'robot{i}',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        )
        footprint_publishers.append(footprint_publisher)

        # ROS control spawners
        controller_manager_timeout = ['--controller-manager-timeout', '50']
        controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
        diffdrive_controller_spawner = Node(
            namespace=f'robot{i}',
            package='controller_manager',
            executable='spawner',
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['diffdrive_controller'] + controller_manager_timeout,
        )
        joint_state_broadcaster_spawner = Node(
            namespace=f'robot{i}',
            package='controller_manager',
            executable='spawner',
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        )
        ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

        robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')
        ros2_control_params = os.path.join(my_dir, 'resource/ros2control', f'robot{i}_ros2control.yml')
        # ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
        use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
        if use_twist_stamped:
            mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
        else:
            mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
        turtlebot_driver = WebotsController(
            namespace=f'robot{i}',
            robot_name=f'robot{i}',
            parameters=[
                {'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True,
                'tf_prefix': f'robot{i}',
                'publish_sensors_tf': True,
                'update_rate': 50},
                ros2_control_params
            ],
            remappings=mappings,
            respawn=True
        )
        turtlebot_drivers.append(turtlebot_driver)

        # Navigation
        navigation_node = []
        os.environ['TURTLEBOT3_MODEL'] = 'burger'
        nav2_map = os.path.join(package_dir, 'resource', 'turtlebot3_burger_example_map.yaml')
        nav2_params = os.path.join(package_dir, 'resource', 'nav2_params.yaml')
        if 'turtlebot3_navigation2' in get_packages_with_prefixes():
            turtlebot_navigation = GroupAction([
                PushRosNamespace(f'robot{i}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')),
                    launch_arguments=[
                        ('map', nav2_map),
                        ('params_file', nav2_params),
                        ('use_sim_time', use_sim_time),
                    ],
                    condition=launch.conditions.IfCondition(use_nav)
                    )
                

            ])
            navigation_node.append(turtlebot_navigation)
            
            
            
            


        # SLAM
        if 'turtlebot3_cartographer' in get_packages_with_prefixes():
            turtlebot_slam = GroupAction([
                PushRosNamespace(f'robot{i}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')),
                    launch_arguments=[
                        ('use_sim_time', use_sim_time),
                    ],
                    condition=launch.conditions.IfCondition(use_slam))

            ])

            navigation_node.append(turtlebot_slam)

        # Wait for the simulation to be ready to start navigation nodes
        waiting_node = WaitForControllerConnection(
            target_driver=turtlebot_driver,
            nodes_to_start=navigation_node + ros_control_spawners
        )
        waiting_nodes.append(waiting_node)

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot3_burger_example.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,

        *robot_state_publishers,
        *footprint_publishers,

        *turtlebot_drivers,
        *waiting_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ])