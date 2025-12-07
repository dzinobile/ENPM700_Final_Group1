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

"""Launch multiple Webots TurtleBot3 Burger robots."""

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_turtlebot')
    my_dir = get_package_share_directory('sheepdog')
    
    # Configuration
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    number_of_robots = LaunchConfiguration('num_robots', default=4)
    
    # Robot configuration
    namespace = 'TB3'
    
    # Launch Webots
    webots = WebotsLauncher(
        world=PathJoinSubstitution([my_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    # Determine twist message type based on ROS distro
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    
    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
    
    # Create lists to hold robot nodes
    robot_groups = []
    all_drivers = []
    all_waiting_nodes = []
    
    # Number of robots (you can make this configurable)
    num_robots = 4
    
    for i in range(num_robots):
        robot_namespace = f'{namespace}_{i+1}'
        robot_name = f'TurtleBot3Burger_{i+1}'
        
        # Robot state publisher for each robot
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=robot_namespace,
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>',
                'frame_prefix': f'{robot_namespace}/'
            }],
        )
        
        # Footprint publisher for each robot
        footprint_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            namespace=robot_namespace,
            arguments=['0', '0', '0', '0', '0', '0', 
                      f'{robot_namespace}/base_link', 
                      f'{robot_namespace}/base_footprint'],
        )
        
        # ROS control spawners for each robot
        controller_manager_timeout = ['--controller-manager-timeout', '50']
        controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
        
        diffdrive_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            prefix=controller_manager_prefix,
            namespace=robot_namespace,
            arguments=['diffdrive_controller',
                       '-c', f'/{robot_namespace}/controller_manager'] + controller_manager_timeout,
        )
        
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            prefix=controller_manager_prefix,
            namespace=robot_namespace,
            arguments=['joint_state_broadcaster',
                       '-c', f'/{robot_namespace}/controller_manager'] + controller_manager_timeout,
        )
        
        ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]
        
        # Set up remappings for each robot
        if use_twist_stamped:
            mappings = [
                (f'/{robot_namespace}/diffdrive_controller/cmd_vel', f'/{robot_namespace}/cmd_vel'),
                (f'/{robot_namespace}/diffdrive_controller/odom', f'/{robot_namespace}/odom')
            ]
        else:
            mappings = [
                (f'/{robot_namespace}/diffdrive_controller/cmd_vel_unstamped', f'/{robot_namespace}/cmd_vel'),
                (f'/{robot_namespace}/diffdrive_controller/odom', f'/{robot_namespace}/odom')
            ]
        
        # Webots controller for each robot
        turtlebot_driver = WebotsController(
            robot_name=robot_name,
            parameters=[
                {'robot_description': robot_description_path,
                 'use_sim_time': use_sim_time,
                 'set_robot_state_publisher': True,
                 'update_rate': 50},
                ros2_control_params
            ],
            remappings=mappings,
            respawn=True,
            namespace=robot_namespace
        )
        
        all_drivers.append(turtlebot_driver)
        
        # Navigation nodes for each robot (optional)
        navigation_nodes = []
        
        if 'turtlebot3_navigation2' in get_packages_with_prefixes():
            os.environ['TURTLEBOT3_MODEL'] = 'burger'
            nav2_map = os.path.join(my_dir, 'resource', 'my_world.yaml')
            nav2_params = os.path.join(package_dir, 'resource', 'nav2_params.yaml')
            
            turtlebot_navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('turtlebot3_navigation2'), 
                    'launch', 
                    'navigation2.launch.py')),
                launch_arguments=[
                    ('map', nav2_map),
                    ('params_file', nav2_params),
                    ('use_sim_time', use_sim_time),
                    ('namespace', robot_namespace),
                ],
                condition=launch.conditions.IfCondition(use_nav))
            navigation_nodes.append(turtlebot_navigation)
        
        # SLAM for each robot (optional)
        if 'turtlebot3_cartographer' in get_packages_with_prefixes():
            turtlebot_slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('turtlebot3_cartographer'), 
                    'launch', 
                    'cartographer.launch.py')),
                launch_arguments=[
                    ('use_sim_time', use_sim_time),
                    ('namespace', robot_namespace),
                ],
                condition=launch.conditions.IfCondition(use_slam))
            navigation_nodes.append(turtlebot_slam)
        
        # Wait for controller connection
        waiting_nodes = WaitForControllerConnection(
            target_driver=turtlebot_driver,
            nodes_to_start=navigation_nodes + ros_control_spawners
        )
        
        all_waiting_nodes.append(waiting_nodes)
        
        # Group all nodes for this robot
        robot_group = GroupAction([
            PushRosNamespace(robot_namespace),
            robot_state_publisher,
            footprint_publisher,
            turtlebot_driver,
            waiting_nodes,
        ])
        
        robot_groups.append(robot_group)
    
    # Build launch description
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='multi_world.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        DeclareLaunchArgument(
            'nav',
            default_value='false',
            description='Launch with navigation'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Launch with SLAM'
        ),
        DeclareLaunchArgument(
            'num_robots',
            default_value='4',
            description='Number of robots to spawn'
        ),
        webots,
        webots._supervisor,
    ])
    
    # Add all robot groups
    for robot_group in robot_groups:
        ld.add_action(robot_group)
    
    # Kill all nodes when Webots exits
    ld.add_action(
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    )
    
    return ld