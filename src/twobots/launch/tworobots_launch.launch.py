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
    package_dir = get_package_share_directory('webots_ros2_turtlebot')
    my_pkg = get_package_share_directory('twobots')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    webots = WebotsLauncher(
        world=PathJoinSubstitution([my_pkg, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    robot_state_publisher1 = Node(
        namespace='robot1',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )
    # robot_state_publisher2 = Node(
    #     namespace='robot2',
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': '<robot name=""><link name=""/></robot>'
    #     }],
    # )

    footprint_publisher1 = Node(
        namespace='robot1',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    # footprint_publisher2 = Node(
    #     namespace='robot2',
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    # )

    # ROS control spawners
    controller_manager_timeout1 = ['--controller-manager-timeout', '50']
    # controller_manager_timeout2 = ['--controller-manager-timeout', '50']
    controller_manager_prefix1 = 'python.exe' if os.name == 'nt' else ''
    # controller_manager_prefix2 = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner1 = Node(
        namespace='robot1',
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix1,
        arguments=['diffdrive_controller'] + controller_manager_timeout1,
    )
    # diffdrive_controller_spawner2 = Node(
    #     namespace='robot2',
    #     package='controller_manager',
    #     executable='spawner',
    #     output='screen',
    #     prefix=controller_manager_prefix2,
    #     arguments=['diffdrive_controller'] + controller_manager_timeout2,
    # )
    joint_state_broadcaster_spawner1 = Node(
        namespace='robot1',
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix1,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout1,
    )
    # joint_state_broadcaster_spawner2 = Node(
    #     namespace='robot2',
    #     package='controller_manager',
    #     executable='spawner',
    #     output='screen',
    #     prefix=controller_manager_prefix2,
    #     arguments=['joint_state_broadcaster'] + controller_manager_timeout2,
    # )

    ros_control_spawners1 = [diffdrive_controller_spawner1, joint_state_broadcaster_spawner1]
    # ros_control_spawners2 = [diffdrive_controller_spawner2, joint_state_broadcaster_spawner2]

    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot_webots.urdf')
    ros2_control_params1 = os.path.join(my_pkg, 'resource', 'robot1_ros2control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    if use_twist_stamped:
        mappings1 = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings1 = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    turtlebot_driver1 = WebotsController(
        namespace='robot1',
        robot_name='robot1',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True,
             'update_rate': 50},
            ros2_control_params1
        ],
        remappings=mappings1,
        respawn=True
    )
    # turtlebot_driver2 = WebotsController(
    #     namespace='robot2',
    #     robot_name='robot2',
    #     parameters=[
    #         {'robot_description': robot_description_path,
    #          'use_sim_time': use_sim_time,
    #          'set_robot_state_publisher': True},
    #         ros2_control_params
    #     ],
    #     remappings=mappings,
    #     respawn=True
    # )

    # Navigation
    navigation_nodes1 = []
    # navigation_nodes2 = []
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    nav2_map = os.path.join(package_dir, 'resource', 'turtlebot3_burger_example_map.yaml')
    nav2_params = os.path.join(package_dir, 'resource', 'nav2_params.yaml')
    if 'turtlebot3_navigation2' in get_packages_with_prefixes():
        turtlebot_navigation1 = GroupAction([
            PushRosNamespace('robot1'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('turtlebot3_navigation2'),
                        'launch',
                        'navigation2.launch.py'
                    )
                ),
                launch_arguments=[
                    ('map', nav2_map),
                    ('params_file', nav2_params),
                    ('use_sim_time', use_sim_time),
                ],
                condition=launch.conditions.IfCondition(use_nav)
            )

        ])
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(
        #         get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')),
        #     launch_arguments=[
        #         ('map', nav2_map),
        #         ('params_file', nav2_params),
        #         ('use_sim_time', use_sim_time),
        #     ],
        #     condition=launch.conditions.IfCondition(use_nav))
        navigation_nodes1.append(turtlebot_navigation1)

        # turtlebot_navigation2 = GroupAction([
        #     PushRosNamespace('robot2'),
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(
        #             os.path.join(
        #                 get_package_share_directory('turtlebot3_navigation2'),
        #                 'launch',
        #                 'navigation2.launch.py'
        #             )
        #         ),
        #         launch_arguments=[
        #             ('map', nav2_map),
        #             ('params_file', nav2_params),
        #             ('use_sim_time', use_sim_time),
        #         ],
        #         condition=launch.conditions.IfCondition(use_nav)
        #     )

        # ])
        # navigation_nodes2.append(turtlebot_navigation2)

    # SLAM
    if 'turtlebot3_cartographer' in get_packages_with_prefixes():
        turtlebot_slam1 = GroupAction([
            PushRosNamespace('robot1'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')),
                launch_arguments=[
                    ('use_sim_time', use_sim_time),
                ],
                condition=launch.conditions.IfCondition(use_slam))

        ])

        navigation_nodes1.append(turtlebot_slam1)

        # turtlebot_slam2 = GroupAction([
        #     PushRosNamespace('robot2'),
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(os.path.join(
        #             get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')),
        #         launch_arguments=[
        #             ('use_sim_time', use_sim_time),
        #         ],
        #         condition=launch.conditions.IfCondition(use_slam))

        # ])
        # navigation_nodes2.append(turtlebot_slam2)

    # Wait for the simulation to be ready to start navigation nodes
    waiting_nodes1 = WaitForControllerConnection(
        target_driver=turtlebot_driver1,
        nodes_to_start=navigation_nodes1 + ros_control_spawners1
    )
    # waiting_nodes2 = WaitForControllerConnection(
    #     target_driver=turtlebot_driver2,
    #     nodes_to_start=navigation_nodes2 + ros_control_spawners2
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='onebot_world.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,

        robot_state_publisher1,
        # robot_state_publisher2,
        footprint_publisher1,
        # footprint_publisher2,

        turtlebot_driver1,
        # turtlebot_driver2,
        waiting_nodes1,
        # waiting_nodes2,

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