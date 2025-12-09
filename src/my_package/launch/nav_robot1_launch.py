from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("my_package")
    nav2_params = os.path.join(pkg_share, "config", "nav2_robot1_params.yaml")

    # Nav2 lifecycle manager (for Nav2 stack, not slam)
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation_robot1",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "smoother_server",
                ]
            },
        ],
    )

    # Controller server
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
        remappings=[    
            ("/cmd_vel", "/robot1/cmd_vel"),    # if needed
            ("/odom", "/robot1/odom"),
        ],
    )

    # Planner server
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )

    # Behavior tree navigator
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params],
    )

    # Smoother
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav2_params],
    )

    # Behavior server (recovery behaviors)
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params],
    )

    # Waypoint follower (optional but harmless)
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params],
    )

    return LaunchDescription(
        [
            controller_server,
            planner_server,
            bt_navigator,
            smoother_server,
            behavior_server,
            waypoint_follower,
            lifecycle_manager,
        ]
    )
