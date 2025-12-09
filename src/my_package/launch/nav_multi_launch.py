from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def make_nav2_nodes(ns: str, map_topic: str, odom_frame: str, base_frame: str, scan_topic: str):
    pkg_share = get_package_share_directory("my_package")
    params_file = os.path.join(pkg_share, "config", "nav2_multi_params.yaml")

    remaps = [
        ("map", map_topic),
        ("map_updates", f"{map_topic}_updates"),
        ("odom", odom_frame),
        ("scan", scan_topic),
        ("cmd_vel", f"/{ns}/cmd_vel"),
    ]

    return [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            namespace=ns,
            parameters=[params_file],
            remappings=remaps,
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            namespace=ns,
            parameters=[params_file],
            remappings=remaps,
            output="screen",
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            namespace=ns,
            parameters=[params_file],
            remappings=remaps,
            output="screen",
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            namespace=ns,
            parameters=[params_file],
            remappings=remaps,
            output="screen",
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            namespace=ns,
            parameters=[params_file],
            remappings=remaps,
            output="screen",
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            namespace=ns,
            parameters=[params_file],
            remappings=remaps,
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            namespace=ns,
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": [
                        "controller_server",
                        "planner_server",
                        "bt_navigator",
                        "smoother_server",
                        "behavior_server",
                        "waypoint_follower",
                        "global_costmap/global_costmap",
                        "local_costmap/local_costmap",
                    ],
                }
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    robot1_nodes = make_nav2_nodes(
        ns="robot1",
        map_topic="/robot1/map",
        odom_frame="robot1/odom",
        base_frame="robot1/base_link",
        scan_topic="/robot1/scan",
    )

    robot2_nodes = make_nav2_nodes(
        ns="robot2",
        map_topic="/robot2/map",
        odom_frame="robot2/odom",
        base_frame="robot2/base_link",
        scan_topic="/robot2/scan",
    )

    return LaunchDescription(robot1_nodes + robot2_nodes)
