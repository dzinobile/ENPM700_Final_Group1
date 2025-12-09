from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("my_package")

    slam1_config = os.path.join(pkg_share, "config", "slam_robot1.yaml")
    slam2_config = os.path.join(pkg_share, "config", "slam_robot2.yaml")
    # slam3_config = os.path.join(pkg_share, "config", "slam_robot3.yaml")

    lifecycle_nodes = []

    # --- Robot 1 SLAM node ---
    slam1_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox_robot1",
        output="screen",
        parameters=[slam1_config],
        remappings=[
            # "scan" param (in yaml) -> robot1 lidar topic
            ("scan", "/robot1/robot1/lidar1"),
            # per-robot map topics
            ("/map", "/robot1/map"),
            ("/map_metadata", "/robot1_map_metadata"),
        ],
    )
    # lifecycle_nodes.append("slam_toolbox_robot1")

    # Lifecycle manager for robot1's slam
    lifecycle_manager1 = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam_robot1",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"bond_timeout": 10.0},              # give it more time than 4 s
            {"bond_connection_timeout": 10.0},
            {"node_names": ["slam_toolbox_robot1"]},
        ],
    )

    # --- Robot 2 SLAM node ---
    slam2_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox_robot2",
        output="screen",
        parameters=[slam2_config],
        remappings=[
            ("scan", "/robot2/robot2/lidar1"),
            ("/map", "/robot2/map"),
            ("/map_metadata", "/robot2/map_metadata"),
        ],
    )

    # Lifecycle manager for robot2's slam
    lifecycle_manager2 = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam_robot2",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"bond_timeout": 10.0},
            {"bond_connection_timeout": 10.0},
            {"node_names": ["slam_toolbox_robot2"]},
        ],
    )

    # lifecycle_manager = Node(
    #     package="nav2_lifecycle_manager",
    #     executable="lifecycle_manager",
    #     name="lifecycle_manager_slam",
    #     output="screen",
    #     parameters=[
    #         {"use_sim_time": False},
    #         {"autostart": True},
    #         {"node_names": lifecycle_nodes},
    #     ],
    # )

    return LaunchDescription([
        slam1_node,
        slam2_node,
        # slam3_node,
        lifecycle_manager1,
        lifecycle_manager2,
    ])



#================================================
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():
#     pkg_share = get_package_share_directory("my_package")
#     slam_config = os.path.join(pkg_share, "config", "slam_robot1.yaml")

#     slam_node = Node(
#         package="slam_toolbox",
#         executable="sync_slam_toolbox_node",
#         name="slam_toolbox",
#         parameters=[slam_config],
#         remappings=[
#         ("/map", "/map_robot1"),
#         ],
#         # remappings=[
#         #     # Only needed if you change topic names later, but kept here for clarity
#         #     ("robot1/lidar1", "/robot1/scan"),
#         # ],
#     )   

#     return LaunchDescription([slam_node])
