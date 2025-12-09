from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_name = "my_package"
    package_share = get_package_share_directory(package_name)

    world_path = PathJoinSubstitution(
        [package_share, "worlds", "my_world_new.wbt"]  # now containing 3 robots
    )

    urdf_path = PathJoinSubstitution(
        [package_share, "resource", "my_robot.urdf"]
    )

    # Start Webots with the modified world
    webots = WebotsLauncher(
        world=world_path,
        # You can add gui=False here later if you want headless
    )

    # Common parameters: all robots use the same URDF (same plugin)
    driver_params = [{"robot_description": urdf_path}]


    robot1_driver = WebotsController(
        robot_name="robot1",
        parameters=driver_params,
    )

    robot2_driver = WebotsController(
        robot_name="robot2",
        parameters=driver_params,
    )

    robot3_driver = WebotsController(
        robot_name="robot3",
        parameters=driver_params,
    )






    # One controller per robot, each with its own cmd_vel
    # robot1_driver = WebotsController(
    #     robot_name="robot1",
    #     parameters=driver_params,
    #     remappings=[
    #         ("/cmd_vel", "/robot1/cmd_vel"),
    #     ],
    # )

    # robot2_driver = WebotsController(
    #     robot_name="robot2",
    #     parameters=driver_params,
    #     remappings=[
    #         ("/cmd_vel", "/robot2/cmd_vel"),
    #     ],
    # )

    # robot3_driver = WebotsController(
    #     robot_name="robot3",
    #     parameters=driver_params,
    #     remappings=[
    #         ("/cmd_vel", "/robot3/cmd_vel"),
    #     ],
    # )

    # Shut down ROS when Webots is closed
    shutdown_on_webots_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=webots,
            on_exit=[Shutdown()],
        )
    )

    return LaunchDescription(
        [
            webots,
            robot1_driver,
            robot2_driver,
            robot3_driver,
            shutdown_on_webots_exit,
        ]
    )
