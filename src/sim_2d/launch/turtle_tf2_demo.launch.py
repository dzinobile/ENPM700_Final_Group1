from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    num_turtles = 9

    target_frame_args = [
        DeclareLaunchArgument(
            f"target_frame{i}",
            default_value="turtle1" if i !=1 else "turtle2",
            description=f"Target frame for turtle {i}"
        )
        for i in range(1, num_turtles+1)
    ]

    broadcaster_nodes = [
        Node(
            package='sim_2d',
            executable='turtle_tf2_broadcaster',
            name=f"broadcaster{i}",
            parameters=[{'turtlename': f"turtle{i}"}],

        )
        for i in range(1, num_turtles + 1)
    ]

    listener_params = {
        f"target_frame{i}": LaunchConfiguration(f"target_frame{i}")
        for i in range(1, num_turtles+1)
    }

    listener_node = Node(
        package='sim_2d',
        executable='turtle_tf2_listener',
        name='listener',
        parameters=[listener_params],
    )

    return LaunchDescription(
        [
            Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='sim'
            ),
            *target_frame_args,
            *broadcaster_nodes,
            listener_node
        ]
    )


