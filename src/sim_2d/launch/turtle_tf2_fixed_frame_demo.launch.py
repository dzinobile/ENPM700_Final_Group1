from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    num_turtles = 9
    carrot_args = {
        f"target_frame{i}": f"carrot{i}"
        for i in range(1, num_turtles + 1)
    }


    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('sim_2d'),'launch', 'turtle_tf2_demo.launch.py'
            ]),
            launch_arguments=carrot_args.items(),
        ),
        Node(
            package='sim_2d',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_broadcaster',
            
        ),
    ])