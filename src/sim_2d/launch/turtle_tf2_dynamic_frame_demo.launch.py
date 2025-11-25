from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('sim_2d'), 'launch', 'turtle_tf2_demo.launch.py']),
            launch_arguments={'target_frame2': 'carrot1', 'target_frame3': 'carrot2'}.items(),
        ),
        Node(
            package='sim_2d',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_broadcaster',
        ),
    ])