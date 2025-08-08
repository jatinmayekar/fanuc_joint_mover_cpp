from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_mover_cpp',
            executable='j2_mover_cpp',
            name='j2_mover_cpp',
            output='screen'
        ),
    ])