from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value="0.30"
    )
    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="-90.0"
    )
    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value="true"
    )

    obstacle_f = LaunchConfiguration('obstacle')
    degrees_f = LaunchConfiguration('degrees')
    final_approach_f = LaunchConfiguration('final_approach')

    pre_approach_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2_node',
        output='screen',
        name='pre_approach_v2_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        arguments=["-obstacle", obstacle_f,
                   "-degrees", degrees_f,
                   "-final_approach", final_approach_f,]
    )

    # create and return launch description object
    return LaunchDescription(
        [
            obstacle_arg,
            degrees_arg,
            final_approach_arg,
            pre_approach_node
        ]
    )