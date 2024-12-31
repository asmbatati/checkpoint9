import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #files
    description_package_name = "attach_shelf"
    rviz_file = 'attach_shelf.rviz'

    # Declare launch arguments
    obstacle_arg = DeclareLaunchArgument(
        'obstacle', default_value='0.0', description='Distance to obstacle'
    )
    degrees_arg = DeclareLaunchArgument(
        'degrees', default_value='-90', description='Rotation degrees'
    )
    final_approach_arg = DeclareLaunchArgument(
        'final_approach', default_value='false', description='Final approach flag'
    )

    # Arguments: All the arguments have to be strings. Floats will give an error of NonItreable.
    launch.actions.DeclareLaunchArgument('obstacle', default_value='0.0'),
    launch.actions.DeclareLaunchArgument('degrees', default_value='-90.0'),
    launch.actions.DeclareLaunchArgument('final_approach', default_value='true'),
    launch.actions.LogInfo(
        msg=launch.substitutions.LaunchConfiguration('obstacle')),
    launch.actions.LogInfo(
        msg=launch.substitutions.LaunchConfiguration('degrees')),
    launch.actions.LogInfo(
        msg=launch.substitutions.LaunchConfiguration('final_approach')),

    pre_approach_v2_node = Node(
        package=description_package_name,
        executable='pre_approach_v2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'obstacle': launch.substitutions.LaunchConfiguration('obstacle'),
            'degrees': launch.substitutions.LaunchConfiguration('degrees'),
            'final_approach': launch.substitutions.LaunchConfiguration('final_approach'),
        }]
    )

    approach_service_server_node = Node(
        package=description_package_name,
        executable='approach_service_server_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'obstacle': launch.substitutions.LaunchConfiguration('obstacle')
        }]
    )

    # Static transforms for missing frames
    static_transform_elevator = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0", "0", "0", "0", "1",  # Transform from parent to child
            "robot_evelator_base_link", "robot_evelator_platform_link"
        ],
        name="static_transform_elevator"
    )

    static_transform_left_wheel = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0", "0", "0", "0", "1",  # Transform from parent to child
            "robot_base_link", "robot_left_wheel_link"
        ],
        name="static_transform_left_wheel"
    )

    static_transform_right_wheel = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0", "0", "0", "0", "1",  # Transform from parent to child
            "robot_base_link", "robot_right_wheel_link"
        ],
        name="static_transform_right_wheel"
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(description_package_name), 'rviz', rviz_file)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        pre_approach_v2_node,
        approach_service_server_node,
        static_transform_elevator,
        static_transform_left_wheel,
        static_transform_right_wheel,
        rviz_node,
    ])