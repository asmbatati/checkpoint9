import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #files
    description_package_name = "attach_shelf"
    rviz_file = 'attach_shelf.rviz'

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
        static_transform_elevator,
        static_transform_left_wheel,
        static_transform_right_wheel,
        rviz_node,
    ])