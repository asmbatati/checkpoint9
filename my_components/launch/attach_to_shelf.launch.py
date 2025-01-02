import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
      "use_sim_time",
      default_value="True"
    )

    attach_server_node = ExecuteProcess(
        cmd=["ros2", "run", "my_components", "manual_composition"],
        output="screen",
    )

    pre_approach_node = ComposableNode(
      package='my_components',
      plugin='my_components::PreApproach',
      name='pre_approach',
      parameters = [{"use_sim_time" : use_sim_time}]
    )
    # attach_server_node = ComposableNode(
    #   package='my_components',
    #   plugin='my_components::AttachServer',
    #   name='attach_server',
    #   parameters = [{"use_sim_time" : use_sim_time}]
    # )

    container = ComposableNodeContainer(
      name='my_container',
      namespace='',
      package='rclcpp_components',
      executable='component_container_mt',
      composable_node_descriptions=[
        pre_approach_node,
        # attach_server_node,
      ],
      output='screen',
      parameters = [{"use_sim_time" : use_sim_time}]
    )

    #files
    description_package_name = "my_components"
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

    return launch.LaunchDescription([
      use_sim_time_arg,
      attach_server_node,
      container,
      static_transform_elevator,
      static_transform_left_wheel,
      static_transform_right_wheel,
      rviz_node,
    ])