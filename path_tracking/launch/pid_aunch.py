#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory for your package (e.g., "path_tracking")
    package_name = "path_tracking"

    # Declare a launch argument called "mode"
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='noslip',
        description='Pure pursuit mode: basic or noslip'
    )

    # Use the LaunchConfiguration to pass the "mode" parameter to the node.
    pid_node = Node(
        package=package_name,
        executable='pid.py',  # Ensure this matches your executable name
        name='pid_node',
        output='screen',
        parameters=[{'mode': LaunchConfiguration('mode')}]
    )

    path_data_record = Node(
        package=package_name,
        executable='path_data_record.py',  # Ensure this matches your executable name
        name='path_data_record_node',
        output='screen'
    )
    launch_description = LaunchDescription()
    launch_description.add_action(pid_node)
    launch_description.add_action(path_data_record)


    return launch_description
