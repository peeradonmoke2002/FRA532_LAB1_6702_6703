import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "localization_ekf"

    # Declare a launch argument to choose which EKF configuration file to use
    ekf_config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='ekf.yaml',  # Default to ekf.yaml
        description='Select EKF configuration file: ekf.yaml, ekf-doubletrack.yaml, ekf-singletrack.yaml, ekf-yawrate.yaml'
    )

    # Correctly set up the parameter as a dictionary
    ekf_config_path = os.path.join(
        get_package_share_directory(package_name), "config"
    )

    # EKF Localization Node
    ekf_localization = Node(
        package=package_name,
        executable="ekf.py",
        name="ekf_localization",
        output="screen",
        parameters=[
            {'use_sim_time': True},
            {"ekf_config_file": LaunchConfiguration('config_file')},  # Pass as a ROS2 parameter
            {"ekf_config_path": ekf_config_path}  # Pass the folder path separately
        ]
    )

    # Create LaunchDescription
    launch_description = LaunchDescription([
        ekf_config_arg,  # Add the launch argument
        ekf_localization  # Add EKF node
    ])

    return launch_description
