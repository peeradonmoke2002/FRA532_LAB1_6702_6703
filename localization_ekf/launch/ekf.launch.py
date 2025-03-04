import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    package_name = "localization_ekf"

    # Path to EKF configuration file
    ekf_config_path = os.path.join(get_package_share_directory(package_name), "config", "ekf.yaml")


    ekf_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_localization",
        output="screen",
        parameters=[{'use_sim_time': True}, ekf_config_path]
    )




    # Add launch actions
    # Create LaunchDescription
    launch_description = LaunchDescription()
    
    launch_description.add_action(ekf_localization)


    return launch_description