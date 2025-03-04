import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    package_name = "localization_ekf"

    config_folder = os.path.join(get_package_share_directory(package_name), "config")

    ekf_config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='ekf-singletrack.yaml',
        description='Select EKF configuration file'
    )

    ekf_config_path = PathJoinSubstitution([config_folder, LaunchConfiguration('config_file')])

    ekf_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_localization",
        output="screen",
        parameters=[{"use_sim_time": True, "ekf_config_path": ekf_config_path}]
    )

    yawrate_script_path = os.path.join(get_package_share_directory(package_name), "scripts/double_track")

    gps_node = Node(
        package="localization_ekf",
        executable="gps.py",
        name="gps_node",
        output="screen"
    )

    ekf_node = Node(
        package="localization_ekf",
        executable="odom_filtered_doubletrack.py",
        name="ekf_node",
        output="screen"
    )

    ackerman_odom_node = Node(
        package="localization_ekf",
        executable="ackerman_odom_double_track.py",
        name="ackerman_odom_node",
        output="screen"
    )



    return LaunchDescription([
        ekf_config_arg,
        ekf_localization,
        ekf_node,
        gps_node,
        ackerman_odom_node,
    ])
