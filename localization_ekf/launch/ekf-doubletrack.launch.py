#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "localization_ekf"

    # Local nodes from your package "localization_ekf"
    gps_node = Node(
        package=package_name,
        executable="gps.py",
        name="gps_node",
        output="screen"
    )

    # Use the EKF node from your single_track folder (ensure the file is installed as ekf-single.py)
    ekf_node = Node(
        package=package_name,
        executable="ekf-double.py",  # This must match the installed file name!
        name="ekf_node",
        output="screen"
    )

    ackerman_odom_node = Node(
        package=package_name,
        executable="ackerman_odom_double_track.py",
        name="ackerman_odom_node",
        output="screen"
    )

    return LaunchDescription([
        gps_node,
        ekf_node,
        ackerman_odom_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
