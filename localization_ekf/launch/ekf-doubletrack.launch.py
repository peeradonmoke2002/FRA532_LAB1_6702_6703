#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "localization_ekf"

    # EKF node from robot_localization package (using inline parameters)
    ekf_localization = Node(
        package="localization_ekf",
        executable="ekf_node",
        name="ekf_localization",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Local nodes from your package "localization_ekf"
    gps_node = Node(
        package=package_name,
        executable="gps.py",
        name="gps_node",
        output="screen"
    )

    ekf_node = Node(
        package=package_name,
        executable="ekf.py",
        name="ekf_node",
        output="screen"
    )

    ackerman_odom_node = Node(
        package=package_name,
        executable="ekf-double.py",
        name="ackerman_odom_node",
        output="screen"
    )

    return LaunchDescription([
        ekf_localization,
        ekf_node,
        gps_node,
        ackerman_odom_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
