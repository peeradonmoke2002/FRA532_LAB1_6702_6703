#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "localization_ekf"

    # EKF node from robot_localization package (using inline parameters)


    # Local nodes from your package "localization_ekf"
    gps_node = Node(
        package="localization_ekf",
        executable="gps.py",  # Use the exact name from your pkg executables list
        name="gps_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )


    ekf_node = Node(
        package=package_name,
        executable="ekf-yawrate.py",
        name="ekf_node",
        output="screen"

    )

    ackerman_odom_node = Node(
        package=package_name,
        executable="ackerman_yaw_rate_odom.py",
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
