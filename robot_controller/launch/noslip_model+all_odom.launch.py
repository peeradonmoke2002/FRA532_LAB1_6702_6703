from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = "robot_controller"

    ackerman_controller_no_slip = Node(
        package=package_name,
        executable="ackerman_controller_no_slip.py",
        name="ackerman_controller_no_slip",
        output="screen"
    )

    ackerman_yaw_rate_odom = Node(
        package=package_name,
        executable="ackerman_yaw_rate_odom.py",
        name="ackerman_yaw_rate_odom",
        output="screen"
    )

    ackerman_odom_single_track = Node(
        package=package_name,
        executable="ackerman_odom_single_track.py",
        name="ackerman_odom_single_track",
        output="screen"
    )

    ackerman_odom_double_track = Node(
        package=package_name,
        executable="ackerman_odom_double_track.py",
        name="ackerman_odom_double_track",
        output="screen"
    )

    launch_description = LaunchDescription()

    launch_description.add_action(ackerman_controller_no_slip)
    launch_description.add_action(ackerman_yaw_rate_odom)
    launch_description.add_action(ackerman_odom_single_track)
    launch_description.add_action(ackerman_odom_double_track)

    return launch_description


