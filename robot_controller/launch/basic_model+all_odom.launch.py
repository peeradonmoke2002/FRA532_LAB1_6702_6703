from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = "robot_controller"

    ackerman_controller_basic_model = Node(
        package=package_name,
        executable="ackerman_controller_basic_model.py",
        name="ackerman_controller_basic_model",
    )

    ackerman_yaw_rate_odom = Node(
        package=package_name,
        executable="ackerman_yaw_rate_odom.py",
        name="ackerman_yaw_rate_odom",
    )

    ackerman_odom_single_track = Node(
        package=package_name,
        executable="ackerman_odom_single_track.py",
        name="ackerman_odom_single_track",
    )

    ackerman_odom_double_track = Node(
        package=package_name,
        executable="ackerman_odom_double_track.py",
        name="ackerman_odom_double_track",
    )

    launch_description = LaunchDescription()

    launch_description.add_action(ackerman_controller_basic_model)
    launch_description.add_action(ackerman_yaw_rate_odom)
    launch_description.add_action(ackerman_odom_single_track)
    launch_description.add_action(ackerman_odom_double_track)

    return launch_description


