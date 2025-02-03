import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    # package_name = "limo_description"
    # rviz_file_name = "limo.rviz"
    # rviz_file_path = os.path.join(
    #     get_package_share_directory(package_name),
    #     'rviz',
    #     rviz_file_name
    # )s

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('limo_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'limo_ackerman.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Define robot description parameter
    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    # Start robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Add joint_state_publisher
    node_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    # # Start RViz
    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", rviz_file_path],
    #     output="screen"
    # )

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        node_robot_state_publisher,
        node_joint_state_publisher,  
        # rviz
    ])
