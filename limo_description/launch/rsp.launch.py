import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name = 'limo_description'
    urdf_file_path = 'urdf/limo_ackerman.xacro'

    # Process the URDF file
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    pkg_path = os.path.join(get_package_share_directory('limo_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'limo_ackerman.xacro')
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    urdf_model = LaunchConfiguration('urdf_model')

    # Ensure Xacro file processes correctly
    robot_description_config = xacro.process_file(xacro_file, mappings={"use_sim_time": "true"})

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')

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
        parameters=[{'robot_description': Command(['xacro ', urdf_model]),'use_sim_time': use_sim_time}]
    )

    # Add joint_state_publisher (required for visualization)
    node_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        declare_urdf_model_path_cmd,
        node_robot_state_publisher,
        # node_joint_state_publisher
    ])
