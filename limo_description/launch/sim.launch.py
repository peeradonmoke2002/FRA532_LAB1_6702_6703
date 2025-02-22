import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    package_name = "limo_description"
    rviz_file_name = "gazebo.rviz"
    world_file = "basic.world"
    #Pose: x=-4.711, y=9.611, yaw=153.92° 

    #deafult
    spawn_x_val = "9.073500"
    spawn_y_val = "0.0"
    spawn_z_val = "0.0"
    spawn_yaw_val = "1.57"

    #curve1
    # spawn_x_val = "-4.633837776338229"
    # spawn_y_val = "9.687033850999052"
    # spawn_z_val = "0.0"
    # spawn_yaw_val = "2.6799560258356285"

    #curve2
    # spawn_x_val = "-9.625820"
    # spawn_y_val = "0.152707"
    # spawn_z_val = "0.0"
    # spawn_yaw_val = "-1.558429"

    # Paths
    rviz_file_path = os.path.join(get_package_share_directory(package_name), "rviz", rviz_file_name)
    world_path = os.path.join(get_package_share_directory(package_name), "worlds", world_file)
    # Path to EKF configuration file
    ekf_config_path = os.path.join(get_package_share_directory(package_name), "config", "ekf.yaml")
    # Include Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items()
    )

    # Launch Gazebo with a specific world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path}.items()
    )

    # Spawn the robot at a specific location
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "limo",
            "-x", spawn_x_val,
            "-y", spawn_y_val,
            "-z", spawn_z_val,
            "-Y", spawn_yaw_val
        ],
        output="screen"
    )




    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_position_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": False}]
    )

    # slam_toolbox_node = Node(
    #         package='slam_toolbox',
    #         executable='sync_slam_toolbox_node',  # Use the correct executable
    #         name='slam_toolbox',
    #         output='screen',
    #         parameters=[{'use_sim_time': True}],
    #         remappings=[('/scan', '/laser/scan')]  # Remap laser scan topic if needed
    #     )



    # Start RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file_path],
        output="screen"
    )

    launch_description = LaunchDescription()

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[velocity_controller_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=velocity_controller_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        )
    )
        

    ekf_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_localization",
        output="screen",
        parameters=[{'use_sim_time': True}, ekf_config_path]
    )




    # Add launch actions
    launch_description.add_action(rviz)
    launch_description.add_action(gazebo)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(rsp)
    # launch_description.add_action(slam_toolbox_node)
    launch_description.add_action(ekf_localization)


    return launch_description
