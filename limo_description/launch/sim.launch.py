import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    package_name = "limo_description"
    rviz_file_name = "gazebo.rviz"
    world_file = "basic.world"

    spawn_x_val = "9.073500"
    spawn_y_val = "0.0"
    spawn_z_val = "0.0"
    spawn_yaw_val = "1.57"

    # Paths
    rviz_file_path = os.path.join(get_package_share_directory(package_name), "rviz", rviz_file_name)
    world_path = os.path.join(get_package_share_directory(package_name), "worlds", world_file)

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

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )

    joint_trajectory_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_position_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )

    steering_monitor = Node(
        package="robot_controller",
        executable="steering_monitor.py"
    )

    ackerman_controller = Node(
        package="robot_controller",
        executable="ackerman_controller.py"
    )

    # Start RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file_path],
        output="screen"
    )

    launch_description = LaunchDescription()

    # Start controllers in correct order
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
                on_exit=[joint_trajectory_position_controller_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_trajectory_position_controller_spawner,
                on_exit=[velocity_controller_spawner],
            )
        )
    )

    # Add launch actions
    launch_description.add_action(rviz)
    launch_description.add_action(gazebo)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(ackerman_controller)
    launch_description.add_action(steering_monitor)
    launch_description.add_action(rsp)

    return launch_description
