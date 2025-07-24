import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# ament_index_python is not needed when using FindPackageShare substitution

def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_package",
            default_value="misumi_gripper_hardware",
            description="Package with robot description files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_file",
            default_value="urdf/misumi_gripper.urdf.xacro",
            description="Path to the URDF/XACRO file relative to the package.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file_package",
            default_value="misumi_gripper_hardware", # Assuming it's in the same package
            description="Package with controller configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="config/misumi_gripper_controllers.yaml",
            description="Path to the controller configuration file relative to the package.",
        )
    )

    # Initialize Arguments
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")
    controllers_file_package = LaunchConfiguration("controllers_file_package")
    controllers_file = LaunchConfiguration("controllers_file")

    # Get robot description path and content
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare(robot_description_package), robot_description_file]
    )
    robot_description_content = Command(["xacro", " ", robot_description_path])
    robot_description = {"robot_description": robot_description_content}

    # Get controller config path
    gripper_controller_config = PathJoinSubstitution(
        [FindPackageShare(controllers_file_package), controllers_file]
    )

    # Controller Manager Node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, gripper_controller_config],
        output="screen",
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawner for Joint State Broadcaster
    # Note: We start this one first, as other controllers may depend on it.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner for the main gripper controller
    # This spawner is delayed until the joint_state_broadcaster_spawner has finished successfully.
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["misumi_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Use an event handler to launch the gripper controller spawner after the joint state broadcaster spawner exits
    # This solves the race condition.
    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    nodes_to_start = [
        controller_manager,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_gripper_controller_spawner, # This will trigger the gripper_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)