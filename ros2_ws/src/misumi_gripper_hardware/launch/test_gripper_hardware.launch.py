import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

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

    # Initialize Arguments
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")

    # Get robot description
    robot_description_content = Command(
        [
            "xacro ",
            os.path.join(
                get_package_share_directory(robot_description_package), robot_description_file
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get controller config
    gripper_controller_config = os.path.join(
        get_package_share_directory("misumi_gripper"), # Assumes the controller package is in the same workspace
        "config",
        "misumi_gripper_controllers.yaml"
    )

    # Controller Manager Node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, gripper_controller_config],
        output="screen",
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Gripper Controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["misumi_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription(declared_arguments + [
        controller_manager,
        joint_state_broadcaster_spawner,
        gripper_controller_spawner,
        robot_state_publisher_node,
    ])
