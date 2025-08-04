import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# 导入 ParameterValue
from launch_ros.parameter_descriptions import ParameterValue

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
            default_value="misumi_gripper_hardware",
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
    
    # *** 这是关键的修改 ***
    # 使用 ParameterValue 包装 Command 的结果，并指定其类型为 str
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

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
        remappings=[
        ("/joint_states", "/misumi_gripper/joint_states"),
        # You can add other remappings here too
        # ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"), 
    ],
    namespace="misumi"
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
       parameters=[
        {
            'robot_description': robot_description,
            # 添加下面这个参数！
            'frame_prefix': 'misumi/'
        }
        ],
        remappings=[
        ("/joint_states", "/misumi_gripper/joint_states")
        ],
        namespace="misumi"
    )

    # Spawner for Joint State Broadcaster
    gripper_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_joint_state_broadcaster", "--controller-manager", "/misumi/controller_manager"],
    )

    # Spawner for the main gripper controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["misumi_gripper_controller", "--controller-manager", "/misumi/controller_manager"],
        output="screen",
    )

    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler= OnProcessStart(
            target_action=controller_manager,
            on_start=[
                gripper_joint_state_broadcaster_spawner,
                gripper_controller_spawner,
            ],
        )
    )    

    nodes_to_start = [
        robot_state_publisher_node,        
        controller_manager,
        delay_gripper_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)