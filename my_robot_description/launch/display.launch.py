from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the argument for the URDF path
    urdf_path_arg = DeclareLaunchArgument(
        name="urdf_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("my_robot_description"),
            "urdf",
            "my_robot.urdf.xacro"
        ]),
        description="Path to the URDF file"
    )

    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", LaunchConfiguration("urdf_path")])
        }]
    )

    # Define the joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    # Define the rviz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )

    # Return the LaunchDescription
    return LaunchDescription([
        urdf_path_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])