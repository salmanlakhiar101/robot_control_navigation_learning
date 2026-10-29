from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
import os



def generate_launch_description():


    # This will get the path to the URDF file as a parameter, it also has a default value for our robot file. 
    model_argument = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(get_package_share_directory("robot_description"), "urdf", "bumperbot_complete.urdf.xacro"),
        description = "Path to the Robot URDF file" 
    )

    #This will convert the model from Xacro to URDF
    robot_description = ParameterValue(
            Command([
                "xacro ",
                LaunchConfiguration("model"),
                " is_ignition:=false"
            ]),
        value_type=str
    )


    #Run the robot_state_publisher node with our URDF file
    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description": robot_description}]
    )

    joint_state_publisher_gui = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui",
    )

    rviz2_node =  Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "screen",
        arguments = ["-d", os.path.join(get_package_share_directory("robot_bringup"), "rviz", "rviz_config.rviz")]
    )

    return LaunchDescription([
        model_argument,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2_node
    ])