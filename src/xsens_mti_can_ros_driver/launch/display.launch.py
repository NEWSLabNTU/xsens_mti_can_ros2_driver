from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("xsens_mti_can_ros_driver"))
    default_model = pkg_share / "urdf" / "MTi_6xx.urdf"
    default_rviz = pkg_share / "rviz" / "display.rviz"

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=str(default_model),
        description="Absolute path to the URDF/Xacro robot description.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=str(default_rviz),
        description="Absolute path to the RViz config.",
    )

    robot_description = {
        "robot_description": Command(["xacro ", LaunchConfiguration("model")]),
    }

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_share / "launch" / "xsens_mti_can_node.launch.py")
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        output="screen",
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        driver_launch,
        robot_state_publisher,
        rviz,
    ])
