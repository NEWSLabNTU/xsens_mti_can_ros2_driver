from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("xsens_mti_can_ros_driver"))
    params = pkg_share / "param" / "xsens_mti_can_ros_node.yaml"

    return LaunchDescription([
        Node(
            package="xsens_mti_can_ros_driver",
            executable="xsens_mti_can_ros_node",
            name="xsens_mti_can_ros_node",
            output="screen",
            emulate_tty=True,
            parameters=[str(params)],
        ),
    ])
