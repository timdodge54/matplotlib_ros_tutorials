import os

import launch.actions as actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    test_plotter = Node(
        package="test_plot",
        executable="test_plot.py",
        name="test_plotter",
        output="screen",
        emulate_tty=True,
    )
    test_pub = Node(
        package="test_plot",
        executable="test_pub.py",
        name="test_pub",
        output="screen",
        emulate_tty=True,
    )

    ld = LaunchDescription()
    ld.add_action(test_pub)
    ld.add_action(test_plotter)
    return ld
