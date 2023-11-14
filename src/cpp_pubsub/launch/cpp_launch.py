"""@file cpp_launch.py
@author Lowell Lobo
@brief Launch file to start up talker and listener nodes

"""
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

"""@brief Generate the launch descriptions for ROS
"""
def generate_launch_description():
    desc = LaunchDescription()
    frequency_para = DeclareLaunchArgument(name="pub_frequency", default_value='750')

    talker = Node(
        package="cpp_pubsub",
        executable="talker",
        parameters=[
            {
                "pub_frequency": LaunchConfiguration("pub_frequency")
            },
        ]
    )

    listener = Node(
        package="cpp_pubsub",
        executable="listener",
    )

    desc.add_action(frequency_para)
    desc.add_action(talker)
    desc.add_action(listener)

    return desc