"""@file cpp_launch.py
@author Lowell Lobo
@brief Launch file to start up talker and listener nodes
@copyright Copyright (c) Lowell Lobo 2023
"""
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.conditions import IfCondition

"""@brief Generate the launch descriptions for ROS
"""
def generate_launch_description():
    desc = LaunchDescription()
    frequency_para = DeclareLaunchArgument(name="pub_frequency", default_value='750')
    ros_bag = DeclareLaunchArgument(name="ros_bag", default_value='True', choices=['True', 'False'])


    delete_bag = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('ros_bag')),
        cmd=["rm -rf ./results/ros2_bag"],
        shell = True
    )

    ros2_bag = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('ros_bag')),
        cmd=[[
            FindExecutable(name='ros2'),
            ' bag record -o ./results/ros2bag -a '
        ]],
        shell = True
    )

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

    desc.add_action(delete_bag)
    desc.add_action(frequency_para)
    desc.add_action(ros_bag)
    desc.add_action(talker)
    desc.add_action(listener)
    desc.add_action(ros2_bag)

    return desc
