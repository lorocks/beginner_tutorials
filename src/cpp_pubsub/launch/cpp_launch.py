from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

def generate_launch_descriptions():
    desc = LaunchDescription()
    frequency_para = DeclareLaunchArgument("pub_frequency", 750)

    talker = Node(
        package="cpp_pubsub",
        executable="talker",
    )

    listener = Node(
        package="cpp_pubsub",
        executable="listener",
    )

    desc.add_action(frequency_para)
    desc.add_action(talker)
    desc.add_action(listener)

    return desc