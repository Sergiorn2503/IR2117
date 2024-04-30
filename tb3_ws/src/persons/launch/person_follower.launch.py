from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions

def generate_launch_description():
    return LaunchDescription([
        Node(
            package= "persons",
            executable="detector",
            namespace= "front",
        ),
        Node(
            package= "persons",
            executable="detector",
            namespace= "left",
        ),
        Node(
            package= "persons",
            executable="detector",
            namespace= "right",
        ),
        Node(
            package= "persons",
            executable="follower"
        )
    ])