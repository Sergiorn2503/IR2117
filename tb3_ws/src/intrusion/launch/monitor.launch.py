from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions

def generate_launch_description():
    return LaunchDescription([
        Node(
            package= "intrusion",
            executable="detector",
            namespace= "north",
            parameters=[
                {"obs_angle_min": -0.3927},
                {"obs_angle_max": 0.3927},
                {"obs_threshold": 2.0}]
        ),
        Node(
            package= "intrusion",
            executable="detector",
            namespace= "northwest",
            parameters=[
                {"obs_angle_min": 0.3927},
                {"obs_angle_max": 1.1781},
                {"obs_threshold": 2.0}]
        ),
        Node(
            package= "obstacles",
            executable="detector",
            namespace= "west",
            parameters=[
                {"obs_angle_min": 1.1781},
                {"obs_angle_max": 1.9635},
                {"obs_threshold": 2.0}]
        ),
        Node(
            package= "intrusion",
            executable="detector",
            namespace= "southwest",
            parameters=[
                {"obs_angle_min": 1.9635},
                {"obs_angle_max": 2.7489},
                {"obs_threshold": 2.0}]
        ),
        Node(
            package= "intrusion",
            executable="detector",
            namespace= "south",
            parameters=[
                {"obs_angle_min": 2.7489},
                {"obs_angle_max": 3.5343},
                {"obs_threshold": 2.0}]
        ),
        Node(
            package= "intrusion",
            executable="detector",
            namespace= "southeast",
            parameters=[
                {"obs_angle_min": 3.5343},
                {"obs_angle_max": 4.3197},
                {"obs_threshold": 2.0}]
        ),
        Node(
            package= "intrusion",
            executable="detector",
            namespace= "east",
            parameters=[
                {"obs_angle_min": 4.3197},
                {"obs_angle_max": 5.105},
                {"obs_threshold": 2.0}]
        ),
        Node(
            package= "intrusion",
            executable="detector",
            namespace= "northeast",
            parameters=[
                {"obs_angle_min": 5.105},
                {"obs_angle_max": 5.8905},
                {"obs_threshold": 2.0}]
        ),
        Node(
            package= "intrusion",
            executable="monitor"
        )
    ])