from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
	  Node(
	     package="robot_trajectory",
	     executable="polygon",
	     remappings=[
	       ("/cmd_vel", "/turtle1/cmd_vel"),
	      ],
	      parameters=[
	        {"segement_size" : 1.25},
	        {"number_segments" : 6}
	      ]
	    )
	 ])
