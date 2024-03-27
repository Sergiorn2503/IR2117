#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

using namespace std::chrono_literals;

double global_x; double global_y; double global_angle;
double initial_x; double initial_y; double initial_angle;
double distance; double angular_dif;
bool stop;

double angle_difference(double initial_angle, double global_angle) {
    double dtheta = global_angle - initial_angle;
    if (dtheta > M_PI) {
        dtheta -= 2 * M_PI;
    } else if (dtheta < -M_PI) {
        dtheta += 2 * M_PI;
    }
    return dtheta;
}

double euler_from_quaternion(geometry_msgs::msg::Quaternion quaternion){
	double qx = quaternion.x;
	double qy = quaternion.y;
	double qz = quaternion.z;
	double qw = quaternion.w;

	double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
	return yaw;
}


void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
	
	if(stop){
		initial_x = msg->pose.pose.position.x;
		initial_y = msg->pose.pose.position.y;
		initial_angle = euler_from_quaternion(msg->pose.pose.orientation);

		stop = false;
	}

	global_x = msg->pose.pose.position.x;
	global_y = msg->pose.pose.position.y;
	global_angle = euler_from_quaternion(msg->pose.pose.orientation);

	distance = std::sqrt((global_x - initial_x) * (global_x - initial_x) + (global_y - initial_y) * (global_y - initial_y)); 
	angular_dif = angle_difference(initial_angle, global_angle);
	
	RCLCPP_INFO(rclcpp::get_logger("odom_listener"), "Position: (%f,%f,%f)", global_x, global_y,global_angle);
	RCLCPP_INFO(rclcpp::get_logger("odom_listener"), "Initial: (%f,%f,%f)", initial_x, initial_y,initial_angle);
	RCLCPP_INFO(rclcpp::get_logger("odom_listener"), "Distance: (%f)", distance);
	RCLCPP_INFO(rclcpp::get_logger("odom_listener"), "Angilar Dif: (%f)", angular_dif);

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cmd_vel");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto subscription = node->create_subscription<nav_msgs::msg::Odometry>("/odom",10,odom_callback);
	
    node->declare_parameter("linear_speed",0.1);
    node->declare_parameter("angular_speed",M_PI / 20);
    node->declare_parameter("square_length",1.0);
 
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(10ms);
    
    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    double square_length = node->get_parameter("square_length").get_parameter_value().get<double>();

    for(int s=0; s<4; s++){
		stop = true;						
	    
	    while (rclcpp::ok() && distance < square_length) {
	    	message.linear.x = linear_speed;
			publisher->publish(message);
			rclcpp::spin_some(node);
			distance = std::sqrt((global_x - initial_x) * (global_x - initial_x) + (global_y - initial_y) * (global_y - initial_y)); 
			loop_rate.sleep();
	    }
	    distance = 0;

	    message.linear.x = 0;
	    publisher->publish(message);
	    
		stop = true;
	    while (rclcpp::ok() && angular_dif < M_PI_2) {
	    	message.angular.z = angular_speed;
		publisher->publish(message);
		rclcpp::spin_some(node);
		angular_dif = angle_difference(initial_angle, global_angle);
		loop_rate.sleep();
	    }
	    
		angular_dif = 0;
	    message.angular.z = 0;
	    publisher->publish(message);
    }

	rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
