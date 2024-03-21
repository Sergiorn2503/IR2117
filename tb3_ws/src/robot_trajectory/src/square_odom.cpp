#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

using namespace std::chrono_literals;

double global_x; double global_y; double global_angle;
double initial_x; double initial_y; double initial_angle;

double euler_from_quaternion(geometry_msgs::msg::Quaternion quaternion){
	double x = quaternion.x;
	double y = quaternion.y;
	double z = quaternion.z;
	double w = quaternion.w;

	double siny_cosp = 2*(w*x + y*z);
	double cosy_cosp = 1 - 2*(x*x + y*y);
	double yaw = std::atan2(siny_cosp, cosy_cosp);
	return yaw;
}

bool aux = true;
void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
	
	if(aux){
		initial_x = msg->pose.pose.position.x;
		initial_y = msg->pose.pose.position.y;
		initial_angle = euler_from_quaternion(msg->pose.pose.orientation);

		aux = false;
	}

	global_x = msg->pose.pose.position.x;
	global_y = msg->pose.pose.position.y;
	global_angle = euler_from_quaternion(msg->pose.pose.orientation);

	RCLCPP_INFO(rclcpp::get_logger("odom_listener"), "Position: (%f,%f,%f)", global_x, global_y,global_angle);
	RCLCPP_INFO(rclcpp::get_logger("odom_listener"), "Initial: (%f,%f,%f)", initial_x, initial_y,initial_angle);
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
    
    int linear_iterations = square_length / (0.01 * linear_speed);
    int angular_iterations = M_PI_2 / (0.01 * angular_speed);
    
    for(int s=0; s<4; s++){
    
	    int i=0;
	    while (rclcpp::ok() && i<linear_iterations) {
	    	i++;
	    	message.linear.x = linear_speed;
		publisher->publish(message);
		rclcpp::spin_some(node);
		loop_rate.sleep();
	    }
	    
	    message.linear.x = 0;
	    publisher->publish(message);
	    
	    i = 0;
	    while (rclcpp::ok() && i<angular_iterations) {
	    	i++;
	    	message.angular.z = angular_speed;
		publisher->publish(message);
		rclcpp::spin_some(node);
		loop_rate.sleep();
	    }
	    
	    message.angular.z = 0;
	    publisher->publish(message);
    }
    
	while(rclcpp::ok()){
		RCLCPP_INFO(rclcpp::get_logger("odom_listener"), "Position: (%f,%f,%f)", global_x, global_y,global_angle);
		RCLCPP_INFO(rclcpp::get_logger("odom_listener"), "Initial: (%f,%f,%f)", initial_x, initial_y,initial_angle);
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}

	rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
