#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("square");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    node->declare_parameter("linear_speed",0.8);
    node->declare_parameter("angular_speed",0.5);
    node->declare_parameter("segement_size",0.5);
    node->declare_parameter("number_segments",4);
 
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(10ms);
    
    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    double segement_size = node->get_parameter("segement_size").get_parameter_value().get<double>();
    int number_segments = node->get_parameter("number_segments").get_parameter_value().get<int>();

    int linear_iterations = segement_size / (0.01 * linear_speed);
    int angular_iterations = (2*M_PI/number_segments) / (0.01 * angular_speed);
    
    for(int s=0; s<number_segments; s++){
    
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
    
    rclcpp::shutdown();
    return 0;
}
