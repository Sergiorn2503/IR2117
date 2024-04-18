#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>
#include "example_interfaces/msg/bool.hpp"

#include<cmath>

std::shared_ptr< rclcpp::Publisher<example_interfaces::msg::Bool> > publisher;

void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    example_interfaces::msg::Bool out_msg;
    out_msg.data = false;
    publisher->publish(out_msg);
}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dtector");
    auto subscription =
        node->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, callback);
    publisher = node->create_publisher<example_interfaces::msg::Bool>("obstacle", 10);
    node->declare_parameter("obs_angle_min", -M_PI/8);
    node->declare_parameter("obs_angle_max", M_PI/8);
    node->declare_parameter("obs_threshold", 1.0);
    obs_angle_min = node->get_parameter("obs_angle_min").get_parameter_value().get<double>();
    obs_angle_max = node->get_parameter("obs_angle_max").get_parameter_value().get<double>();
    obs_threshold = node->get_parameter("obs_threshold").get_parameter_value().get<double>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
