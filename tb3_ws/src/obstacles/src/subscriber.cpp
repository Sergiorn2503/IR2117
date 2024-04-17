#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include "example_interfaces/msg/bool.hpp"

std::shared_ptr< rclcpp::Publisher<example_interfaces::msg::Bool> > publisher;

void callback(const std_msgs::msg::String::SharedPtr msg)
{
   
}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("subscriber");
    auto subscription =
        node->create_subscription<std_msgs::msg::String>("topic", 10, callback);
    publisher = node->create_publisher<example_interfaces::msg::Bool>("obstacle",10);     
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
