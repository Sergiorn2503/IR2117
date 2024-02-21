#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

int sum;
int mensajes = 1;
int mean;

std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32> > publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    sum += msg->data;
    mean = sum / mensajes;
    
    std_msgs::msg::Int32 out_msg;
    out_msg.data = mean;
    publisher -> publish(out_msg);
    mensajes += 1;
}

int main(int argc, char * argv[])
{
    sum = 0;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mean");
    auto subscription = 
    	node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    
    publisher = node->create_publisher<std_msgs::msg::Int32>("mean", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
