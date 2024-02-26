#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32> > publisher;
int max = 0;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if(max > msg->data){
    	max = msg->data;
    }
    //Igual que min pero cambiando el if
    std_msgs::msg::Int32 out_msg;
    out_msg.data = max;  //Publicamos
    publisher -> publish(out_msg);
    
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("max");
    auto subscription = 
    	node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    publisher = node->create_publisher<std_msgs::msg::Int32>("max", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
