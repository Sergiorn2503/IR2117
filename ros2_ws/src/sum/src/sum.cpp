#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>

int sum;
std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32> > publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    sum += msg->data;  //Cogemos dato de msg
    std_msgs::msg::Int32 out_msg;
    out_msg.data = sum;  //mensaje de salida = sum
    publisher -> publish(out_msg);
}

int main(int argc, char * argv[])
{
    sum = 0;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sum"); //Crea Nodo
    auto subscription =     //Crea Subs
    	node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    
    publisher = node->create_publisher<std_msgs::msg::Int32>("sum", 10); //Crea Publish
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
