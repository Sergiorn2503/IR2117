#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <iostream>

std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32MultiArray> > publisher;
int max = 0;
int min = 0;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if(max > msg->data){
    	max = msg->data;
    }
    
    if(min < msg->data){
    	min = msg->data;
    }
    
    //combinamos condiciones
    std_msgs::msg::Int32MultiArray out_msg; //ya no es int32
    
    out_msg.data.push_back(min); //Añadimos al array salida
    out_msg.data.push_back(max);
    
    publisher -> publish(out_msg);
    
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("minmax");
    auto subscription = 
    	node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    publisher = node->create_publisher<std_msgs::msg::Int32MultiArray>("minmax", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
