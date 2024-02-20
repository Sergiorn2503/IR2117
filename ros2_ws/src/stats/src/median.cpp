#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <vector>

std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32> > publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    vect.push_back(msg -> data);
    std_msgs::msg::Int32 out_msg;
    out_msg.data = msg->data;
    //publisher -> publish(out_msg);
    
    for(int i = 0; i < vect.size(); i++){
    	std::cout << vect[i] << std::endl;
    }
}

int main(int argc, char * argv[])
{
    std::vector<int> vect = {};
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("median");
    auto subscription = 
    	node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    publisher = node->create_publisher<std_msgs::msg::Int32>("median", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
