#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/Int32MultiArray.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <map>


std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32MultiArray> > publisher;
std::vector<int> vect = {};
std::map<int, int> map;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    vect.push_back(msg -> data);
    map[msg -> data]++;
    
    std_msgs::msg::Int32MultiArray out_msg;
    int mode;
    int max = 0;
    
    for(auto &pair : map) {
        if(max < pair.second){
        	max = pair.second;
        }
    }
    
    for(auto &pair : map) {
        if(pair.second == max){
            out_msg.data.push_back(pair.first);
        }
    }
    
    publisher -> publish(out_msg);    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mode");
    auto subscription = 
    	node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    publisher = node->create_publisher<std_msgs::msg::Int32MultiArray>("mode", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
