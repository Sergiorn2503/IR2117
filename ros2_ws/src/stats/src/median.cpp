#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>

std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Float64> > publisher;
std::vector<int> vect = {};
int talla;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    vect.push_back(msg -> data);
    talla = vect.size();
    std_msgs::msg::Float64 out_msg;
    std::sort(vect.begin(), vect.end());
    
    int sum = std::accumulate(vect.begin(), vect.end(), 0);
    double mean = static_cast<double>(sum) / talla;
    
    out_msg.data = mean;
    publisher -> publish(out_msg);
    
    for(int i = 0; i < talla; i++){
    	std::cout << vect[i] << " ";
    }
    std::cout << std::endl;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("median");
    auto subscription = 
    	node->create_subscription<std_msgs::msg::Int32>("number", 10, topic_callback);
    publisher = node->create_publisher<std_msgs::msg::Float64>("median", 10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
