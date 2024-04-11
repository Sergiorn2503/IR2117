#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int min = 0;
    for(int i= 1; i <10; i++){        
        if(msg->ranges[i] < msg->ranges[min]){
            min = i;
        }
    }
    std::cout << "Valor Mín (0-9)" << msg->ranges[min] << std::endl;

    min = 350;
    for(int i= 351; i <360; i++){        
        if(msg->ranges[i] < msg->ranges[min]){
            min = i;
        }
    }
    std::cout << "Valor Mín" << min << ":" << msg->ranges[min] << std::endl;

}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("publisher");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 0.0);
    auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10, topic_callback);
    rclcpp::WallRate loop_rate(10ms);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
