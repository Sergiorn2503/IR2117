#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::cout << "Valor 0:" << msg->ranges[0] << std::endl;
    std::cout << "Valor 90:" << msg->ranges[90] << std::endl;
    std::cout << "Valor 180:" << msg->ranges[180] << std::endl;
    std::cout << "Valor 270:" << msg->ranges[270] << std::endl;
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
