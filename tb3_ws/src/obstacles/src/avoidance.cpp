#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/bool.hpp"

#include<cmath>



void callback(const example_interfaces::msg::Bool::SharedPtr msg)
{
}

int main(int argc, char * argv[])
{
    /*
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto subscription1 =
        node->create_subscription<sensor_msgs::msg::LaserScan>("/front/obstacle", 10, callback);
    */
}
