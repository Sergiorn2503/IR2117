#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

geometry_msgs::msg::Twist message;
bool stop = false;
double min_izq; double min_der;

auto Min(const sensor_msgs::msg::LaserScan::SharedPtr msg, int n){
    int min = n;
    for(int i= n; i <= n+9; i++){        
        if(msg->ranges[i] < msg->ranges[min]){
            min = i;
        }
    }
    return msg->ranges[min];
}

void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    min_izq = Min(msg, 0);
    min_der = Min(msg, 350);
    
    std::cout << "Min der:" << min_der << "     Min izq:" << min_izq << std::endl;

    if(min_der <= 1 || min_izq <= 1){
        stop = true;
    }

}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("publisher");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10, topic_callback);
    rclcpp::WallRate loop_rate(10ms);

    while (rclcpp::ok()){
        message.linear.x = 0.7;
        publisher->publish(message);

        if(stop){
            message.linear.x = 0.0;
            publisher->publish(message);

            if(min_izq > min_der){

                while(min_izq <= 1){
                    message.angular.z = 0.3;
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                }
            }else{
                
                while(min_der <= 1){
                    message.angular.z =-0.3;
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                }
            }

            message.angular.z = 0;
            publisher->publish(message);
            stop = false;
            
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
