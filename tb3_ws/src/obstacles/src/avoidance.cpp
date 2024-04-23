#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp
#include "example_interfaces/msg/bool.hpp"

"#include <tinyfsm.hpp>"

#include <chrono>

using namespace std::chrono_literals;

bool obj_front; bool obj_right; bool obj_left;

bool foward; bool choice = false; bool turn_left = false; bool turn_right = false;

void callback_front(const example_interfaces::msg::Bool::SharedPtr msg)
{
    obj_front = msg->data;
}

void callback_right(const example_interfaces::msg::Bool::SharedPtr msg)
{
    obj_right = msg->data;
}

void callback_left(const example_interfaces::msg::Bool::SharedPtr msg)
{
    obj_left = msg->data;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("avoidance");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto sub_front =
        node->create_subscription<example_interfaces::msg::Bool>("/front/detector", 10, callback_front);
    auto sub_right =
        node->create_subscription<example_interfaces::msg::Bool>("/right/detector", 10, callback_right);
    auto sub_left =
        node->create_subscription<example_interfaces::msg::Bool>("/left/detector", 10, callback_left);
    
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(50ms);

    while(rclcpp::ok()){
        while(foward){
            message.linear.x = 0.5;
            publisher->publish(message);
            rclcpp::spin_some(node);
            if()
        }

    }

    rclcpp::shutdown();
    return 0;
}
