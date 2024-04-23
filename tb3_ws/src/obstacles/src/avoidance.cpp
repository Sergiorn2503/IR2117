#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/bool.hpp"

#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;

geometry_msgs::msg::Twist message;
bool obj_front; bool obj_right; bool obj_left;
int state;

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
        node->create_subscription<example_interfaces::msg::Bool>("/front/obstacle", 10, callback_front);
    auto sub_right =
        node->create_subscription<example_interfaces::msg::Bool>("/right/obstacle", 10, callback_right);
    auto sub_left =
        node->create_subscription<example_interfaces::msg::Bool>("/left/obstacle", 10, callback_left);
    
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(50ms);

    srand(time(0)); //para el n random

    double n;
    state = 1;
    while(rclcpp::ok()){
        switch (state){
            case 1:
                message.linear.x = 0.5;
                publisher->publish(message);

                if(obj_front){
                    message.linear.x = 0.0;
                    publisher->publish(message);

                    if(obj_right){
                        state = 4;
                    }else if(obj_left){
                        state = 3;
                    }else{
                        state = 2;
                    }
                }
                rclcpp::spin_some(node);

                break;
            
            case 2:
                n = static_cast<double>(rand()) / RAND_MAX;
                if(n > 0.5){
                state = 3;
                }else{
                    state = 4;
                }
                rclcpp::spin_some(node);

                break;

            case 3: 
                message.angular.z = -0.3;
                publisher->publish(message);

                if(!obj_front){
                    message.angular.z = 0.0;
                    publisher->publish(message);
                    state = 1;  
                }
                rclcpp::spin_some(node);

                break;

            case 4:
                message.angular.z = 0.3;
                publisher->publish(message);

                if(!obj_front){
                    message.angular.z = 0.0;
                    publisher->publish(message);
                    state = 1;  
                }
                rclcpp::spin_some(node);

                break;
        }

    }

    rclcpp::shutdown();
    return 0;
}
