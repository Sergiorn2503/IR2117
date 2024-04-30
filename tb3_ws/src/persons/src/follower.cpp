#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/bool.hpp"

#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;

geometry_msgs::msg::Twist message;
bool person_front; bool person_right; bool person_left;
int state;

void callback_front(const example_interfaces::msg::Bool::SharedPtr msg)
{
    person_front = msg->data;
}

void callback_right(const example_interfaces::msg::Bool::SharedPtr msg)
{
    person_right = msg->data;
}

void callback_left(const example_interfaces::msg::Bool::SharedPtr msg)
{
    person_left = msg->data;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("follower");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto sub_front =
        node->create_subscription<example_interfaces::msg::Bool>("/front/person", 10, callback_front);
    auto sub_right =
        node->create_subscription<example_interfaces::msg::Bool>("/right/person", 10, callback_right);
    auto sub_left =
        node->create_subscription<example_interfaces::msg::Bool>("/left/person", 10, callback_left);
    
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(50ms);

    srand(time(0)); //para el n random

    double n;
    state = 1;
    while(rclcpp::ok()){
        switch (state){
            case 1:
                message.linear.x = 0.0;
                publisher->publish(message);

                if(person_front){
                    state = 2;
                }else if(person_right){
                    state = 3;
                }else if(person_left){
                    state = 4;
                }

                rclcpp::spin_some(node);

                break;
            
            case 2:
                message.linear.x = 0.7;
                publisher->publish(message);


                if(!person_front){
                    state = 1;
                }

                rclcpp::spin_some(node);

                break;

            case 3: 
                message.angular.z = -0.3;
                publisher->publish(message);

                
                if(person_front){
                    state = 2;
                }else if(!person_right){
                    state = 1;
                }

                rclcpp::spin_some(node);

                break;

            case 4:
                message.angular.z = 0.3;
                publisher->publish(message);

                if(person_front){
                    state = 2;
                }else if(!person_left){
                    state = 1;
                }

                rclcpp::spin_some(node);
                
                break;
        }

    }

    rclcpp::shutdown();
    return 0;
}
