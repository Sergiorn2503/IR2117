#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;


int main(int argc, char * argv[])
{
 rclcpp::init(argc, argv);
 auto node = rclcpp::Node::make_shared("publisher");
 auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
 node->declare_parameter("radius",1.0);
 geometry_msgs::msg::Twist message;
 auto publish_count = 0;
 rclcpp::WallRate loop_rate(500ms);

  double radius = node->get_parameter("radius").get_parameter_value().get<double>();

 while (rclcpp::ok()) {
   message.linear.x = 1.0;
   message.angular.z = 1.0 / radius;
   publisher->publish(message);
   rclcpp::spin_some(node);
   loop_rate.sleep();
 }
 rclcpp::shutdown();
 return 0;
}
