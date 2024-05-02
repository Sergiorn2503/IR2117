#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"

using namespace std::chrono_literals;
using turtlesim::srv::SetPen;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("publisher");

    rclcpp::Client<SetPen>::SharedPtr client =
        node->create_client<SetPen>("/turtle1/set_pen");
    auto request_setpen =
        std::make_shared<SetPen::Request>();
    request_setpen-> r = atoll(argv[1]);
    request_setpen-> g = atoll(argv[2]);
    request_setpen-> b = atoll(argv[3]);
    request_setpen-> width = atoll(argv[4]);
    request_setpen-> off = atoll(argv[5]);

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                "Interrupted while waiting for the service.");
            return 0;
            }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
        "service not available, waiting again...");
    }

    auto result = client->async_send_request(request_setpen);

    // Espera a que la llamada al servicio se complete.
    rclcpp::spin_until_future_complete(node, result);

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
