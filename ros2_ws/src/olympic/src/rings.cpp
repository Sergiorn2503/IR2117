#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

#include <chrono>
#include <vector>

using namespace std::chrono_literals;
using  turtlesim::srv::SetPen;
using  turtlesim::srv::TeleportAbsolute;

std::vector<std::vector<float>> colors = {{0.0,0.0,255.0},{0.0,0.0,0.0},{255.0,0.0,0.0},{255.0,255.0,0.0},{0.0,255.0,0.0}}; //b,negro,r,y,g
std::vector<std::vector<float>> positions = {{3.5,5.5},{5.5,5.5},{8.0,5.5},{4.25,4.5},{6.75,4.5}}; //mismo orden colores

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rings");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    node->declare_parameter("radius",1.0);
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(500ms);

    double radius = node->get_parameter("radius").get_parameter_value().get<double>();

    rclcpp::Client<SetPen>::SharedPtr client_pen =
            node->create_client<SetPen>("/turtle1/set_pen");
        auto request_setpen =
            std::make_shared<SetPen::Request>();
        request_setpen-> off = 1.0;

        while (!client_pen->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "Interrupted while waiting for the service.");
                return 0;
                }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
            "service not available, waiting again...");
        }

        auto result_pen = client_pen->async_send_request(request_setpen);

        // Espera a que la llamada al servicio se complete.
        rclcpp::spin_until_future_complete(node, result_pen);

    rclcpp::Client<TeleportAbsolute>::SharedPtr client_teleport =
            node->create_client<TeleportAbsolute>("/turtle1/teleport_absolute");
        auto request_teleport =
            std::make_shared<TeleportAbsolute::Request>();
        request_teleport-> x = positions[0][0];
        request_teleport-> y = positions[0][1];
        request_teleport-> theta = 0.0;

        while (!client_teleport->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "Interrupted while waiting for the service.");
                return 0;
                }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
            "service not available, waiting again...");
        }

        auto result_teleport = client_teleport->async_send_request(request_teleport);

        // Espera a que la llamada al servicio se complete.
        rclcpp::spin_until_future_complete(node, result_teleport);

    while (rclcpp::ok()) {
        request_setpen-> r = colors[0][0];
        request_setpen-> g = colors[0][1];
        request_setpen-> b = colors[0][2];
        request_setpen-> width = 3.7;
        request_setpen-> off = 0.0;
        result_pen = client_pen->async_send_request(request_setpen);

        message.linear.x = 1.0;
        message.angular.z = 1.0 / radius;
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
        //rclcpp::sleep_for(5000ms);
    }

    rclcpp::shutdown();
    return 0;
}
