#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

using namespace std::chrono_literals;
using  turtlesim::srv::SetPen;
using  turtlesim::srv::TeleportAbsolute;



int main(int argc, char * argv[])
{
    if(argc < 9) {
    std::cout << "Por favor, proporciona 8 argumentos: r g b width off x y theta" << std::endl;
    return 0;
    }

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("publisher");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    node->declare_parameter("radius",1.0);
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(500ms);

    double radius = node->get_parameter("radius").get_parameter_value().get<double>();

    while (rclcpp::ok()) {
        rclcpp::Client<SetPen>::SharedPtr client_pen =
            node->create_client<SetPen>("/turtle1/set_pen");
        auto request_setpen =
            std::make_shared<SetPen::Request>();
        request_setpen-> r = std::stoi(argv[1]);
        request_setpen-> g = std::stoi(argv[2]);
        request_setpen-> b = std::stoi(argv[3]);
        request_setpen-> width = std::stoi(argv[4]);
        request_setpen-> off = std::stoi(argv[5]);

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
        request_teleport-> x = std::stof(argv[6]);
        request_teleport-> y = std::stof(argv[7]);
        request_teleport-> theta = std::stof(argv[8]);

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

        message.linear.x = 1.0;
        message.angular.z = 1.0 / radius;
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
