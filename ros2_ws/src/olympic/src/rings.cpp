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

std::vector<std::vector<double>> colors = {{0.0,0.0,255.0},{0.0,0.0,0.0},{255.0,0.0,0.0},{255.0,255.0,0.0},{0.0,255.0,0.0}}; //b,negro,r,y,g
bool stop = false;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rings"); //Crea Nodo
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    node->declare_parameter("radius",1.0); //Declaramos parametro 1.0 defecto
    double radius = node->get_parameter("radius").get_parameter_value().get<double>(); //cogemos valor parametro
    std::vector<std::vector<double>> positions = 
        {{3.0*radius,5.5*radius},{5.5*radius,5.5*radius},{8.0*radius,5.5*radius},{4.25*radius,4.5*radius},{6.75*radius,4.5*radius}};
    geometry_msgs::msg::Twist message; //Tipo mensaje
    rclcpp::WallRate loop_rate(500ms);

    rclcpp::Client<SetPen>::SharedPtr client_pen =   //Crea el cliente Pen
            node->create_client<SetPen>("/turtle1/set_pen");
        auto request_setpen =    //Crea el Request Pen
            std::make_shared<SetPen::Request>();
        request_setpen-> width = 7.0;
        request_setpen-> off = 1.0;

        while (!client_pen->wait_for_service(1s)) {   //Espera respuesta
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "Interrupted while waiting for the service.");
                return 0;
                }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
            "service not available, waiting again...");
        }

        auto result_pen = client_pen->async_send_request(request_setpen); //Envia resultado

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

    int angular_iterations = 4 * M_PI / (1.0/radius);

    while (rclcpp::ok() && !stop) {
        for(int i=0; i < 5; i++){
            request_setpen-> r = colors[i][0]; // da valores a los requirimientos
            request_setpen-> g = colors[i][1];
            request_setpen-> b = colors[i][2];
            request_setpen-> off = 0.0;
            result_pen = client_pen->async_send_request(request_setpen);  //Llama al servicio

            for(int v=0; v <= angular_iterations; v++){
                message.linear.x = 1.0;
                message.angular.z = 1.0/radius;
                publisher->publish(message);
                rclcpp::spin_some(node);
                loop_rate.sleep();
            }
            
            request_setpen-> off = 1.0;  //Sino se modifican se quedan los valores anteriores
            result_pen = client_pen->async_send_request(request_setpen);

            if(i+1 != 5){
                request_teleport-> x = positions[i+1][0];
                request_teleport-> y = positions[i+1][1];
                request_teleport-> theta = 0.0;
                result_teleport = client_teleport->async_send_request(request_teleport);
            }
        }
        stop = true;
    }

    rclcpp::shutdown();
    return 0;
}
