#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <cmath>

#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;

geometry_msgs::msg::Twist message;
bool north; bool northeast; bool east; bool southeast;
bool south; bool southwest; bool west; bool northwest;
int state; double angular_dif; double initial_angle; double global_angle;
bool stop;


double angle_difference(double initial_angle, double global_angle) {
    double dtheta = global_angle - initial_angle;
    if (dtheta > M_PI) {
        dtheta -= 2 * M_PI;
    } else if (dtheta < -M_PI) {
        dtheta += 2 * M_PI;
    }
    return dtheta;
}
double euler_from_quaternion(geometry_msgs::msg::Quaternion quaternion){
	double qx = quaternion.x;
	double qy = quaternion.y;
	double qz = quaternion.z;
	double qw = quaternion.w;

	double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
	return yaw;
}
void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
	
	if(stop){
		initial_angle = euler_from_quaternion(msg->pose.pose.orientation);
		stop = false;
	}

	global_angle = euler_from_quaternion(msg->pose.pose.orientation);

	angular_dif = angle_difference(initial_angle, global_angle);
}


void callback_north(const example_interfaces::msg::Bool::SharedPtr msg)
{
    north = msg->data;  //Obtiene valor
}

void callback_northeast(const example_interfaces::msg::Bool::SharedPtr msg)
{
    northeast = msg->data;  //Obtiene valor
}

void callback_east(const example_interfaces::msg::Bool::SharedPtr msg)
{
    east = msg->data;  //Obtiene valor
}

void callback_shoutheast(const example_interfaces::msg::Bool::SharedPtr msg)
{
    southeast = msg->data;  //Obtiene valor
}

void callback_south(const example_interfaces::msg::Bool::SharedPtr msg)
{
    south = msg->data;  //Obtiene valor
}

void callback_southwest(const example_interfaces::msg::Bool::SharedPtr msg)
{
    southwest = msg->data;  //Obtiene valor
}

void callback_west(const example_interfaces::msg::Bool::SharedPtr msg)
{
    west = msg->data;  //Obtiene valor
}

void callback_northwest(const example_interfaces::msg::Bool::SharedPtr msg)
{
    northwest = msg->data;  //Obtiene valor
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("monitor");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto sub_odom =
        node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, callback_odom); //subscripción a cada nodo
    auto sub_north =
        node->create_subscription<example_interfaces::msg::Bool>("/north", 10, callback_north);
    auto sub_northeast =
        node->create_subscription<example_interfaces::msg::Bool>("/northeast", 10, callback_northeast);
    auto sub_east =
        node->create_subscription<example_interfaces::msg::Bool>("/east", 10, callback_east);
    auto sub_southeast =
        node->create_subscription<example_interfaces::msg::Bool>("/southeast", 10, callback_shoutheast);
    auto sub_south =
        node->create_subscription<example_interfaces::msg::Bool>("/south", 10, callback_south);
    auto sub_southwest =
        node->create_subscription<example_interfaces::msg::Bool>("/southwest", 10, callback_southwest);
    auto sub_west =
        node->create_subscription<example_interfaces::msg::Bool>("/west", 10, callback_west);
    auto sub_northwest =
        node->create_subscription<example_interfaces::msg::Bool>("/northweast", 10, callback_northwest);
    
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(50ms);
    double angular_speed = 0.2; // postivo va izq
    state = 1;

    while(rclcpp::ok()){
        switch (state){   //Para estados
            case 1:  //Parado
                message.angular.z = 0.0;
                stop = true;
                publisher->publish(message);
                if(west){
                    state = 2;
                }else if(east){
                    state = 3;
                }else if(southwest){
                    state = 4;
                }else if(south){
                    state = 5;
                }else if(southeast){
                    state = 6;
                }else if(northwest){
                    state = 7;
                }else if(north){
                    state = 8;
                }else if(northeast){
                    state = 9;
                }
                rclcpp::spin_some(node); 

                break;  //Obligatorio, corta estado
            
            case 2:
                while (west && angular_dif < M_PI_2) {
                    message.angular.z = angular_speed;
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    angular_dif = angle_difference(initial_angle, global_angle);
                    loop_rate.sleep();
                }
                state = 1;

                break;

            case 3: 
                while (east && angular_dif < M_PI_2) {
                    message.angular.z = - angular_speed; // poque es hacia la der
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    angular_dif = angle_difference(initial_angle, global_angle);
                    loop_rate.sleep();
                }
                state = 1;

                break;

            case 4:
                while (southwest && angular_dif < (0.75*M_PI)) {
                    message.angular.z = angular_speed;
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    angular_dif = angle_difference(initial_angle, global_angle);
                    loop_rate.sleep();
                }
                state = 1;

                break;
            
            case 5:
                while (south && angular_dif < M_PI) {
                    message.angular.z = - angular_speed; // poque es hacia la der(aqui indif)
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    angular_dif = angle_difference(initial_angle, global_angle);
                    loop_rate.sleep();
                }
                state = 1;

                break;
            
            case 6:
                while (southeast && angular_dif < (0.75*M_PI)) {
                    message.angular.z = - angular_speed; // poque es hacia la der
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    angular_dif = angle_difference(initial_angle, global_angle);
                    loop_rate.sleep();
                }
                state = 1;

                break;
            
            case 7:
                while (northwest && angular_dif < (0.25*M_PI)) {
                    message.angular.z = angular_speed; 
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    angular_dif = angle_difference(initial_angle, global_angle);
                    loop_rate.sleep();
                }
                state = 1;

                break;
            
            case 8:
                // EL Norte me varia con el desp del robot, se emueve con el detector por lo que el norte
                // sera hacia donde quede mirando el robot, por lo que si hay un obj ya estaría orientado
                if(!north){
                    state = 1;
                }

            case 9:
                while (northeast && angular_dif < (0.25*M_PI)) {
                    message.angular.z = - angular_speed; // poque es hacia la der
                    publisher->publish(message);
                    rclcpp::spin_some(node);
                    angular_dif = angle_difference(initial_angle, global_angle);
                    loop_rate.sleep();
                }
                state = 1;

                break;
        }

    }

    rclcpp::shutdown(); 
    return 0;
}

