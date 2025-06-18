#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/bool.hpp"
#include <chrono>
#include <cstdlib>
#include <ctime>

using namespace std::chrono_literals;

// Flags for obstacle detection
bool obj_front = false;
bool obj_right = false;
bool obj_left = false;

// State machine state
enum State { MOVE_FORWARD = 1, CHOOSE_DIR, TURN_LEFT, TURN_RIGHT };
State state = MOVE_FORWARD;

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

  // Publishers & Subscribers
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  auto sub_front = node->create_subscription<example_interfaces::msg::Bool>(
    "/front/obstacle", 10, callback_front);
  auto sub_right = node->create_subscription<example_interfaces::msg::Bool>(
    "/right/obstacle", 10, callback_right);
  auto sub_left = node->create_subscription<example_interfaces::msg::Bool>(
    "/left/obstacle", 10, callback_left);

  std::srand(std::time(nullptr));
  rclcpp::WallRate rate(50ms);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    geometry_msgs::msg::Twist msg;

    switch (state) {
      case MOVE_FORWARD:
        msg.linear.x = 0.5;
        msg.angular.z = 0.0;
        publisher->publish(msg);

        if (obj_front) {
          state = CHOOSE_DIR;
        }
        break;

      case CHOOSE_DIR:
        // Stop before turning
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher->publish(msg);

        // Decide direction based on side obstacles or randomly
        if (obj_right && !obj_left) {
          state = TURN_LEFT;
        } else if (obj_left && !obj_right) {
          state = TURN_RIGHT;
        } else {
          // neither or both blocked -> random
          double r = static_cast<double>(std::rand()) / RAND_MAX;
          state = (r < 0.5) ? TURN_LEFT : TURN_RIGHT;
        }
        break;

      case TURN_LEFT:
        msg.linear.x = 0.0;
        msg.angular.z = 0.3;
        publisher->publish(msg);

        if (!obj_front) {
          state = MOVE_FORWARD;
        }
        break;

      case TURN_RIGHT:
        msg.linear.x = 0.0;
        msg.angular.z = -0.3;
        publisher->publish(msg);

        if (!obj_front) {
          state = MOVE_FORWARD;
        }
        break;
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

