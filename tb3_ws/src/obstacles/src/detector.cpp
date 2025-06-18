#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "example_interfaces/msg/bool.hpp"
#include <cmath>

using std::placeholders::_1;

class DetectorNode : public rclcpp::Node
{
public:
  DetectorNode()
  : Node("detector")
  {
    // Declare angle windows and distance threshold
    this->declare_parameter("front/angle_min", -M_PI/8);
    this->declare_parameter("front/angle_max", M_PI/8);
    this->declare_parameter("right/angle_min", -M_PI/2);
    this->declare_parameter("right/angle_max", -M_PI/8);
    this->declare_parameter("left/angle_min", M_PI/8);
    this->declare_parameter("left/angle_max", M_PI/2);
    this->declare_parameter("obs_threshold", 0.5);

    // Get parameters
    obs_threshold_ = this->get_parameter("obs_threshold").as_double();
    angle_min_["front"] = this->get_parameter("front/angle_min").as_double();
    angle_max_["front"] = this->get_parameter("front/angle_max").as_double();
    angle_min_["right"] = this->get_parameter("right/angle_min").as_double();
    angle_max_["right"] = this->get_parameter("right/angle_max").as_double();
    angle_min_["left"] = this->get_parameter("left/angle_min").as_double();
    angle_max_["left"] = this->get_parameter("left/angle_max").as_double();

    // Create three publishers
    pub_front_ = this->create_publisher<example_interfaces::msg::Bool>("/front/obstacle", 10);
    pub_right_ = this->create_publisher<example_interfaces::msg::Bool>("/right/obstacle", 10);
    pub_left_ = this->create_publisher<example_interfaces::msg::Bool>("/left/obstacle", 10);

    // Laser scan subscription
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&DetectorNode::scan_callback, this, _1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Check each region
    publish_region(msg, "front", pub_front_);
    publish_region(msg, "right", pub_right_);
    publish_region(msg, "left", pub_left_);
  }

  void publish_region(const sensor_msgs::msg::LaserScan::SharedPtr & msg,
                      const std::string & region,
                      rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr publisher)
  {
    example_interfaces::msg::Bool out_msg;
    out_msg.data = false;
    double angle = msg->angle_min;
    for (auto range : msg->ranges) {
      double norm_angle = angle;
      if (norm_angle > M_PI) norm_angle -= 2 * M_PI;
      if (norm_angle < -M_PI) norm_angle += 2 * M_PI;

      if (norm_angle >= angle_min_[region] && norm_angle <= angle_max_[region]) {
        if (range <= obs_threshold_) {
          out_msg.data = true;
          break;
        }
      }
      angle += msg->angle_increment;
    }
    publisher->publish(out_msg);
  }

  // Parameters
  double obs_threshold_;
  std::map<std::string, double> angle_min_, angle_max_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr pub_front_, pub_right_, pub_left_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectorNode>());
  rclcpp::shutdown();
  return 0;
}

