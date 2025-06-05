#include <chrono>

#include "cube_ca_plugins/dissemination_rules.hpp"

namespace custom_ca_plugins
{

using namespace std::chrono_literals;

class CustomDisseminationRules : public cube_ca_plugins::DisseminationRules
{
public:
  CustomDisseminationRules() : logger_(rclcpp::get_logger("CustomDisseminationRules")) {}

  void initialize(rclcpp::Node * node) override
  {
    cam_rate_ = rclcpp::Duration::from_seconds(node->declare_parameter("cam_rate", cam_rate_.seconds()));
    RCLCPP_INFO(logger_, "CAM rate: %.2fs", cam_rate_.seconds());
  }

  bool apply(etsi_its_cam_msgs::msg::CAM::SharedPtr) override
  {
    const rclcpp::Time now = rclcpp::Clock().now();
    const rclcpp::Duration elapsed = now - last_transmission_;
    return elapsed >= cam_rate_;
  }

  void notify(const etsi_its_cam_msgs::msg::CAM::ConstSharedPtr) override
  {
    last_transmission_ = rclcpp::Clock().now();
  }

private:
  rclcpp::Logger logger_;
  rclcpp::Duration cam_rate_ = 1000ms; /*< CAM rate, default is 1 Hz */
  rclcpp::Time last_transmission_;
};

}  // namespace custom_ca_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_ca_plugins::CustomDisseminationRules, cube_ca_plugins::DisseminationRules)
