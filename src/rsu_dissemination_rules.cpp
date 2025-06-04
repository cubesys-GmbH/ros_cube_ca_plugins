#include <chrono>

#include "cube_ca_plugins/dissemination_rules.hpp"
#include "cube_ca_plugins/visibility_control.h"

namespace cube_ca_plugins
{

using namespace std::chrono_literals;

/**
 * CAM dissemination rules according to EN 302 637-2 v1.4.1 section 6.1.4
 */
class CUBE_CA_PLUGINS_PUBLIC RsuDisseminationRules : public DisseminationRules
{
public:
  RsuDisseminationRules() : logger_(rclcpp::get_logger("RsuDisseminationRules")) {}

  void initialize(rclcpp::Node * node) override
  {
    cam_rate_ = rclcpp::Duration::from_seconds(
      node->declare_parameter("cam_rate", cam_rate_.seconds()));
    cam_rate_tolerance_ = rclcpp::Duration::from_seconds(
      node->declare_parameter("cam_rate_tolerance", cam_rate_tolerance_.seconds()));
    RCLCPP_INFO(logger_, "CAM rate: %.2fs (%.2fs tolerance)", cam_rate_.seconds(), cam_rate_tolerance_.seconds());
  }

  bool apply(etsi_its_cam_msgs::msg::CAM::SharedPtr) override
  {
    const rclcpp::Time now = rclcpp::Clock().now();
    const rclcpp::Duration elapsed = now - last_transmission_;
    return elapsed + cam_rate_tolerance_ >= cam_rate_;
  }

  void notify(const etsi_its_cam_msgs::msg::CAM::ConstSharedPtr) override
  {
    last_transmission_ = rclcpp::Clock().now();
  }

private:
  rclcpp::Logger logger_;
  rclcpp::Duration cam_rate_ = 1000ms; /*< CAM rate, default is 1 Hz */
  rclcpp::Duration cam_rate_tolerance_ = 10ms; /*< accept a little bit too early CAMs */
  rclcpp::Time last_transmission_;
};

}  // namespace cube_ca_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cube_ca_plugins::RsuDisseminationRules, cube_ca_plugins::DisseminationRules)
