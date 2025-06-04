#include <chrono>

#include "cube_ca_plugins/cam_utils.hpp"
#include "cube_ca_plugins/dissemination_rules.hpp"
#include "cube_ca_plugins/visibility_control.h"

namespace cube_ca_plugins
{

using namespace std::chrono_literals;

/**
 * CAM dissemination rules according to EN 302 637-2 v1.4.1 section 6.1.3
 */
class CUBE_CA_PLUGINS_PUBLIC VehicleDisseminationRules : public DisseminationRules
{
public:
  VehicleDisseminationRules() : logger_(rclcpp::get_logger("VehicleDisseminationRules")) {}

  void initialize(rclcpp::Node * node) override
  {
    cam_rate_min_ = rclcpp::Duration::from_seconds(node->declare_parameter("cam_rate_min", cam_rate_min_.seconds()));
    cam_rate_max_ = rclcpp::Duration::from_seconds(node->declare_parameter("cam_rate_max", cam_rate_max_.seconds()));
    cam_rate_counter_max_ = node->declare_parameter("cam_rate_counter_max", static_cast<int>(cam_rate_counter_max_));
    cam_rate_tolerance_ =
      rclcpp::Duration::from_seconds(node->declare_parameter("cam_rate_tolerance", cam_rate_tolerance_.seconds()));

    cam_rate_ = cam_rate_max_;
    cam_rate_counter_ = cam_rate_counter_max_;
    RCLCPP_INFO(logger_, "CAM rate: %.2fs to %.2fs", cam_rate_min_.seconds(), cam_rate_max_.seconds());
    RCLCPP_INFO(logger_, "CAM counter maximum: %d", cam_rate_counter_max_);

    delta_heading_ = node->declare_parameter("delta_heading", delta_heading_);
    delta_speed_ = node->declare_parameter("delta_speed", delta_speed_);
    delta_dist_ = node->declare_parameter("delta_dist", delta_dist_);

    low_frequency_interval_ = rclcpp::Duration::from_seconds(
      node->declare_parameter("low_frequency_interval", low_frequency_interval_.seconds()));
    special_vehicle_interval_ = rclcpp::Duration::from_seconds(
      node->declare_parameter("special_vehicle_interval", special_vehicle_interval_.seconds()));
  }

  bool apply(etsi_its_cam_msgs::msg::CAM::SharedPtr cam) override
  {
    const rclcpp::Time now = rclcpp::Clock().now();
    const rclcpp::Duration elapsed = now - last_transmission_;

    if (checkTriggeringConditions(cam, elapsed)) {
      manageOptionalContainers(cam, now);
      return true;
    } else {
      return false;
    }
  }

  void notify(const etsi_its_cam_msgs::msg::CAM::ConstSharedPtr cam) override
  {
    last_transmission_ = rclcpp::Clock().now();
    if (cam->cam.cam_parameters.low_frequency_container_is_present) {
      last_low_frequency_container_ = last_transmission_;
    }
    if (cam->cam.cam_parameters.special_vehicle_container_is_present) {
      last_special_vehicle_container_ = last_transmission_;
    }
    transmitted_cam_ = cam;
  }

private:
  CUBE_CA_PLUGINS_LOCAL bool checkTriggeringConditions(
    const etsi_its_cam_msgs::msg::CAM::ConstSharedPtr & cam, rclcpp::Duration elapsed)
  {
    if (!cam) {
      // nothing to transmit
      return false;
    } else if (!transmitted_cam_) {
      // first ever CAM, go for it!
      return true;
    } else if (checkHeading(cam) || checkSpeed(cam) || checkPosition(cam)) {
      cam_rate_ = std::min(elapsed, cam_rate_max_);
      cam_rate_counter_ = 1;
      RCLCPP_DEBUG(logger_, "CAM rate is %.2lfs", cam_rate_.seconds());
      return true;
    } else if (elapsed + cam_rate_tolerance_ >= cam_rate_) {
      // T_GenCamDcc limit is enforced by caller, so we can skip it here
      if (cam_rate_counter_ >= cam_rate_counter_max_) {
        if (cam_rate_ != cam_rate_max_) {
          RCLCPP_DEBUG(
            logger_, "CAM rate is reset to %.2lfs after %d transmissions",
            cam_rate_max_.seconds(), cam_rate_counter_max_);
        }
        cam_rate_ = cam_rate_max_;
      } else {
        ++cam_rate_counter_;
      }
      return true;
    } else {
      // wait a little bit longer before next CAM transmission
      return false;
    }
  }

  CUBE_CA_PLUGINS_LOCAL bool checkHeading(const etsi_its_cam_msgs::msg::CAM::ConstSharedPtr & cam) const
  {
    assert(cam);
    assert(transmitted_cam_);

    using etsi_its_cam_msgs::msg::HeadingValue;
    const HeadingValue prev_cam_heading = get_heading(*transmitted_cam_);
    const HeadingValue curr_cam_heading = get_heading(*cam);

    if (prev_cam_heading.value < HeadingValue::UNAVAILABLE && curr_cam_heading.value < HeadingValue::UNAVAILABLE) {
      static_assert(
        etsi_its_cam_msgs::msg::HeadingValue::WGS84_EAST == 900, "HeadingValue is expected to be tenth of degrees");
      return abs_diff(prev_cam_heading.value, curr_cam_heading.value) > delta_heading_ * 10;
    } else {
      return false;
    }
  }

  CUBE_CA_PLUGINS_LOCAL bool checkSpeed(const etsi_its_cam_msgs::msg::CAM::ConstSharedPtr & cam) const
  {
    assert(cam);
    assert(transmitted_cam_);

    using etsi_its_cam_msgs::msg::SpeedValue;
    const SpeedValue prev_cam_speed = get_speed(*transmitted_cam_);
    const SpeedValue curr_cam_speed = get_speed(*cam);

    if (prev_cam_speed.value != SpeedValue::UNAVAILABLE && curr_cam_speed.value != SpeedValue::UNAVAILABLE) {
      const unsigned delta_speed = std::round(delta_speed_ * 100) * SpeedValue::ONE_CENTIMETER_PER_SEC;
      return abs_diff(prev_cam_speed.value, curr_cam_speed.value) > delta_speed;
    } else {
      return false;
    }
  }

  CUBE_CA_PLUGINS_LOCAL bool checkPosition(const etsi_its_cam_msgs::msg::CAM::ConstSharedPtr & cam) const
  {
    assert(cam);
    assert(transmitted_cam_);

    auto prior_pos = transmitted_cam_->cam.cam_parameters.basic_container.reference_position;
    auto now_pos = cam->cam.cam_parameters.basic_container.reference_position;

    if (!valid_position(prior_pos) || !valid_position(now_pos)) {
      return false;
    }

    const double dist = distance_haversine(
      1e-7 * prior_pos.latitude.value, 1e-7 * prior_pos.longitude.value, 1e-7 * now_pos.latitude.value,
      1e-7 * now_pos.longitude.value);
    return dist > delta_dist_;
  }

  CUBE_CA_PLUGINS_LOCAL void manageOptionalContainers(etsi_its_cam_msgs::msg::CAM::SharedPtr & cam, rclcpp::Time now)
  {
    // omit low frequency container if it is too frequent
    if (last_low_frequency_container_ + low_frequency_interval_ > now) {
      cam->cam.cam_parameters.low_frequency_container_is_present = false;
    }
    // omit special vehicle container if it is too frequent
    if (last_special_vehicle_container_ + special_vehicle_interval_ > now) {
      cam->cam.cam_parameters.special_vehicle_container_is_present = false;
    }
  }

  rclcpp::Logger logger_;
  etsi_its_cam_msgs::msg::CAM::ConstSharedPtr transmitted_cam_; /*< last transmitted CAM */
  double delta_heading_ = 4.0; /*< degree */
  double delta_speed_ = 0.5; /*< meter per second */
  double delta_dist_ = 4.0; /*< meter */
  rclcpp::Duration cam_rate_min_ = 100ms; /*< T_GenCamMin */
  rclcpp::Duration cam_rate_max_ = 1000ms; /*< T_GenCamMax */
  rclcpp::Duration cam_rate_ = cam_rate_max_; /*< T_GenCam */
  rclcpp::Duration cam_rate_tolerance_ = 10ms; /*< accept a little bit too early CAMs */
  unsigned cam_rate_counter_max_ = 3;
  unsigned cam_rate_counter_ = cam_rate_counter_max_; /*< N_GenCam */
  rclcpp::Time last_transmission_;
  rclcpp::Time last_low_frequency_container_;
  rclcpp::Time last_special_vehicle_container_;
  rclcpp::Duration low_frequency_interval_ = 500ms;
  rclcpp::Duration special_vehicle_interval_ = 500ms;
};

}  // namespace cube_ca_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cube_ca_plugins::VehicleDisseminationRules, cube_ca_plugins::DisseminationRules)
