#include "cube_ca_plugins/cam_utils.hpp"

#include <cmath>

namespace cube_ca_plugins
{

etsi_its_cam_msgs::msg::HeadingValue get_heading(const etsi_its_cam_msgs::msg::CAM & msg)
{
  using etsi_its_cam_msgs::msg::HighFrequencyContainer;

  auto & hfc = msg.cam.cam_parameters.high_frequency_container;
  if (hfc.choice == HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY) {
    return hfc.basic_vehicle_container_high_frequency.heading.heading_value;
  }

  etsi_its_cam_msgs::msg::HeadingValue na;
  na.value = etsi_its_cam_msgs::msg::HeadingValue::UNAVAILABLE;
  return na;
}

etsi_its_cam_msgs::msg::SpeedValue get_speed(const etsi_its_cam_msgs::msg::CAM & msg)
{
  using etsi_its_cam_msgs::msg::HighFrequencyContainer;

  auto & hfc = msg.cam.cam_parameters.high_frequency_container;
  if (hfc.choice == HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY) {
    return hfc.basic_vehicle_container_high_frequency.speed.speed_value;
  }

  etsi_its_cam_msgs::msg::SpeedValue na;
  na.value = etsi_its_cam_msgs::msg::SpeedValue::UNAVAILABLE;
  return na;
}

bool valid_position(const etsi_its_cam_msgs::msg::ReferencePosition & pos)
{
  using etsi_its_cam_msgs::msg::Latitude;
  using etsi_its_cam_msgs::msg::Longitude;
  if (pos.latitude.value >= Latitude::MIN && pos.latitude.value <= Latitude::MAX && pos.longitude.value >= Longitude::MIN && pos.longitude.value <= Longitude::MAX) {
    return true;
  } else {
    return false;
  }
}

double distance_haversine(double lat1, double lon1, double lat2, double lon2)
{
  static const double r = 6378137.0;  // equatorial earth radius of WGS84
  static const double deg2rad = std::atan(1.0) * 4.0 / 180.0;  // conversion factor from degree to radian

  // convert degrees to radians
  lat1 *= deg2rad;
  lon1 *= deg2rad;
  lat2 *= deg2rad;
  lon2 *= deg2rad;

  const double i = std::sin(0.5 * (lat2 - lat1));
  const double j = std::sin(0.5 * (lon2 - lon1));
  return 2.0 * r * std::asin(std::sqrt(i * i + std::cos(lat1) * std::cos(lat2) * j * j));
}

}  // namespace cube_ca_plugins