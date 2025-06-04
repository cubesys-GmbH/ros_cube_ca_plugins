#pragma once
#include <etsi_its_cam_msgs/msg/cam.hpp>

namespace cube_ca_plugins
{

/**
 * Get heading value from CAM message's high frequency container.
 * 
 * @param msg CAM message to extract heading from
 * @return HeadingValue from the high frequency container, or UNAVAILABLE if not present.
 */
etsi_its_cam_msgs::msg::HeadingValue get_heading(const etsi_its_cam_msgs::msg::CAM & msg);

/**
 * Get speed value from CAM message's high frequency container.
 * 
 * @param msg CAM message to extract speed from
 * @return SpeedValue from the high frequency container, or UNAVAILABLE if not present.
 */
etsi_its_cam_msgs::msg::SpeedValue get_speed(const etsi_its_cam_msgs::msg::CAM & msg);

/**
 * Check if ReferencePosition is valid.
 * 
 * @param pos ReferencePosition to check
 * @return true if position is valid, false otherwise.
 */
bool valid_position(const etsi_its_cam_msgs::msg::ReferencePosition & pos);

/**
 * Calculate the Haversine distance between two geographical points.
 * 
 * @param lat1 Latitude of the first point in degrees
 * @param lon1 Longitude of the first point in degrees
 * @param lat2 Latitude of the second point in degrees
 * @param lon2 Longitude of the second point in degrees
 * @return Distance in meters between the two points.
 */
double distance_haversine(double lat1, double lon1, double lat2, double lon2);

template <typename T>
T abs_diff(T a, T b)
{
  if (a > b) {
    return a - b;
  } else {
    return b - a;
  }
}

}  // namespace cube_ca_plugins
