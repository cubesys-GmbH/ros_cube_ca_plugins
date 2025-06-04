#pragma once
#include <cube_ca_plugins/visibility_control.h>

#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <rclcpp/node.hpp>

namespace cube_ca_plugins
{

/**
 * @brief DisseminationRules manage the generation frequency of Cooperative Awareness Messages (CAM).
 *
 * CAM dissemination concept is outlined in
 * - ETSI EN 302 637-2 section 6.1 for release 1
 * - ETSI TS 300 900 section 6.1 for release 2
 */
class CUBE_CA_PLUGINS_PUBLIC DisseminationRules
{
public:
  virtual ~DisseminationRules() = default;

  /**
   * Initialize dissemination rules.
   * 
   * An implementation may want to read parameters from the node.
   * 
   * @param[in] node Node hosting the plugin instance
   */
  virtual void initialize(rclcpp::Node *) = 0;

  /**
   * Check if a CAM should be transmitted based on the current state and rules.
   *
   * This method is called to determine whether a CAM should be sent based on the
   * current conditions and the rules defined in the derived class.
   * The method may modify the CA message if necessary, e.g. removing optional containers.
   *
   * @param[inout] cam The CA message prepared for transmission.
   * @return true if the CAM should be transmitted, false otherwise.
   */
  virtual bool apply(etsi_its_cam_msgs::msg::CAM::SharedPtr cam) = 0;

  /**
   * Notification about a transmitted CAM.
   *
   * This method is called after a CAM has been successfully passed to the BTP layer.
   * It can be used to update internal state or counters related to CAM transmission.
   * 
   * @param[in] cam The actually transmitted CA message.
   */
  virtual void notify(etsi_its_cam_msgs::msg::CAM::ConstSharedPtr) = 0;
};

}  // namespace cube_ca_plugins
