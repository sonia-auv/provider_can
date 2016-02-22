/**
 * \file	bottom_light.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	04/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */


#include <sonia_msgs/CanDevicesProperties.h>
#include "provider_can/devices/bottom_light.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const uint16_t BottomLight::SET_LIGHT_MSG = 0xF00;
const uint16_t BottomLight::SET_LIGHT_DLC = 1;
const std::string BottomLight::NAME = "bottom_light";

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
BottomLight::BottomLight(const CanDispatcher::Ptr &can_dispatcher,
                         const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(lights, bottom_light, can_dispatcher, NAME, nh),
      actual_light_level_(200),
      asked_light_level_(0),
      properties_sent_(false) {
  SetLevel(0);

  bottom_light_pub_ =
      nh->advertise<sonia_msgs::BottomLightMsg>(NAME + "_msgs", 100);

  // sends device's properties if device is present
  if (DevicePresenceCheck()) {
    SendProperties();
    properties_sent_ = true;
  }
}

//------------------------------------------------------------------------------
//
BottomLight::~BottomLight() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void BottomLight::Process() ATLAS_NOEXCEPT {
  std::vector<CanMessage> rx_buffer;
  std::vector<ComputerMessage> pc_messages_buffer;
  bool message_rcvd = false;

  // default value: no ping received
  ros_msg_.ping_rcvd = (uint8_t) false;
  ros_msg_.intensity = actual_light_level_;

  if (DevicePresenceCheck()) {
    // is device is present and properties has not been sent
    if (!properties_sent_) {
      SendProperties();
      properties_sent_ = true;
    }

    // fetching CAN messages
    rx_buffer = FetchMessages();

    // if messages have been received
    if (rx_buffer.size() != 0) {
      // Collects the last message received (previous messages can be bypassed)
      actual_light_level_ = rx_buffer[rx_buffer.size() - 1].data[0];
      bottom_light_pub_.publish(ros_msg_);
    }

    // fetching pc messages (ROS)
    pc_messages_buffer = FetchComputerMessages();

    // loops through all PC messages received
    for (auto &pc_message : pc_messages_buffer) {
      // if messages askes to call set_level function
      switch (pc_message.method_number) {
        case set_level:
          SetLevel((uint8_t)pc_message.parameter_value);
          break;
        case ping_req:
          Ping();
          break;
        case get_properties:
          SendProperties();
          break;
        default:
          break;
      }
    }

    // if ping has been received
    if (GetPingStatus()) {
      ros_msg_.ping_rcvd = true;
      message_rcvd = true;
    }else
      ros_msg_.ping_rcvd = false;

    // if a fault has been received
    const uint8_t *fault = GetFault();
    if (fault != NULL) {
      for (uint8_t i = 0; i < 8; i++) ros_msg_.fault[i] = fault[i];
      message_rcvd = true;
    }else
      for (uint8_t i = 0; i < 8; i++) ros_msg_.fault[i] = ' ';

    if (message_rcvd) bottom_light_pub_.publish(ros_msg_);
  }
}

//------------------------------------------------------------------------------
//
void BottomLight::SetLevel(uint8_t level) ATLAS_NOEXCEPT {
  if (actual_light_level_ != level) {
    PushMessage(SET_LIGHT_MSG, &level, SET_LIGHT_DLC);
    actual_light_level_ = level;
  }
}

//------------------------------------------------------------------------------
//
uint8_t BottomLight::GetLevel() const ATLAS_NOEXCEPT {
  return actual_light_level_;
}

}  // namespace provider_can
