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
const uint8_t BottomLight::SET_LIGHT_DLC = 1;
const std::string BottomLight::NAME = "bottom_light";

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
BottomLight::BottomLight(const CanDispatcher::Ptr &can_dispatcher,
                         const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(lights, bottom_light, can_dispatcher, NAME, nh),
      actual_light_level_(200),
      asked_light_level_(0) {
  SetLevel(0);

  bottom_light_pub_ =
      nh->advertise<sonia_msgs::BottomLightMsg>(NAME + "_msgs", 100);
}

//------------------------------------------------------------------------------
//
BottomLight::~BottomLight() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void BottomLight::ProcessMessages(
    const std::vector<CanMessage> &from_can_rx_buffer,
    const std::vector<ComputerMessage> &from_pc_rx_buffer) ATLAS_NOEXCEPT {
  bool message_rcvd = false;

  ros_msg_.intensity = actual_light_level_;

  // if messages have been received
  if (from_can_rx_buffer.size() != 0) {
    // Collects the last message received (previous messages can be bypassed)
    actual_light_level_ =
        from_can_rx_buffer[from_can_rx_buffer.size() - 1].data[0];
    message_rcvd = true;
  }

  // loops through all PC messages received
  for (auto &pc_message : from_pc_rx_buffer) {
    // if messages askes to call set_level function
    switch (pc_message.method_number) {
      case set_level:
        SetLevel((uint8_t)pc_message.parameter_value);
        break;
      default:
        break;
    }
  }

  if (message_rcvd) bottom_light_pub_.publish(ros_msg_);
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
