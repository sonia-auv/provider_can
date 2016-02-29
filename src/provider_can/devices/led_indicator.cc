/**
 * \file	barometer.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	21/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */
/**
#include <provider_can/can/can_dispatcher.h>
#include "provider_can/devices/led_indicator.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

  const std::string LedIndicator::NAME = "grabber";

// Receivable CAN messages
  const uint16_t LedIndicator::STATE_MSG = 0xF00;
// transmittable CAN messages
  const uint16_t LedIndicator::PRESS_MSG = 0xF02;
// transmittable CAN messages
  const uint16_t LedIndicator::STARBOARD_TARGET = 0xF01;
// transmittable CAN messages
  const uint16_t LedIndicator::PORT_TARGET = 0xF03;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
  LedIndicator::LedIndicator(const CanDispatcher::Ptr &can_dispatcher,
                       const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(actuators, grabber, can_dispatcher, NAME, nh) {
    grabber_pub_ = nh->advertise<sonia_msgs::GrabberMsg>(NAME + "_msgs", 100);
  }

//------------------------------------------------------------------------------
//
  LedIndicator::~LedIndicator() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void LedIndicator::ProcessMessages(
  const std::vector<CanMessage> &rx_buffer,
  const std::vector<ComputerMessage> &pc_messages_buffer) ATLAS_NOEXCEPT {
  bool message_rcvd = false;

  // if messages have been received
  // loops through all barometer messages received
  for (auto &can_message : rx_buffer) {
    switch (can_message.id & DEVICE_MSG_MASK) {
      case PRESS_MSG:
        ros_msg_.pressure =
        can_message.data[0] + (can_message.data[1] << 8) +
        (can_message.data[2] << 16) + (can_message.data[3] << 24);
        message_rcvd = true;
      break;
      case STATE_MSG:
        ros_msg_.state = (bool)can_message.data[0];
        message_rcvd = true;
        break;
      default:
      break;
    }
  }

  // loops through all PC messages received
  for (auto &pc_message : pc_messages_buffer) {
  switch (pc_message.method_number) {
    case port_set_target:
      PortSetTarget(pc_message.parameter_value);
    break;
    case starboard_set_target:
      StarSetTarget(pc_message.parameter_value);
    break;
    default:
    break;
  }
}

if (message_rcvd) grabber_pub_.publish(ros_msg_);
}

//------------------------------------------------------------------------------
//

void LedIndicator::SetColor(uint8_t color){
  PushMessage(STARBOARD_TARGET, &target, 1);
}

//------------------------------------------------------------------------------
//

void LedIndicator::SetMode(uint8_t mode){
  PushMessage(PORT_TARGET, &target, 1);
}

}  // namespace provider_can*/
