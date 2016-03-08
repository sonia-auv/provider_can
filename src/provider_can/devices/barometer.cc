/**
 * \file	barometer.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	21/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_can/devices/barometer.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const std::string Barometer::NAME = "barometer";

// Receivable CAN messages
const uint16_t Barometer::INTERNAL_PRESS_MSG = 0xF02;
// transmittable CAN messages
const uint16_t Barometer::RELATIVE_PRESS_MSG = 0xF01;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
Barometer::Barometer(const CanDispatcher::Ptr &can_dispatcher,
                     const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(sensors, barometer, can_dispatcher, NAME, nh) {
  barometer_pub_ = nh->advertise<sonia_msgs::BarometerMsg>(NAME + "_msgs", 100);
}

//------------------------------------------------------------------------------
//
Barometer::~Barometer() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void Barometer::ProcessMessages(
    const std::vector<CanMessage> &from_can_rx_buffer,
    const std::vector<ComputerMessage> &from_pc_rx_buffer) ATLAS_NOEXCEPT {
  bool message_rcvd = false;

  // if messages have been received
  // loops through all barometer messages received
  for (auto &can_message : from_can_rx_buffer) {
    switch (can_message.id & DEVICE_MSG_MASK) {
      case INTERNAL_PRESS_MSG:
        ros_msg_.internal_pressure =
            can_message.data[0] + (can_message.data[1] << 8) +
            (can_message.data[2] << 16) + (can_message.data[3] << 24);
        message_rcvd = true;
        break;
      case RELATIVE_PRESS_MSG:
        ros_msg_.ext_relative_pressure =
            can_message.data[0] + (can_message.data[1] << 8) +
            (can_message.data[2] << 16) + (can_message.data[3] << 24);
        message_rcvd = true;
        break;
      default:
        break;
    }
  }

  if (message_rcvd) barometer_pub_.publish(ros_msg_);
}

//------------------------------------------------------------------------------
//

}  // namespace provider_can
