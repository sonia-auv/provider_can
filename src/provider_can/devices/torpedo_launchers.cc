/**
 * \file	torpedo_launchers.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	11/03/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_can/devices/torpedo_launchers.h"
#include <provider_can/can/can_dispatcher.h>

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const std::string TorpedoLaunchers::NAME = "torpedo_launchers";

// receivable CAN messages
const uint16_t TorpedoLaunchers::PRESS_MSG = 0xF01;

// transmittable CAN messages
const uint16_t TorpedoLaunchers::LAUNCH_MSG = 0xF00;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
TorpedoLaunchers::TorpedoLaunchers(const CanDispatcher::Ptr &can_dispatcher,
                                   const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(markers, launcher, can_dispatcher, NAME, nh) {
  torpedo_launchers_pub_ =
      nh->advertise<sonia_msgs::TorpedoLaunchersMsg>(NAME + "_msgs", 100);
}

//------------------------------------------------------------------------------
//
TorpedoLaunchers::~TorpedoLaunchers() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void TorpedoLaunchers::ProcessMessages(
    const std::vector<CanMessage> &from_can_rx_buffer,
    const std::vector<ComputerMessage> &from_pc_rx_buffer) ATLAS_NOEXCEPT {
  bool message_rcvd = false;

  // if messages have been received
  // loops through all grabber messages received
  for (auto &can_message : from_can_rx_buffer) {
    switch (can_message.id) {
      case PRESS_MSG:
        ros_msg_.pressure = can_message.data[0] + (can_message.data[1] << 8) +
                            (can_message.data[2] << 16) +
                            (can_message.data[3] << 24);
        message_rcvd = true;
        break;
      default:
        break;
    }
  }

  // loops through all PC messages received
  for (auto &pc_message : from_pc_rx_buffer) {
    switch (pc_message.method_number) {
      case launch:
        Launch();
        break;
      default:
        break;
    }
  }

  if (message_rcvd) torpedo_launchers_pub_.publish(ros_msg_);
}

//------------------------------------------------------------------------------
//

void TorpedoLaunchers::Launch() const ATLAS_NOEXCEPT {
  uint8_t *msg = nullptr;
  PushMessage(LAUNCH_MSG, msg, 0);
}

}  // namespace provider_can
