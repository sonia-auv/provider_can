/**
 * \file	mission_switch.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	29/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <sonia_msgs/CanDevicesProperties.h>
#include <provider_can/can/can_dispatcher.h>
#include "provider_can/devices/mission_switch.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const std::string MissionSwitch::NAME = "mission_switch";
const uint16_t MissionSwitch::MISSION_SWITCH_STATE = 0xF00;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
MissionSwitch::MissionSwitch(const CanDispatcher::Ptr &can_dispatcher,
                             const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(interfaces, mission_switch, can_dispatcher, NAME, nh) {
  mission_switch_pub_ =
      nh->advertise<sonia_msgs::MissionSwitchMsg>(NAME + "_msgs", 100);
}

//------------------------------------------------------------------------------
//
MissionSwitch::~MissionSwitch() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void MissionSwitch::ProcessMessages(
    const std::vector<CanMessage> &from_can_rx_buffer,
    const std::vector<ComputerMessage> &from_pc_rx_buffer) ATLAS_NOEXCEPT {
  bool message_rcvd = false;

  // if messages have been received
  // loops through all barometer messages received
  for (auto &can_message : from_can_rx_buffer) {
    switch (can_message.id & DEVICE_MSG_MASK) {
      case MISSION_SWITCH_STATE:
        ros_msg_.state = (bool)can_message.data[0] ^ 0x01;
        message_rcvd = true;
        break;
      default:
        break;
    }
  }

  if (message_rcvd) mission_switch_pub_.publish(ros_msg_);
}

}  // namespace provider_can
