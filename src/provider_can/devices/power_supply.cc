/**
 * \file	power_supply.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	16/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_can/devices/power_supply.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const std::string PowerSupply::NAME = "Power Supply";

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
PowerSupply::PowerSupply(const CanDispatcher::Ptr &can_dispatcher,
                         const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(power, power_distribution, can_dispatcher, NAME) {

	//power_supply_pub_ =
    //  nh->advertise<sonia_msgs::BottomLightMsg>("power_supply_msgs", 100);
}

//------------------------------------------------------------------------------
//
PowerSupply::~PowerSupply() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void PowerSupply::Process()ATLAS_NOEXCEPT {
  std::vector<CanMessage> rx_buffer;
  std::vector<ComputerMessage> pc_messages_buffer;

  if (DevicePresenceCheck()) {
    // fetching CAN messages
    rx_buffer = FetchMessages();

    // if messages have been received


    // fetching pc messages (ROS)
    pc_messages_buffer = FetchComputerMessages();

    // loops through all PC messages received
    for (uint8_t i = 0; i < pc_messages_buffer.size(); i++) {
      // if messages askes to call set_level function
      switch (pc_messages_buffer[i].method_number) {
        case CommonMethods::ping_req:
          Ping();
          break;
        default:
          break;
      }
    }

    // if ping has been received
    if (GetPingStatus()) {
      //ros_msg.ping_rcvd = true;
      //bottom_light_pub_.publish(ros_msg);
    }

    // if a fault has been received
    const uint8_t *fault = GetFault();
    if (fault != NULL) {
      //for (uint8_t i = 0; i < 8; i++) ros_msg.fault[i] = fault[i];
      //bottom_light_pub_.publish(ros_msg);
    }
  }
}

//------------------------------------------------------------------------------
//
}  // namespace provider_can
