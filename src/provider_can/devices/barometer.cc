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
const uint16_t Barometer::INTERNAL_PRESS_MSG = 0xF01;
// transmittable CAN messages
const uint16_t Barometer::RELATIVE_PRESS_MSG = 0xF02;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
Barometer::Barometer(const CanDispatcher::Ptr &can_dispatcher,
                     const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(sensors, barometer, can_dispatcher, NAME, nh),
      properties_sent_(false) {
  barometer_pub_ = nh->advertise<sonia_msgs::BarometerMsg>(NAME + "_msgs", 100);

  // sends device's properties if device is present
  if (DevicePresenceCheck()) {
    SendProperties();
    properties_sent_ = true;
  }
}

//------------------------------------------------------------------------------
//
Barometer::~Barometer() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void Barometer::Process() ATLAS_NOEXCEPT {
  std::vector<CanMessage> rx_buffer;
  std::vector<ComputerMessage> pc_messages_buffer;
  bool message_rcvd = false;

  // default value: no ping received
  ros_msg_.ping_rcvd = (uint8_t) false;

  if (DevicePresenceCheck()) {
    // is device is present and properties has not been sent
    if (!properties_sent_) {
      SendProperties();
      properties_sent_ = true;
    }

    // fetching CAN messages
    rx_buffer = FetchMessages();

    // if messages have been received
    // loops through all barometer messages received
    for (auto &can_message : rx_buffer) {
      switch (can_message.id & DEVICE_MSG_MASK) {
        case INTERNAL_PRESS_MSG:
          ros_msg_.internal_pressure = can_message.data[0] + can_message.data[1]
                                      << 8 + can_message.data[2]
                                      << 16 + can_message.data[3] << 24;
          message_rcvd = true;
          break;
        case RELATIVE_PRESS_MSG:
          ros_msg_.ext_relative_pressure =
              can_message.data[0] + can_message.data[1]
              << 8 + can_message.data[2] << 16 + can_message.data[3] << 24;
          message_rcvd = true;
          break;
        default:
          break;
      }
    }

    // fetching pc messages (ROS)
    pc_messages_buffer = FetchComputerMessages();

    // loops through all PC messages received
    for (auto &pc_message : pc_messages_buffer) {
      switch (pc_message.method_number) {
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
    } else
    ros_msg_.ping_rcvd = false;

    // if a fault has been received
    const uint8_t *fault = GetFault();
    if (fault != NULL) {
      for (uint8_t i = 0; i < 8; i++) ros_msg_.fault[i] = fault[i];
      message_rcvd = true;
    }else
      for (uint8_t i = 0; i < 8; i++) ros_msg_.fault[i] = ' ';

    if (message_rcvd) barometer_pub_.publish(ros_msg_);
  }
}

//------------------------------------------------------------------------------
//

}  // namespace provider_can
