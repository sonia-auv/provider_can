/**
 * \file	thruster.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	21/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_can/devices/thruster.h"
#include <provider_can/can/can_dispatcher.h>
#include <sonia_msgs/CanDevicesProperties.h>

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const uint16_t Thruster::THRUSTER_STATE_MSG = 0xF00;

const uint16_t Thruster::SET_SPEED_MSG = 0xF01;

const std::string Thruster::NAME = "thruster";

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
Thruster::Thruster(const CanDispatcher::Ptr &can_dispatcher,
                   const ros::NodeHandlePtr &nh, std::string thruster_name,
                   Actuators unique_id) ATLAS_NOEXCEPT
    : CanDevice(actuators, unique_id, can_dispatcher,
                NAME + "_" + thruster_name, nh),
      thruster_specific_name_(thruster_name),
      properties_sent_(false) {
  thruster_pub_ = nh->advertise<sonia_msgs::ThrusterMsg>(
      NAME + "_" + thruster_specific_name_ + "_msgs", 100);
}

//------------------------------------------------------------------------------
//
Thruster::~Thruster() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void Thruster::ProcessMessages(
    const std::vector<CanMessage> &from_can_rx_buffer,
    const std::vector<ComputerMessage> &from_pc_rx_buffer) ATLAS_NOEXCEPT {
  bool message_rcvd = false;

  // if messages have been received
  // loops through all thruster messages received
  for (auto &can_message : from_can_rx_buffer) {
    switch (can_message.id & DEVICE_MSG_MASK) {
      case THRUSTER_STATE_MSG:
        ros_msg.factory_infos = can_message.data[0];
        ros_msg.current = can_message.data[1];
        ros_msg.speed = can_message.data[2] + (can_message.data[3] << 8);
        ros_msg.temperature = can_message.data[4];
        ros_msg.i2c_fault_number = can_message.data[5];
        message_rcvd = true;
        break;
      default:
        break;
    }
  }

  // loops through all PC messages received
  for (auto &pc_message : from_pc_rx_buffer) {
    switch (pc_message.method_number) {
      case set_speed:
        SetSpeed((int8_t)pc_message.parameter_value);
        break;
      default:
        break;
    }
  }

  if (message_rcvd) thruster_pub_.publish(ros_msg);
}

//------------------------------------------------------------------------------
//

void Thruster::SetSpeed(int8_t speed) const ATLAS_NOEXCEPT {
  if (speed > 100) speed = 0;

  if (speed < -100) speed = 0;

  uint8_t absolute_speed = (uint8_t)speed;

  PushMessage(SET_SPEED_MSG, &absolute_speed, 1);
}

}  // namespace provider_can
