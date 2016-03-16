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
#include <sonia_msgs/CanDevicesProperties.h>

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const uint16_t PowerSupply::KILL_STATE_MSG = 0xF04;
const uint16_t PowerSupply::VOLT1_MSG = 0xF10;
const uint16_t PowerSupply::VOLT2_MSG = 0xF11;
const uint16_t PowerSupply::VOLT3_MSG = 0xF12;
const uint16_t PowerSupply::CURR1_MSG = 0xF20;
const uint16_t PowerSupply::CURR2_MSG = 0xF21;
const uint16_t PowerSupply::CURR3_MSG = 0xF22;
const uint16_t PowerSupply::STATES1_MSG = 0xF30;
const uint16_t PowerSupply::STATES2_MSG = 0xF31;

const uint16_t PowerSupply::PC_RST_MSG = 0xF01;
const uint16_t PowerSupply::SET_CHANNEL_MSG = 0xF03;
const uint16_t PowerSupply::REMOTE_KILL_MSG = 0xF02;

const std::string PowerSupply::NAME = "power_supply";

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
PowerSupply::PowerSupply(const CanDispatcher::Ptr &can_dispatcher,
                         const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(power, power_distribution, can_dispatcher, NAME, nh) {
  power_supply_pub_ =
      nh->advertise<sonia_msgs::PowerSupplyMsg>(NAME + "_msgs", 100);
}

//------------------------------------------------------------------------------
//
PowerSupply::~PowerSupply() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void PowerSupply::ProcessMessages(
    const std::vector<CanMessage> &from_can_rx_buffer,
    const std::vector<ComputerMessage> &from_pc_rx_buffer) ATLAS_NOEXCEPT {
  bool message_rcvd = false;

  // if messages have been received
  // loops through all power supply messages received
  for (auto &can_message : from_can_rx_buffer) {
    switch (can_message.id & DEVICE_MSG_MASK) {
      case KILL_STATE_MSG:
        ros_msg_.kill_switch_state = (bool)can_message.data[0];
        message_rcvd = true;
        break;
      case VOLT1_MSG:
        ros_msg_.volt_bus1_voltage =
            can_message.data[0] + (can_message.data[1] << 8);
        ros_msg_.volt_bus2_voltage =
            can_message.data[2] + (can_message.data[3] << 8);
        ros_msg_.pc_voltage = can_message.data[4] + (can_message.data[5] << 8);
        ros_msg_.motor_bus1_voltage =
            can_message.data[6] + (can_message.data[7] << 8);
        message_rcvd = true;
        break;
      case VOLT2_MSG:
        ros_msg_.motor_bus2_voltage =
            can_message.data[0] + (can_message.data[1] << 8);
        ros_msg_.motor_bus3_voltage =
            can_message.data[2] + (can_message.data[3] << 8);
        ros_msg_.dvl_voltage = can_message.data[4] + (can_message.data[5] << 8);
        ros_msg_.actuator_bus_voltage =
            can_message.data[6] + (can_message.data[7] << 8);
        message_rcvd = true;
        break;
      case VOLT3_MSG:
        ros_msg_.light_voltage =
            can_message.data[0] + (can_message.data[1] << 8);
        message_rcvd = true;
        break;
      case CURR1_MSG:
        ros_msg_.volt_bus1_current =
            can_message.data[0] + (can_message.data[1] << 8);
        ros_msg_.volt_bus2_current =
            can_message.data[2] + (can_message.data[3] << 8);
        ros_msg_.pc_current = can_message.data[4] + (can_message.data[5] << 8);
        ros_msg_.motor_bus1_current =
            can_message.data[6] + (can_message.data[7] << 8);
        message_rcvd = true;
        break;
      case CURR2_MSG:
        ros_msg_.motor_bus2_current =
            can_message.data[0] + (can_message.data[1] << 8);
        ros_msg_.motor_bus3_current =
            can_message.data[2] + (can_message.data[3] << 8);
        ros_msg_.dvl_current = can_message.data[4] + (can_message.data[5] << 8);
        ros_msg_.actuator_bus_current =
            can_message.data[6] + (can_message.data[7] << 8);
        message_rcvd = true;
        break;
      case CURR3_MSG:
        ros_msg_.light_current =
            can_message.data[0] + (can_message.data[1] << 8);
        message_rcvd = true;
        break;
      case STATES1_MSG:
        ros_msg_.volt_bus1_state = (bool)can_message.data[0];
        ros_msg_.volt_bus2_state = (bool)can_message.data[1];
        ros_msg_.pc_state = (bool)can_message.data[2];
        ros_msg_.motor_bus1_state = (bool)can_message.data[3];
        ros_msg_.motor_bus2_state = (bool)can_message.data[4];
        ros_msg_.motor_bus3_state = (bool)can_message.data[5];
        ros_msg_.dvl_state = (bool)can_message.data[6];
        ros_msg_.actuator_bus_state = (bool)can_message.data[7];
        message_rcvd = true;
        break;
      case STATES2_MSG:
        ros_msg_.light_state = (bool)can_message.data[0];
        message_rcvd = true;
        break;
      default:
        break;
    }
  }

  // loops through all PC messages received
  for (auto &pc_message : from_pc_rx_buffer) {
    switch (pc_message.method_number) {
      case pc_reset:
        PcReset();
        break;
      case remote_kill:
        RemoteKill((uint8_t)pc_message.parameter_value);
        break;
      case set_channel:
        SetChannel((uint8_t)pc_message.parameter_value);
        break;
      case clr_channel:
        ClrChannel((uint8_t)pc_message.parameter_value);
        break;
      default:
        break;
    }
  }

  if (message_rcvd) power_supply_pub_.publish(ros_msg_);
}

//------------------------------------------------------------------------------
//

void PowerSupply::PcReset() const ATLAS_NOEXCEPT {
  PushMessage(PC_RST_MSG, 0, 0);
}

//------------------------------------------------------------------------------
//

void PowerSupply::RemoteKill(uint8_t state) const ATLAS_NOEXCEPT {
  PushMessage(REMOTE_KILL_MSG, &state, 1);
}

//------------------------------------------------------------------------------
//

void PowerSupply::SetChannel(uint8_t channel) const ATLAS_NOEXCEPT {
  uint8_t can_msg[2] = {channel, 0};
  PushMessage(SET_CHANNEL_MSG, can_msg, 2);
}

//------------------------------------------------------------------------------
//

void PowerSupply::ClrChannel(uint8_t channel) const ATLAS_NOEXCEPT {
  uint8_t can_msg[2] = {channel, 1};
  PushMessage(SET_CHANNEL_MSG, can_msg, 2);
}
}  // namespace provider_can
