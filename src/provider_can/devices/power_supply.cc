/**
 * \file	power_supply.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	16/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */


#include <sonia_msgs/CanDevicesProperties.h>
#include "provider_can/devices/power_supply.h"

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

const std::string PowerSupply::NAME = "Power Supply";

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
PowerSupply::PowerSupply(const CanDispatcher::Ptr &can_dispatcher,
                         const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(power, power_distribution, can_dispatcher, NAME),
	  properties_sent_(false){
   power_supply_pub_ =
    nh->advertise<sonia_msgs::PowerSupplyMsg>("power_supply_msgs", 100);
   power_supply_properties_pub_ =
    nh->advertise<sonia_msgs::CanDevicesProperties>("power_supply_properties", 100);

   // sends device's properties if device is present
   if(DevicePresenceCheck()){
	   SendProperties();
	   properties_sent_ = true;
   }

}

//------------------------------------------------------------------------------
//
PowerSupply::~PowerSupply() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void PowerSupply::Process() ATLAS_NOEXCEPT {
  std::vector<CanMessage> rx_buffer;
  std::vector<ComputerMessage> pc_messages_buffer;
  bool message_rcvd = false;

  if (DevicePresenceCheck()) {

	// is device is present and properties has not been sent
	if(!properties_sent_)  {
		SendProperties();
		properties_sent_ = true;
	}

    // fetching CAN messages
    rx_buffer = FetchMessages();

    // if messages have been received
    // loops through all power supply messages received
	for (uint8_t i = 0; i < rx_buffer.size(); i++) {
	  switch (rx_buffer[i].id & DEVICE_MSG_MASK) {
		case KILL_STATE_MSG:
		  ros_msg.kill_switch_state = (bool)rx_buffer[i].data[0];
		  message_rcvd = true;
		  break;
		case VOLT1_MSG:
		  ros_msg.volt_bus1_voltage = rx_buffer[i].data[0]
									+ (rx_buffer[i].data[1] << 8);
		  ros_msg.volt_bus2_voltage = rx_buffer[i].data[2]
									+ (rx_buffer[i].data[3] << 8);
		  ros_msg.pc_voltage = rx_buffer[i].data[4]
									+ (rx_buffer[i].data[5] << 8);
		  ros_msg.motor_bus1_voltage = rx_buffer[i].data[6]
									+ (rx_buffer[i].data[7] << 8);
		  message_rcvd = true;
		  break;
		case VOLT2_MSG:
		  ros_msg.motor_bus2_voltage = rx_buffer[i].data[0]
									+ (rx_buffer[i].data[1] << 8);
		  ros_msg.motor_bus3_voltage = rx_buffer[i].data[2]
									+ (rx_buffer[i].data[3] << 8);
		  ros_msg.dvl_voltage = rx_buffer[i].data[4]
									+ (rx_buffer[i].data[5] << 8);
		  ros_msg.actuator_bus_voltage = rx_buffer[i].data[6]
									+ (rx_buffer[i].data[7] << 8);
		  message_rcvd = true;
		  break;
		case VOLT3_MSG:
		  ros_msg.light_voltage = rx_buffer[i].data[0]
									+ (rx_buffer[i].data[1] << 8);
		  message_rcvd = true;
		  break;
		case CURR1_MSG:
		  ros_msg.volt_bus1_current = rx_buffer[i].data[0]
									+ (rx_buffer[i].data[1] << 8);
		  ros_msg.volt_bus2_current = rx_buffer[i].data[2]
									+ (rx_buffer[i].data[3] << 8);
		  ros_msg.pc_current = rx_buffer[i].data[4]
									+ (rx_buffer[i].data[5] << 8);
		  ros_msg.motor_bus1_current = rx_buffer[i].data[6]
									+ (rx_buffer[i].data[7] << 8);
		  message_rcvd = true;
		  break;
		case CURR2_MSG:
		  ros_msg.motor_bus2_current = rx_buffer[i].data[0]
									+ (rx_buffer[i].data[1] << 8);
		  ros_msg.motor_bus3_current = rx_buffer[i].data[2]
									+ (rx_buffer[i].data[3] << 8);
		  ros_msg.dvl_current = rx_buffer[i].data[4]
									+ (rx_buffer[i].data[5] << 8);
		  ros_msg.actuator_bus_current = rx_buffer[i].data[6]
									+ (rx_buffer[i].data[7] << 8);
		  message_rcvd = true;
		  break;
		case CURR3_MSG:
		  ros_msg.light_current = rx_buffer[i].data[0]
									+ (rx_buffer[i].data[1] << 8);
		  message_rcvd = true;
		  break;
		case STATES1_MSG:
		  ros_msg.volt_bus1_state = (bool)rx_buffer[i].data[0];
		  ros_msg.volt_bus2_state = (bool)rx_buffer[i].data[1];
		  ros_msg.pc_state = (bool)rx_buffer[i].data[2];
		  ros_msg.motor_bus1_state = (bool)rx_buffer[i].data[3];
		  ros_msg.motor_bus2_state = (bool)rx_buffer[i].data[4];
		  ros_msg.motor_bus3_state = (bool)rx_buffer[i].data[5];
		  ros_msg.dvl_state = (bool)rx_buffer[i].data[6];
		  ros_msg.actuator_bus_state = (bool)rx_buffer[i].data[7];
		  message_rcvd = true;
		  break;
		case STATES2_MSG:
		  ros_msg.light_state = (bool)rx_buffer[i].data[0];
		  message_rcvd = true;
		  break;
		default:
		  break;
	  }
	}

    // fetching pc messages (ROS)
    pc_messages_buffer = FetchComputerMessages();

    // loops through all PC messages received
    for (uint8_t i = 0; i < pc_messages_buffer.size(); i++) {
      switch (pc_messages_buffer[i].method_number) {
        case ping_req:
          Ping();
          break;
        case get_properties:
		  SendProperties();
		  break;
        case pc_reset:
          PcReset();
		  break;
		case remote_kill:
		  RemoteKill((uint8_t)pc_messages_buffer[i].parameter_value);
		  break;
		case set_channel:
		  SetChannel((uint8_t)pc_messages_buffer[i].parameter_value);
		  break;
		case clr_channel:
		  ClrChannel((uint8_t)pc_messages_buffer[i].parameter_value);
		  break;
        default:
          break;
      }
    }

    // if ping has been received
    if (GetPingStatus()) {
       ros_msg.ping_rcvd = true;
       message_rcvd = true;
    }
    else
       ros_msg.ping_rcvd = false;

    // if a fault has been received
    const uint8_t *fault = GetFault();
    if (fault != NULL) {
       for (uint8_t i = 0; i < 8; i++) ros_msg.fault[i] = fault[i];
       message_rcvd = true;
    }
    else
       for (uint8_t i = 0; i < 8; i++) ros_msg.fault[i] = ' ';

    if(message_rcvd)
    		power_supply_pub_.publish(ros_msg);
  }
}

//------------------------------------------------------------------------------
//

void PowerSupply::SendProperties() const ATLAS_NOEXCEPT {
	sonia_msgs::CanDevicesProperties ros_msg;
	DeviceProperties properties = GetProperties();

	ros_msg.capabilities = properties.capabilities;
	ros_msg.device_data = properties.device_data;
	ros_msg.firmware_version = properties.firmware_version;
	ros_msg.uc_signature = properties.uc_signature;
	//ros_msg.poll_rate = properties.poll_rate; // unsupported

	power_supply_properties_pub_.publish(ros_msg);
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
	uint8_t can_msg[2] = {channel,1};
	PushMessage(SET_CHANNEL_MSG, can_msg, 2);
}

//------------------------------------------------------------------------------
//

void PowerSupply::ClrChannel(uint8_t channel) const ATLAS_NOEXCEPT {
	uint8_t can_msg[2] = {channel,0};
	PushMessage(SET_CHANNEL_MSG, can_msg, 2);
}
}  // namespace provider_can
