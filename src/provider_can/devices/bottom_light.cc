/**
 * \file	bottom_light.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	04/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <sonia_msgs/BottomLightMsg.h>
#include <sonia_msgs/CanDevicesProperties.h>
#include "provider_can/devices/bottom_light.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const uint16_t BottomLight::SET_LIGHT_MSG = 0xF00;
const uint16_t BottomLight::SET_LIGHT_DLC = 1;
const std::string BottomLight::NAME = "Bottom Light";

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
BottomLight::BottomLight(const CanDispatcher::Ptr &can_dispatcher,
                         const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(lights, bottom_light, can_dispatcher, NAME),
      actual_light_level_(200),
      asked_light_level_(0),
	  properties_sent_(false){
  SetLevel(0);

  bottom_light_pub_ =
      nh->advertise<sonia_msgs::BottomLightMsg>("bottom_light_msgs", 100);

  bottom_light_properties_pub_ =
        nh->advertise<sonia_msgs::CanDevicesProperties>("bottom_light_properties", 100);

  // sends device's properties if device is present
  if(DevicePresenceCheck()){
	  SendProperties();
	  properties_sent_ = true;
  }
}

//------------------------------------------------------------------------------
//
BottomLight::~BottomLight() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void BottomLight::Process() ATLAS_NOEXCEPT {
  std::vector<CanMessage> rx_buffer;
  std::vector<ComputerMessage> pc_messages_buffer;
  sonia_msgs::BottomLightMsg ros_msg;

  // default value: no ping received
  ros_msg.ping_rcvd = (uint8_t) false;
  ros_msg.intensity = actual_light_level_;

  if (DevicePresenceCheck()) {

	// is device is present and properties has not been sent
	if(!properties_sent_)  {
		SendProperties();
		properties_sent_ = true;
	}

    // fetching CAN messages
    rx_buffer = FetchMessages();

    // if messages have been received
    if (rx_buffer.size() != 0) {
      // Collects the last message received (previous messages can be bypassed)
      actual_light_level_ = rx_buffer[rx_buffer.size() - 1].data[0];
      bottom_light_pub_.publish(ros_msg);
    }

    // fetching pc messages (ROS)
    pc_messages_buffer = FetchComputerMessages();

    // loops through all PC messages received
    for (uint8_t i = 0; i < pc_messages_buffer.size(); i++) {
      // if messages askes to call set_level function
      switch (pc_messages_buffer[i].method_number) {
        case set_level:
        	SetLevel((uint8_t)pc_messages_buffer[i].parameter_value);
          break;
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
      ros_msg.ping_rcvd = true;
      bottom_light_pub_.publish(ros_msg);
    }

    // if a fault has been received
    const uint8_t *fault = GetFault();
    if (fault != NULL) {
      for (uint8_t i = 0; i < 8; i++) ros_msg.fault[i] = fault[i];
      bottom_light_pub_.publish(ros_msg);
    }
  }
}

//------------------------------------------------------------------------------
//
void BottomLight::SetLevel(uint8_t level) ATLAS_NOEXCEPT {
  if(actual_light_level_ != level){
	  PushMessage(SET_LIGHT_MSG, &level, SET_LIGHT_DLC);
	  actual_light_level_ = level;
  }
}

//------------------------------------------------------------------------------
//
uint8_t BottomLight::GetLevel() const ATLAS_NOEXCEPT {
  return actual_light_level_;
}

//------------------------------------------------------------------------------
//
void BottomLight::SendProperties() const ATLAS_NOEXCEPT {
	sonia_msgs::CanDevicesProperties ros_msg;
	DeviceProperties properties = GetProperties();

	ros_msg.capabilities = properties.capabilities;
	ros_msg.device_data = properties.device_data;
	ros_msg.firmware_version = properties.firmware_version;
	ros_msg.uc_signature = properties.uc_signature;
	//ros_msg.poll_rate = properties.poll_rate; // unsupported

	bottom_light_properties_pub_.publish(ros_msg);
}

}  // namespace provider_can
