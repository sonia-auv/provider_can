/**
 * \file	thruster.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	21/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */


#include <sonia_msgs/CanDevicesProperties.h>
#include <provider_can/can/can_dispatcher.h>
#include "provider_can/devices/thruster.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const uint16_t Thruster::THRUSTER_STATE_MSG = 0xF00;

const uint16_t Thruster::SET_SPEED_MSG = 0xF01;

const std::string Thruster::NAME = "_thruster";

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
	Thruster::Thruster(const CanDispatcher::Ptr &can_dispatcher,
                         const ros::NodeHandlePtr &nh,std::string thruster_name,
										 			Actuators unique_id) ATLAS_NOEXCEPT
    : CanDevice(actuators, unique_id, can_dispatcher, thruster_name + NAME),
			thruster_specific_name_(thruster_name),
	 	 	properties_sent_(false){

		thruster_pub_ =
    nh->advertise<sonia_msgs::ThrusterMsg>(thruster_specific_name_ + "_thruster_msgs", 100);
		thruster_properties_pub_ =
    nh->advertise<sonia_msgs::CanDevicesProperties>(thruster_specific_name_ + "_thruster_properties", 100);

   // sends device's properties if device is present
   if(DevicePresenceCheck()){
	   SendProperties();
	   properties_sent_ = true;
   }

}

//------------------------------------------------------------------------------
//
	Thruster::~Thruster() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void Thruster::Process() ATLAS_NOEXCEPT {
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
		case THRUSTER_STATE_MSG:
		  ros_msg.current = rx_buffer[i].data[0];
			ros_msg.speed = rx_buffer[i].data[1];
			ros_msg.temperature = rx_buffer[i].data[2];
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
				case set_speed:
					SetSpeed((int8_t)pc_messages_buffer[i].parameter_value);
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
			thruster_pub_.publish(ros_msg);
  }
}

//------------------------------------------------------------------------------
//

void Thruster::SendProperties() const ATLAS_NOEXCEPT {
	sonia_msgs::CanDevicesProperties ros_msg;
	DeviceProperties properties = GetProperties();

	ros_msg.capabilities = properties.capabilities;
	ros_msg.device_data = properties.device_data;
	ros_msg.firmware_version = properties.firmware_version;
	ros_msg.uc_signature = properties.uc_signature;
	//ros_msg.poll_rate = properties.poll_rate; // unsupported

	thruster_properties_pub_.publish(ros_msg);
}

//------------------------------------------------------------------------------
//

void Thruster::SetSpeed(int8_t speed) const ATLAS_NOEXCEPT {
	if(speed > 100)
		speed = 0;

	if(speed < -100)
		speed = 0;

	PushMessage(SET_SPEED_MSG, (uint8_t*)speed, 1);
}

}  // namespace provider_can
