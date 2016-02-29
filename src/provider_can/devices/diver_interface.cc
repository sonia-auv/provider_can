/**
 * \file	diver_interface.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	04/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <sonia_msgs/CanDevicesProperties.h>
#include <provider_can/can/can_dispatcher.h>
#include "provider_can/devices/diver_interface.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

  const std::string DiverInterface::NAME = "diver_interface";
  const uint16_t DiverInterface::SET_STATE_MSG = 0xF01;
  const uint16_t DiverInterface::SET_MISSION_MSG = 0xF00;
  const uint16_t DiverInterface::MISSION_SWITCH_STATE = 0xF00;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
  DiverInterface::DiverInterface(const CanDispatcher::Ptr &can_dispatcher,
                           const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(interfaces, diver_interface, can_dispatcher, NAME, nh)
  {
  }

//------------------------------------------------------------------------------
//
  DiverInterface::~DiverInterface() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
  void DiverInterface::ProcessMessages(
    const std::vector<CanMessage> &rx_buffer,
    const std::vector<ComputerMessage> &pc_messages_buffer) ATLAS_NOEXCEPT {

  // loops through all PC messages received
  for (auto &pc_message : pc_messages_buffer) {
  // if messages askes to call specific functions
    switch (pc_message.method_number) {
      case set_mission_string:
        SendMissionString(pc_message.string_param);
      break;
      case set_state_string:
        SendStateString(pc_message.string_param);
      break;
      default:
      break;
    }
  }


}
//------------------------------------------------------------------------------
//

  void DiverInterface::SendMissionString(std::string string) const ATLAS_NOEXCEPT{
	  // Strings must start and end by these characters
	  string.push_back('}');
	  string.insert(0,1,'{');

	  // each can message cannot contain more than 8 characters
	  if(string.length() > 8){
		  while(string.length()/8){
			  PushMessage(SET_MISSION_MSG, (uint8_t*)string.data(), 8);
			  string.erase(0,8);
		  }
	  }
	  PushMessage(SET_MISSION_MSG, (uint8_t*)string.data(), 8);
  }

//------------------------------------------------------------------------------
//

  void DiverInterface::SendStateString(std::string string) const ATLAS_NOEXCEPT{
	  // Strings must start and end by these characters
	  string.push_back('}');
	  string.insert(0,1,'{');

	  // each can message cannot contain more than 8 characters
	  if(string.length() > 8){
		  while(string.length()/8){
			  PushMessage(SET_STATE_MSG, (uint8_t*)string.data(), 8);
			  string.erase(0,8);
		  }
	  }
	  PushMessage(SET_STATE_MSG, (uint8_t*)string.data(), 8);
  }
}  // namespace provider_can
