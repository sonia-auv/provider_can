/**
 * \file	bottom_light.cc
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
  const uint16_t DiverInterface::SET_STATE_MSG = 0xF00;
  const uint16_t DiverInterface::SET_MISSION_MSG = 0xF01;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
  DiverInterface::DiverInterface(const CanDispatcher::Ptr &can_dispatcher,
                           const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(interfaces, diver_interface, can_dispatcher, NAME, nh),
      mission_string_(),
      state_string_(),
      mission_string_in_construction(false),
      state_string_in_construction(false)
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
        SendMissionString((char)pc_message.parameter_value);
      break;
      case set_state_string:
        SendStateString((char)pc_message.parameter_value);
      break;
      default:
      break;
    }
  }

}
//------------------------------------------------------------------------------
//

  void DiverInterface::SendMissionString(char character) ATLAS_NOEXCEPT{

    // if a new string was started
    if(character == '{')
      mission_string_in_construction = true;
    else if (!mission_string_in_construction)
      ROS_WARN("Diver interface mission string error. Be sure to start and end the string with {}");

    // If a string is in construction
    if(mission_string_in_construction){
      // Add a new character to the string
      mission_string_.push_back(character);

      // Each 8 characters added, sends a can message
      if(mission_string_.length() == 8 ){
        PushMessage(SET_MISSION_MSG, (uint8_t*)mission_string_.data(), 8);
        mission_string_.clear();
      }
    }

    // if the construction is asked to end
    if(character == '}')
    {
      // End construction and sends last characters
      mission_string_in_construction = false;
      if(mission_string_.length() != 0)
        PushMessage(SET_MISSION_MSG, (uint8_t*)mission_string_.data(), (uint8_t)mission_string_.length());
    }
  }

//------------------------------------------------------------------------------
//

  void DiverInterface::SendStateString(char character) ATLAS_NOEXCEPT{

    // if a new string was started
    if(character == '{')
      state_string_in_construction = true;
    else if (!state_string_in_construction)
      ROS_WARN("Diver interface mission string error. Be sure to start and end the string with {}");

    // If a string is in construction
    if(state_string_in_construction){
      // Add a new character to the string
      state_string_.push_back(character);

      // Each 8 characters added, sends a can message
      if(state_string_.length() == 8 ){
        PushMessage(SET_STATE_MSG, (uint8_t*)state_string_.data(), 8);
        state_string_.clear();
      }
    }

    // if the construction is asked to end
    if(character == '}')
    {
      // End construction and sends last characters
      state_string_in_construction = false;
      if(state_string_.length() != 0)
        PushMessage(SET_MISSION_MSG, (uint8_t*)state_string_.data(), (uint8_t)state_string_.length());
    }
  }

}  // namespace provider_can
