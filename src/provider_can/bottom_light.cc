/**
* \file	bottom_light.cc
* \author	Alexi Demers <alexidemers@gmail.com>
* \date	25/11/2015
*
* \copyright Copyright (c) 2015 Copyright
*
* \section LICENSE http://www.gnu.org/licenses/gpl-3.0.en.html
*
* Changes by: S.O.N.I.A.
* \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
*
* \section LICENSE
*
* This file is part of S.O.N.I.A. software.
*
* S.O.N.I.A. software is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* S.O.N.I.A. software is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
*/


#include "bottom_light.h"
#include "can_dispatcher.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const uint16_t BottomLight::SET_LIGHT_MSG = 0xF00;
const uint16_t BottomLight::SET_LIGHT_DLC = 1;
const std::string BottomLight::NAME = "Bottom Light";

//==============================================================================
// C / D T O R   S E C T I O N

BottomLight::BottomLight(std::shared_ptr<CanDispatcher> can_dispatcher):
          CanDevice(lights,bottom_light,can_dispatcher, NAME){
  actual_light_level_ = 200; // ensure we turn off the light at startup
  asked_light_level_ = 0;
  SetLevel(0);
}

//------------------------------------------------------------------------------
//

BottomLight::~BottomLight() {
}

//==============================================================================
// M E T H O D S   S E C T I O N

void BottomLight::Process() {
  std::vector<CanMessage> rx_buffer;
  std::vector<ComputerMessage> pc_messages_buffer;

  if(DevicePresenceCheck()) {
    // fetching CAN messages
    rx_buffer = FetchMessages();

    // if messages have been received
    if (rx_buffer.size() != 0) {
      // Collects the last message received (previous messages can be bypassed)
      actual_light_level_ = rx_buffer[rx_buffer.size() - 1].data[0];
    }
    // If a new light level has been asked
    if (asked_light_level_ != actual_light_level_) {
      // sets light level
      PushMessage(SET_LIGHT_MSG, &asked_light_level_, SET_LIGHT_DLC);
      actual_light_level_ = asked_light_level_;
    }

    // fetching pc messages (ROS)
    pc_messages_buffer = FetchComputerMessages();

    // loops through all PC messages received
    for(uint8_t i = 0; i < pc_messages_buffer.size(); i++){
      // if messages askes to call set_level function
      if(pc_messages_buffer[i].method_number == set_level){
        SetLevel((uint8_t)pc_messages_buffer[i].parameter_value);
      }
    }

  }
}

//------------------------------------------------------------------------------
//
void BottomLight::SetLevel(uint8_t level) {
  asked_light_level_ = level;
}

//------------------------------------------------------------------------------
//
uint8_t BottomLight::GetLevel() {
  return actual_light_level_;
}

} // namespace provider_can
