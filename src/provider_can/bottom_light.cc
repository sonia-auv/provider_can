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
  actual_light_level_ = 0;
  asked_light_level_ = 0;

}

//------------------------------------------------------------------------------
//

BottomLight::~BottomLight() {
}

//==============================================================================
// M E T H O D S   S E C T I O N

void BottomLight::Process() {
  std::vector<CanMessage> rx_buffer;

  if(DevicePresenceCheck()) {
    rx_buffer = FetchMessages();

    if (rx_buffer.size() != 0) {
      actual_light_level_ = rx_buffer[rx_buffer.size() - 1].data[0];
      if (asked_light_level_ != actual_light_level_) {
        PushMessage(SET_LIGHT_MSG, &asked_light_level_, SET_LIGHT_DLC);
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
