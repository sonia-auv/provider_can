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
// C / D T O R   S E C T I O N

BottomLight::BottomLight(CanDispatcher *can) {
  can_dispatcher_ = can;
  actual_light_level_ = 0;
  asked_light_level_ - 0;
}

//------------------------------------------------------------------------------
//

BottomLight::~BottomLight() {

}

//==============================================================================
// M E T H O D S   S E C T I O N

void BottomLight::lightProcess() {
  CanMessage *rx_buffer;
  uint8_t num_of_messages;
  SoniaDeviceStatus status =
      can_dispatcher_->fetchMessages(lights, bottom_light, rx_buffer, &num_of_messages);

  if (status != SONIA_DEVICE_NOT_PRESENT) {
    device_present_ = true;

    if (num_of_messages != 0) {
      actual_light_level_ = rx_buffer->data[num_of_messages - 1];
      if (asked_light_level_ != actual_light_level_) {
        can_dispatcher_->pushUnicastMessage(lights, bottom_light, SET_LIGHT_MSG,
                                            &asked_light_level_, SET_LIGHT_DLC);
      }
    }

    if (status == SONIA_DEVICE_FAULT) {
      device_fault = true;
      can_dispatcher_->getDeviceFault(lights, bottom_light, fault_message);
    }
  }
  else {
    device_present_ = false;
  }

}

//------------------------------------------------------------------------------
//
void BottomLight::setLightLevel(uint8_t level) {
  asked_light_level_ = level;
}

//------------------------------------------------------------------------------
//
void BottomLight::resetLight() {
  can_dispatcher_->sendResetRequest(lights, bottom_light);
}

} // namespace provider_can
