/**
 * \file	bottom_light.h
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

#ifndef PROVIDER_CAN_BOTTOM_LIGHT_H_
#define PROVIDER_CAN_BOTTOM_LIGHT_H_

#include <memory>
#include "provider_can/can_dispatcher.h"
#include "provider_can/can_def.h"

namespace provider_can {


class BottomLight {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BottomLight>;

  const static uint16_t SET_LIGHT_MSG = 0xF00;
  const static uint16_t SET_LIGHT_DLC = 1;

  //============================================================================
  // P U B L I C   C / D T O R S

  BottomLight(CanDispatcher *can);

  ~BottomLight();

  //============================================================================
  // P U B L I C   M E T H O D S

  void setLightLevel(uint8_t level);
  void lightProcess();
  void resetLight();

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  CanDispatcher::Ptr can_dispatcher_; // pointer to can controller
  uint8_t actual_light_level_;    // Light actual state
  uint8_t asked_light_level_;     // set by setLightLevel()
  bool device_present_;           // True if device is present on CAN bus
  bool device_fault;              // True if a fault has been encountered
  uint8_t *fault_message;         // Fault message if device_fault is true
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DRIVER_H_
