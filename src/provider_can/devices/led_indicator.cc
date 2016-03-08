/**
 * \file	led_indicator.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	29/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <provider_can/can/can_dispatcher.h>
#include "provider_can/devices/led_indicator.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const std::string LedIndicator::NAME = "led_indicator";

// transmittable CAN messages
const uint16_t LedIndicator::SET_MODE_MSG = 0xF00;
const uint16_t LedIndicator::SET_COLOR_MSG = 0xF00;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
LedIndicator::LedIndicator(const CanDispatcher::Ptr &can_dispatcher,
                           const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(lights, led_indicator, can_dispatcher, NAME, nh),
      color_(0),
      mode_(0) {}

//------------------------------------------------------------------------------
//
LedIndicator::~LedIndicator() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void LedIndicator::ProcessMessages(
    const std::vector<CanMessage> &from_can_rx_buffer,
    const std::vector<ComputerMessage> &from_pc_rx_buffer) ATLAS_NOEXCEPT {
  // loops through all PC messages received
  for (auto &pc_message : from_pc_rx_buffer) {
    switch (pc_message.method_number) {
      case set_mode:
        SetMode((uint8_t)pc_message.parameter_value);
        break;
      case set_color:
        SetColor((uint8_t)pc_message.parameter_value);
        break;
      default:
        break;
    }
  }
}

//------------------------------------------------------------------------------
//

void LedIndicator::SetColor(uint8_t color) ATLAS_NOEXCEPT {
  color_ = color;

  uint8_t msg[2] = {color_, mode_};
  PushMessage(SET_COLOR_MSG, msg, 2);
}

//------------------------------------------------------------------------------
//

void LedIndicator::SetMode(uint8_t mode) ATLAS_NOEXCEPT {
  mode_ = mode;

  uint8_t msg[2] = {color_, mode_};
  PushMessage(SET_MODE_MSG, msg, 2);
}

}  // namespace provider_can
