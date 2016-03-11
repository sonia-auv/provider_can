/**
 * \file	grabber.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	11/03/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <provider_can/can/can_dispatcher.h>
#include "provider_can/devices/droppers.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const std::string Droppers::NAME = "droppers";

// transmittable CAN messages
const uint16_t Droppers::DROP_MSG = 0xF00;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
Droppers::Droppers(const CanDispatcher::Ptr &can_dispatcher,
                   const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(markers, dropper, can_dispatcher, NAME, nh) {}

//------------------------------------------------------------------------------
//
Droppers::~Droppers() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void Droppers::ProcessMessages(
    const std::vector<CanMessage> &from_can_rx_buffer,
    const std::vector<ComputerMessage> &from_pc_rx_buffer) ATLAS_NOEXCEPT {
  // loops through all PC messages received
  for (auto &pc_message : from_pc_rx_buffer) {
    switch (pc_message.method_number) {
      case drop:
        Drop((uint8_t)pc_message.parameter_value);
        break;
      default:
        break;
    }
  }
}

//------------------------------------------------------------------------------
//

void Droppers::Drop(uint8_t dropper_to_drop) const ATLAS_NOEXCEPT {
  PushMessage(DROP_MSG, &dropper_to_drop, 1);
}

}  // namespace provider_can
