/**
 * \file	can_device.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	26/01/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_can/can_node.h"

namespace provider_can {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CanNode::CanNode(std::shared_ptr<ros::NodeHandle> nh) {
  can_ptr_ = std::make_shared<provider_can::CanDispatcher>(
      controllers, on_board_pc, 0, BAUD_125K, 10);

  // initialize all new devices here
  can_devices_vector_.push_back(
      std::make_shared<provider_can::BottomLight>(can_ptr_,nh));

  // initializing service for devices methods calling
  ros::ServiceServer service =
      nh->advertiseService("call_device_method",
  &CanDispatcher::CallDeviceMethod, can_ptr_.get());
}

//------------------------------------------------------------------------------
//
CanNode::~CanNode() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void CanNode::ProcessMessages(void) {
  for (uint8_t i = 0; i < can_devices_vector_.size(); i++)
    can_devices_vector_[i]->Process();
}

} // namespace provider_can
