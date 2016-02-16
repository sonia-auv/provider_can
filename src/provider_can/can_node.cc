/**
 * \file	can_device.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	26/01/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <sonia_msgs/BottomLightMsg.h>
#include "provider_can/can_node.h"

namespace provider_can {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CanNode::CanNode(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : nh_(nh),
      conf_(nh),
      can_ptr_(nullptr),
      call_device_srv_(),
      can_devices_vector_({}) {
  can_ptr_ = std::make_shared<provider_can::CanDispatcher>(
      conf_.device_id, conf_.unique_id, conf_.channel, conf_.baudrate, conf_.loop_rate);
  can_ptr_->Start();

  // initialize all new devices here
  can_devices_vector_.push_back(
      std::make_shared<provider_can::BottomLight>(can_ptr_, nh_));

  // initializing service for devices methods calling
  call_device_srv_ = nh_->advertiseService(
      "call_device_method", &CanDispatcher::CallDeviceMethod, can_ptr_.get());
}

//------------------------------------------------------------------------------
//
CanNode::~CanNode() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void CanNode::Run() ATLAS_NOEXCEPT {
  while (IsRunning()) {
    for (auto &device : can_devices_vector_) {
      device->Process();
    }
  }
}

}  // namespace provider_can
