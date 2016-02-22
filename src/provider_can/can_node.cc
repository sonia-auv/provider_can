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
// S T A T I C   M E M B E R S

const uint32_t CanNode::THREAD_INTERVAL_US = 10000;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CanNode::CanNode(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : nh_(nh),
      conf_(nh),
      can_ptr_(nullptr),
      can_devices_vector_({}) {
  can_ptr_ = std::make_shared<provider_can::CanDispatcher>(
      conf_.device_id, conf_.unique_id, conf_.channel, conf_.baudrate, nh_);
  can_ptr_->Start();

  // initialize all new devices here
  can_devices_vector_.push_back(
      std::make_shared<provider_can::BottomLight>(can_ptr_, nh_));

  can_devices_vector_.push_back(
      std::make_shared<provider_can::PowerSupply>(can_ptr_, nh_));

  can_devices_vector_.push_back(std::make_shared<provider_can::Thruster>(
      can_ptr_, nh_, "port", port_motor));
  can_devices_vector_.push_back(std::make_shared<provider_can::Thruster>(
      can_ptr_, nh_, "front_depth", front_depth_motor));
  can_devices_vector_.push_back(std::make_shared<provider_can::Thruster>(
      can_ptr_, nh_, "back_depth", back_depth_motor));
  can_devices_vector_.push_back(std::make_shared<provider_can::Thruster>(
      can_ptr_, nh_, "front_heading", front_heading_motor));
  can_devices_vector_.push_back(std::make_shared<provider_can::Thruster>(
      can_ptr_, nh_, "back_heading", back_heading_motor));
  can_devices_vector_.push_back(std::make_shared<provider_can::Thruster>(
      can_ptr_, nh_, "starboard", starboard_motor));

  can_devices_vector_.push_back(std::make_shared<provider_can::Barometer>(
      can_ptr_, nh_));
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
    usleep(THREAD_INTERVAL_US);
  }
}

}  // namespace provider_can
