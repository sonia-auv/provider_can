/**
 * \file	can_device.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	04/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_can/devices/can_device.h"
#include <sonia_msgs/CanDevicesProperties.h>

namespace provider_can {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CanDevice::CanDevice(const DeviceClass &device_id, uint8_t unique_id,
                     const CanDispatcher::Ptr &can_dispatcher,
                     const std::string &name,
                     const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : can_dispatcher_(can_dispatcher),
      fault_message_(nullptr),
      name_(name),
      device_id_(device_id),
      unique_id_(unique_id) {
  properties_pub_ = nh->advertise<sonia_msgs::CanDevicesProperties>(
      name_ + "_properties", 100);
}

//------------------------------------------------------------------------------
//
CanDevice::~CanDevice() ATLAS_NOEXCEPT {}

//------------------------------------------------------------------------------
//
void CanDevice::SendProperties() const ATLAS_NOEXCEPT {
  sonia_msgs::CanDevicesProperties ros_msg;
  DeviceProperties properties = GetProperties();

  ros_msg.capabilities = properties.capabilities;
  ros_msg.device_data = properties.device_data;
  ros_msg.firmware_version = properties.firmware_version;
  ros_msg.uc_signature = properties.uc_signature;
  // ros_msg.poll_rate = properties.poll_rate; // unsupported

  properties_pub_.publish(ros_msg);
}

}  // namespace provider_can
