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
#include <sonia_msgs/CanDevicesNotices.h>

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
      unique_id_(unique_id),
      properties_pub_(),
      device_notices_pub_(),
      rx_buffer_(),
      pc_messages_buffer_(),
      properties_sent_(false) {
  properties_pub_ = nh->advertise<sonia_msgs::CanDevicesProperties>(
      name_ + "_properties", 100);

  device_notices_pub_ =
      nh->advertise<sonia_msgs::CanDevicesNotices>(name_ + "_notices", 100);

  // sends device's properties if device is present
  if (DevicePresenceCheck()) {
    SendProperties();
    properties_sent_ = true;
    printf("\n\rDevice %s Found", name_.data());
  } else {
    printf("\n\r");
    ROS_WARN_STREAM("Device " + name_ + " not found");
  }
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

//------------------------------------------------------------------------------
//

void CanDevice::Process() ATLAS_NOEXCEPT {
  bool message_rcvd = false;
  sonia_msgs::CanDevicesNotices ros_msg_;

  if (DevicePresenceCheck()) {
    // is device is present and properties has not been sent
    if (!properties_sent_) {
      SendProperties();
      properties_sent_ = true;
      printf("\n\rDevice %s Found", name_.data());
    }

    // if ping has been received
    if (GetPingStatus()) {
      ros_msg_.ping_rcvd = true;
      message_rcvd = true;
    } else
      ros_msg_.ping_rcvd = false;

    // if a fault has been received
    const uint8_t *fault = GetFault();
    if (fault != NULL) {
      for (uint8_t i = 0; i < 8; i++) ros_msg_.fault[i] = fault[i];
      message_rcvd = true;
    } else
      for (uint8_t i = 0; i < 8; i++) ros_msg_.fault[i] = ' ';

    // Sends notices if something happened
    if (message_rcvd) device_notices_pub_.publish(ros_msg_);

    // fetching CAN messages
    can_dispatcher_->FetchMessages(device_id_, unique_id_, rx_buffer_);;

    // fetching pc messages (ROS)
    can_dispatcher_->FetchComputerMessages(device_id_, unique_id_, pc_messages_buffer_);

    // loops through all PC messages received
    for (auto &pc_message : pc_messages_buffer_) {
      // if messages askes to call set_level function
      switch (pc_message.method_number) {
        case ping_req:
          can_dispatcher_->PingDevice(device_id_, unique_id_);
          break;
        case get_properties:
          SendProperties();
          break;
        default:
          break;
      }
    }

    // Allows the device to process specific messages
    ProcessMessages(rx_buffer_, pc_messages_buffer_);
  }
}

}  // namespace provider_can
