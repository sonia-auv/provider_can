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
#include <sonia_msgs/CanDevicesNotices.h>
#include <sonia_msgs/CanDevicesProperties.h>

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

// transmittable can messages
const uint8_t CanDevice::RESET_REQ = 0xfe;
const uint8_t CanDevice::WAKEUP_REQ = 0xf1;
const uint8_t CanDevice::SLEEP_REQ = 0xf0;
const uint8_t CanDevice::PING = 0x01;

// receivable can messages
const uint8_t CanDevice::DEVICE_FAULT = 0xff;
const uint8_t CanDevice::ID_REQ_RESPONSE = 0x00;
const uint8_t CanDevice::PING_RESPONSE = 0x01;

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
      from_can_rx_buffer_(),
      from_pc_rx_buffer_(),
      device_found_(false),
      device_properties_() {
  properties_pub_ = nh->advertise<sonia_msgs::CanDevicesProperties>(
      name_ + "_properties", 100);

  device_notices_pub_ =
      nh->advertise<sonia_msgs::CanDevicesNotices>(name_ + "_notices", 100);

  // sends device's properties if device is present
  if (DevicePresenceCheck()) {
    device_found_ = true;
    ROS_INFO_STREAM("Device " + name_ + " Found");
  } else {
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

  ros_msg.capabilities = device_properties_.capabilities;
  ros_msg.device_data = device_properties_.device_data;
  ros_msg.firmware_version = device_properties_.firmware_version;
  ros_msg.uc_signature = device_properties_.uc_signature;

  properties_pub_.publish(ros_msg);
}

//------------------------------------------------------------------------------
//

void CanDevice::Process() ATLAS_NOEXCEPT {
  if (DevicePresenceCheck()) {
    // is device is present and properties has not been sent
    if (!device_found_) {
      device_found_ = true;
      ROS_INFO_STREAM("Device " + name_ + " Found");
    }

    // fetching CAN messages
    can_dispatcher_->FetchCanMessages(device_id_, unique_id_,
                                      from_can_rx_buffer_);

    // looking for common devices messages into buffer
    ProcessCommonCanMessages();

    // fetching pc messages (ROS)
    can_dispatcher_->FetchComputerMessages(device_id_, unique_id_,
                                           from_pc_rx_buffer_);

    // looking for common devices messages into buffer
    ProcessCommonPcMessages();

    // Allows the device to process specific messages
    ProcessMessages(from_can_rx_buffer_, from_pc_rx_buffer_);
  }
}

//------------------------------------------------------------------------------
//

void CanDevice::ProcessCommonPcMessages(void) ATLAS_NOEXCEPT {
  // loops through all PC messages received
  for (auto &pc_message : from_pc_rx_buffer_) {
    // if messages askes to call a common function
    switch (pc_message.method_number) {
      case ping_req:
        PingDevice();
        break;
      case get_properties:
        SendProperties();
        break;
      default:
        break;
    }
  }
}

//------------------------------------------------------------------------------
//

void CanDevice::ProcessCommonCanMessages(void) ATLAS_NOEXCEPT {
  bool message_rcvd = false;
  sonia_msgs::CanDevicesNotices ros_msg_;
  // loops through all PC messages received
  for (auto &can_message : from_can_rx_buffer_) {
    switch (can_message.id) {
      case PING_RESPONSE:
        // ping has been received from device
        ros_msg_.ping_rcvd = (uint8_t) true;
        message_rcvd = true;
        break;

      case ID_REQ_RESPONSE:
        // Properties have been received from device
        device_properties_.firmware_version =
            (can_message.data[1] << 8) | can_message.data[0];

        device_properties_.uc_signature = (can_message.data[4] << 16) |
                                          (can_message.data[3] << 8) |
                                          can_message.data[2];

        device_properties_.capabilities = can_message.data[5];

        device_properties_.device_data = can_message.data[6];

        // Send properties on ROS topic
        SendProperties();
        break;

      case DEVICE_FAULT:
        for (uint8_t i = 0; i < 8; i++) ros_msg_.fault[i] = can_message.data[i];
        message_rcvd = true;
        break;

      default:
        break;
    }
  }

  // Sends notices on ROS topic if something happened
  if (message_rcvd) device_notices_pub_.publish(ros_msg_);
}

}  // namespace provider_can
