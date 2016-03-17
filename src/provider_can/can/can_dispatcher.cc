/**
 * \file	can_dispatcher.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	07/12/2015
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <mutex>
#include <lib_atlas/exceptions/io_exception.h>
#include "can_dispatcher.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

// Delay after an ID request before listing devices
const uint32_t CanDispatcher::ID_REQ_WAIT = 100000;
// Number of ID request retries
const uint8_t CanDispatcher::DISCOVERY_TRIES = 10;
// Delay between ID request retries (sec)
const uint8_t CanDispatcher::DISCOVERY_DELAY = 5;
// Delay between error recovery retries
const uint8_t CanDispatcher::ERROR_RECOVERY_DELAY = 2;
// Delay to wait for a message to be sent (ms)
const uint32_t CanDispatcher::CAN_SEND_TIMEOUT = 10;

const uint32_t CanDispatcher::THREAD_INTERVAL_US = 1000;

const uint32_t CanDispatcher::PC_BUFFER_SIZE = 100;

const uint16_t CanDispatcher::PROVIDER_CAN_STATUS = 0xF00;

// Messages types
const uint32_t CanDispatcher::UNICAST = 0x10000000;

// Address parameters positions
const uint8_t CanDispatcher::UNIQUE_ID_POSITION = 12;
const uint8_t CanDispatcher::DEVICE_ID_POSITION = 20;

// Mac address mask
const uint32_t CanDispatcher::DEVICE_MAC_MASK = 0x0FFFF000;

// Message type mask
const uint32_t CanDispatcher::DEVICE_MSG_MASK = 0x00000FFF;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CanDispatcher::CanDispatcher(uint32_t device_id, uint32_t unique_id,
                             uint32_t chan, int32_t baudrate,
                             std::string usb_device,
                             const ros::NodeHandlePtr &nh)
    : can_driver_(),
      discovery_tries_(0),
      tx_error_(0),
      rx_error_(0),
      ovrr_error_(0),
      master_id_(),
      nh_(nh),
      call_device_srv_() {

  if (usb_device == "KVaser") {
    can_driver_ = std::make_shared<provider_can::KVaser>(chan, baudrate);
  } else {
    ROS_WARN_STREAM("Unknown USB Device " + usb_device);
    throw atlas::IOException("Unknown USB device");
  }

  master_id_ =
      (device_id << DEVICE_ID_POSITION) | (unique_id << UNIQUE_ID_POSITION);

  can_driver_->GetErrorCount(&tx_error_, &rx_error_, &ovrr_error_);

  clock_gettime(CLOCK_REALTIME, &actual_time_);
  clock_gettime(CLOCK_REALTIME, &initial_time_);
  clock_gettime(CLOCK_REALTIME, &id_req_time_);

  can_driver_->FlushRxBuffer();
  can_driver_->FlushTxBuffer();

  ListDevices();

  // initializing service for devices methods calling
  call_device_srv_ = nh_->advertiseService(
      "send_can_message", &provider_can::CanDispatcher::CallDeviceMethod, this);

  uint8_t can_enabled_ = 1;
  PushBroadMessage(PROVIDER_CAN_STATUS, &can_enabled_, 1);
}

//------------------------------------------------------------------------------
//
CanDispatcher::~CanDispatcher() ATLAS_NOEXCEPT {
  uint8_t can_enabled_ = 0;
  PushBroadMessage(PROVIDER_CAN_STATUS, &can_enabled_, 1);
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
canStatus CanDispatcher::ListDevices() ATLAS_NOEXCEPT {
  canStatus status;

  status = SendIdRequest();  // Ask ID from every device on CAN bus
  if (status < canOK) return status;

  usleep(ID_REQ_WAIT);      // Wait for all responses
  status = ReadMessages();  // Reads all messages into rx_raw_buffer_
  if (status < canOK) return status;

  // For each messages received during sleep,
  rx_raw_buffer_mutex_.lock();
  for (auto &message : rx_raw_buffer_) {
    CanDeviceBuffers new_device;
    // If the address of the message has never been seen
    if (FindDeviceWithAddress(message.id) == SONIA_DEVICE_NOT_PRESENT) {
      // Apending new device to the vector
      new_device.global_address = (message.id & DEVICE_MAC_MASK);

      devices_list_.push_back(new_device);
    }
  }

  rx_raw_buffer_mutex_.unlock();
  DispatchMessages();  // Dispatch and saves all received messages

  unknown_addresses_table_.clear();  // resets unknown addresses discovery.

  return status;
}

//------------------------------------------------------------------------------
//
void CanDispatcher::DispatchMessages() ATLAS_NOEXCEPT {
  SoniaDeviceStatus status;
  size_t index;

  // For each message contained in raw buffer
  std::lock_guard<std::mutex> lock(rx_raw_buffer_mutex_);
  for (auto &message : rx_raw_buffer_) {
    // get the device_list index where address is located
    status = FindDeviceWithAddress(message.id, &index);

    if (status != SONIA_DEVICE_NOT_PRESENT) {  // If device exists

      std::lock_guard<std::mutex> lock(can_rx_buffer_mutex);

      // Avoids buffer overflow
      if (devices_list_[index].can_rx_buffer.size() >=
          DISPATCHED_RX_BUFFER_SIZE) {
        devices_list_[index].can_rx_buffer.clear();
        ROS_WARN("Device %X: Can Rx Buffer Overflow. Messages dropped.",
                 devices_list_[index].global_address);
      }

      // isolate the message type.
      message.id = message.id & DEVICE_MSG_MASK;
      // Saves message into dispatched devices' buffers.
      devices_list_[index].can_rx_buffer.push_back(message);

    } else {  // If address is unknown
      // Adds the address to the unknown addresses table
      AddUnknownAddress(message.id);
    }
  }

  rx_raw_buffer_.clear();
}

//------------------------------------------------------------------------------
//
canStatus CanDispatcher::ReadMessages() ATLAS_NOEXCEPT {
  canStatus status;

  std::lock_guard<std::mutex> lock(rx_raw_buffer_mutex_);
  status = can_driver_->ReadAllMessages(rx_raw_buffer_);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDispatcher::SendMessages() ATLAS_NOEXCEPT {
  canStatus status;

  std::lock_guard<std::mutex> lock(tx_raw_buffer_mutex_);
  status = can_driver_->WriteBuffer(tx_raw_buffer_, CAN_SEND_TIMEOUT);
  tx_raw_buffer_.clear();
  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDispatcher::SendIdRequest() ATLAS_NOEXCEPT {
  CanMessage msg;

  msg.id = 0;
  msg.data[0] = 0;
  msg.dlc = 0;
  msg.flag = canMSG_EXT;
  msg.time = 0;

  return (can_driver_->WriteMessage(msg, 1));
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::FetchCanMessages(
    uint8_t device_id, uint8_t unique_id,
    std::vector<CanMessage> &buffer) ATLAS_NOEXCEPT {
  SoniaDeviceStatus status;
  size_t index;

  // Gets device buffer index
  status = FindDevice(device_id, unique_id, &index);

  buffer.clear();

  if (status != SONIA_DEVICE_NOT_PRESENT) {
    std::lock_guard<std::mutex> lock(can_rx_buffer_mutex);
    // returns device's buffer
    buffer = devices_list_[index].can_rx_buffer;
    devices_list_[index].can_rx_buffer.clear();
  }

  return status;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::PushUnicastMessage(
    uint8_t device_id, uint8_t unique_id, uint16_t message_id, uint8_t *buffer,
    uint8_t ndata) ATLAS_NOEXCEPT {
  CanMessage message;

  if (ndata > 8)  // DLC cannot be larger than 8!!
    ndata = 8;

  // converts id to SONIA format
  message.id = UNICAST | (device_id << DEVICE_ID_POSITION) |
               (unique_id << UNIQUE_ID_POSITION) | (message_id & 0x0FFF);
  message.flag = canMSG_EXT;
  message.dlc = ndata;

  for (int i = 0; i < ndata && buffer != NULL; i++) {
    message.data[i] = buffer[i];
  }

  tx_raw_buffer_mutex_.lock();
  // Ensure the buffer does not overfill
  if (tx_raw_buffer_.size() <= PC_BUFFER_SIZE) {
    tx_raw_buffer_.push_back(message);
  } else {
    ROS_WARN("Trying to send too many messages on can bus");
  }
  tx_raw_buffer_mutex_.unlock();
  return FindDevice(device_id, unique_id);
}

//------------------------------------------------------------------------------
//
void CanDispatcher::PushBroadMessage(uint16_t message_id, uint8_t *buffer,
                                     uint8_t ndata) ATLAS_NOEXCEPT {
  CanMessage message;

  if (ndata > 8)  // DLC cannot be larger than 8!!
    ndata = 8;

  // Pushes a message using master id
  message.id = master_id_ | (message_id & 0x0FFF);
  message.flag = canMSG_EXT;
  message.dlc = ndata;
  for (int i = 0; i < ndata && buffer != NULL; i++) {
    message.data[i] = buffer[i];
  }

  std::lock_guard<std::mutex> lock(tx_raw_buffer_mutex_);
  // Ensure the tx buffer does not overfill
  if (tx_raw_buffer_.size() <= PC_BUFFER_SIZE) {
    tx_raw_buffer_.push_back(message);
  } else {
    ROS_WARN("Trying to send too many messages on can bus");
  }
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::FindDevice(uint8_t device_id,
                                            uint8_t unique_id,
                                            size_t *index) ATLAS_NOEXCEPT {
  uint32_t global_address =
      (device_id << DEVICE_ID_POSITION) | (unique_id << UNIQUE_ID_POSITION);

  return FindDeviceWithAddress(global_address, index);
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::FindDevice(uint8_t device_id,
                                            uint8_t unique_id) ATLAS_NOEXCEPT {
  uint32_t global_address =
      (device_id << DEVICE_ID_POSITION) | (unique_id << UNIQUE_ID_POSITION);

  return FindDeviceWithAddress(global_address);
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::FindDeviceWithAddress(
    uint32_t address, size_t *index) ATLAS_NOEXCEPT {
  SoniaDeviceStatus status = SONIA_DEVICE_NOT_PRESENT;

  address = address & DEVICE_MAC_MASK;

  // predicate used for seeking for a device in device_list_ vector
  auto add_search_pred = [address](const CanDeviceBuffers &device) {
    return device.global_address == address;
  };

  auto vec_it =
      std::find_if(devices_list_.begin(), devices_list_.end(), add_search_pred);

  // Seeking for specified address in device_list_
  if (vec_it != devices_list_.end()) {
    // Calculating the exact index
    *index = std::distance(devices_list_.begin(), vec_it);
    status = SONIA_DEVICE_PRESENT;
  }

  if (status == SONIA_DEVICE_NOT_PRESENT) {
    AddUnknownAddress(address);
  }

  return status;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::FindDeviceWithAddress(uint32_t address)
    ATLAS_NOEXCEPT {
  size_t index;
  return FindDeviceWithAddress(address, &index);
}

//------------------------------------------------------------------------------
//
void CanDispatcher::SendRTR(uint32_t address) ATLAS_NOEXCEPT {
  CanMessage msg;

  msg.id = address;
  msg.data[0] = 0;
  msg.dlc = 0;
  msg.flag = canMSG_RTR;
  msg.time = 0;

  can_driver_->WriteMessage(msg, 1);
}

//------------------------------------------------------------------------------
//
uint64_t CanDispatcher::GetNumberOfDevices() ATLAS_NOEXCEPT {
  return devices_list_.size();
}

//------------------------------------------------------------------------------
//
void CanDispatcher::GetUnknownAddresses(std::vector<uint32_t> &addresses)
    ATLAS_NOEXCEPT {
  addresses = unknown_addresses_table_;
}

//------------------------------------------------------------------------------
//
void CanDispatcher::AddUnknownAddress(uint32_t address) ATLAS_NOEXCEPT {
  if (std::find(unknown_addresses_table_.begin(),
                unknown_addresses_table_.end(),
                address & DEVICE_MAC_MASK) == unknown_addresses_table_.end()) {
    unknown_addresses_table_.push_back(address);
  }
}

//------------------------------------------------------------------------------
//
void CanDispatcher::Run() ATLAS_NOEXCEPT {
  canStatus status;

  while (1) {
    clock_gettime(CLOCK_REALTIME, &actual_time_);

    // verifying CAN errors
    if (tx_error_ >= 50 || rx_error_ >= 50) {
      ROS_WARN(
          "Too many errors encountered (tx: %d, rx: %d, ovrr: %d). Verify "
          "KVaser connectivity. "
          "Stopping "
          "CAN until problem is "
          "solved",
          tx_error_, rx_error_, ovrr_error_);

      sleep(ERROR_RECOVERY_DELAY);

      ROS_INFO("Retrying a recovery");
      can_driver_->GetErrorCount(&tx_error_, &rx_error_, &ovrr_error_);

    }
    // normal process
    else {
      // reading from CAN bus
      status = ReadMessages();

      // verifying errors
      if (status < canOK) {
        can_driver_->PrintErrorText(status);
        can_driver_->GetErrorCount(&tx_error_, &rx_error_, &ovrr_error_);
        ROS_WARN("tx: %d, rx: %d, ovrr: %d", tx_error_, rx_error_, ovrr_error_);
      }

      // Dispatching read messages
      DispatchMessages();

      // sending messages contained in tx_buffer
      status = SendMessages();

      // verifying errors
      if (status < canOK) {
        can_driver_->PrintErrorText(status);
        can_driver_->GetErrorCount(&tx_error_, &rx_error_, &ovrr_error_);
        ROS_WARN("tx: %d, rx: %d, ovrr: %d", tx_error_, rx_error_, ovrr_error_);
      }

      // verifying if devices where not found. if so, sends ID requests to try
      // to find undiscovered devices.
      if (discovery_tries_ <= DISCOVERY_TRIES &&
          (actual_time_.tv_sec - id_req_time_.tv_sec) >= DISCOVERY_DELAY &&
          unknown_addresses_table_.size() != 0) {
        // Printing missing devices
        ROS_INFO("Unknown devices found:");
        for (auto &address : unknown_addresses_table_)
          printf("%X\n\r", address);
        ROS_INFO("Retrying ID Request");
        ListDevices();
        discovery_tries_++;
        id_req_time_ = actual_time_;
      }
    }
    usleep(THREAD_INTERVAL_US);
  }
}

//------------------------------------------------------------------------------
//
bool CanDispatcher::CallDeviceMethod(sonia_msgs::SendCanMessage::Request &req,
                                     sonia_msgs::SendCanMessage::Response &res)
    ATLAS_NOEXCEPT {
  SoniaDeviceStatus status;
  ComputerMessage msg = {msg.method_number = req.method_number,
                         msg.parameter_value = req.parameter_value};
  msg.string_param = req.string_param;
  size_t index;
  // get device index
  status = FindDevice(req.device_id, req.unique_id, &index);

  if (status != SONIA_DEVICE_NOT_PRESENT) {
    std::lock_guard<std::mutex> lock(pc_messages_buffer_mutex);

    // Ensure the buffer does not overfill
    if (devices_list_[index].pc_messages_buffer.size() <= PC_BUFFER_SIZE) {
      devices_list_[index].pc_messages_buffer.push_back(msg);
    } else {
      ROS_WARN(
          "Device %X: send_can_message service called too fast. "
          "Some messages are dropped",
          devices_list_[index].global_address);
    }
  }

  res.device_status = status;
  return true;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::FetchComputerMessages(
    uint8_t device_id, uint8_t unique_id,
    std::vector<ComputerMessage> &buffer) ATLAS_NOEXCEPT {
  SoniaDeviceStatus status;
  size_t index;

  status = FindDevice(device_id, unique_id, &index);

  // ensure the input buffer is empty
  buffer.clear();

  if (status != SONIA_DEVICE_NOT_PRESENT) {
    std::lock_guard<std::mutex> lock(pc_messages_buffer_mutex);
    // returns device buffer
    buffer = devices_list_[index].pc_messages_buffer;
    devices_list_[index].pc_messages_buffer.clear();
  }

  return status;
}
}
