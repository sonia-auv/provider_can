/**
* \file	can_dispatcher.cc
* \author	Alexi Demers <alexidemers@gmail.com>
* \date	07/12/2015
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

#include "can_dispatcher.h"
#include "can_driver.h"
#include <unistd.h>
#include <string.h>

using namespace provider_can;

//==============================================================================
// C / D T O R   S E C T I O N

CanDispatcher::CanDispatcher(uint32_t chan, uint32_t baudrate, uint32_t loop_rate)
    : canDriver_(chan, baudrate) {
  ndevices_present_ = 0;
  nunknown_addresses_ = 0;
  discovery_tries_ = 0;
  loop_rate_ = loop_rate;

  canDriver_.getErrorCount(&tx_error_, &rx_error_, &ovrr_error_);

  clock_gettime(CLOCK_REALTIME, &actual_time_);
  clock_gettime(CLOCK_REALTIME, &initial_time_);
  clock_gettime(CLOCK_REALTIME, &id_req_time_);
  clock_gettime(CLOCK_REALTIME, &error_recovery_);

  tx_raw_buffer_.buffer = new CanMessage[RAW_TX_BUFFER_SIZE];

  canDriver_.flushRxBuffer();
  canDriver_.flushTxBuffer();

  listDevices();
}

//------------------------------------------------------------------------------
//
CanDispatcher::~CanDispatcher() {}

//==============================================================================
// M E T H O D S   S E C T I O N

canStatus CanDispatcher::listDevices() {
  bool new_device_found = true;
  canStatus status;

  nunknown_addresses_ = 0;  // resets unknown addresses discovery.

  status = sendIdRequest();      // Ask ID from every device on CAN bus
  if(status < canOK)
    return status;

  usleep(ID_REQ_WAIT);  // Wait for all responses
  status = readMessages();       // Reads all messages into rx_raw_buffer_
  if(status < canOK)
    return status;

  for (int j = 0; j < rx_raw_buffer_.num_of_messages; j++) {
    // For each messages received during sleep,
    for (int i = 0; i < ndevices_present_; i++) {
      // For each device found until now
      if ((rx_raw_buffer_.buffer[j].id & DEVICE_ADDRESS_MASK) ==
          devices_list_[i].global_address)  // Verify if the ID has
        new_device_found = false;           // already been seen. If so,
                                            // avoids adding it again.
    }
    if (new_device_found)
      // If the messages contains a new address, adds a device
      devices_list_[ndevices_present_++].global_address =
          (rx_raw_buffer_.buffer[j].id & DEVICE_ADDRESS_MASK);

    new_device_found = true;  // Ready for next loop
  }

  dispatchMessages();  // Dispatch and saves all received messages

  return status;
}

//------------------------------------------------------------------------------
//
void CanDispatcher::dispatchMessages() {
  SoniaDeviceStatus status;
  int index;

  // For each message contained in raw buffer

  for (int j = 0; j < rx_raw_buffer_.num_of_messages; j++) {
    // get the device_list index where address is located
    status = getAddressIndex(rx_raw_buffer_.buffer[j].id, &index);

    if (status != SONIA_DEVICE_NOT_PRESENT) {  // If device exists
      // If the ID received correspond to an ID request response
      // saves device's parameters.
      if (devices_list_[index].global_address == rx_raw_buffer_.buffer[j].id) {
        devices_list_[index].device_properties.firmware_version =
            (rx_raw_buffer_.buffer[j].data[1] << 8) |
            rx_raw_buffer_.buffer[j].data[0];

        devices_list_[index].device_properties.uc_signature =
            (rx_raw_buffer_.buffer[j].data[4] << 16) |
            (rx_raw_buffer_.buffer[j].data[3] << 8) |
            rx_raw_buffer_.buffer[j].data[2];

        devices_list_[index].device_properties.capabilities =
            rx_raw_buffer_.buffer[j].data[5];

        devices_list_[index].device_properties.device_data =
            rx_raw_buffer_.buffer[j].data[6];
      }
      // If the ID received correspond to a device fault
      else if ((devices_list_[index].global_address | 0xFF) ==
               rx_raw_buffer_.buffer[j].id) {
        devices_list_[index].device_fault = true;

        printf("\n\rDevice %X: Fault ", devices_list_[index].global_address);
      }
      // If the ID received correspond to any other message
      else if (devices_list_[index].global_address ==
               (rx_raw_buffer_.buffer[j].id & DEVICE_ADDRESS_MASK)) {
        // Avoids buffer overflow
        if (devices_list_[index].num_of_messages >= DISPATCHED_RX_BUFFER_SIZE) {
          devices_list_[index].num_of_messages = 0;
          printf("\n\rDevice %X: Buffer Overflow. You must empty it faster.",
                 devices_list_[index].global_address);
        }

        // Saves message into dispatched devices' buffers.
        devices_list_[index].rx_buffer[devices_list_[index].num_of_messages++] =
            rx_raw_buffer_.buffer[j];
      }
    } else {  // If address is unknown
              // Adds the address to the unknown addresses table
      addUnknownAddress(rx_raw_buffer_.buffer[j].id);
    }
  }
  rx_raw_buffer_.num_of_messages =
      0;  // indicates that all messages have been read and dispatched.
}

//------------------------------------------------------------------------------
//
canStatus CanDispatcher::readMessages() {
  canStatus status;
  status = canDriver_.readAllMessages(rx_raw_buffer_.buffer,
                                      &rx_raw_buffer_.num_of_messages);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDispatcher::sendMessages() {
  canStatus status;

  for (int i = 0; i < tx_raw_buffer_.num_of_messages; i++) {
    status =
        canDriver_.writeMessage(tx_raw_buffer_.buffer[i], CAN_SEND_TIMEOUT);
    if (status < canOK) i = tx_raw_buffer_.num_of_messages;
  }

  tx_raw_buffer_.num_of_messages = 0;

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDispatcher::sendIdRequest() {
  CanMessage msg;

  msg.id = 0;
  msg.data[0] = 0;
  msg.dlc = 0;
  msg.flag = canMSG_EXT;
  msg.time = 0;

  return (canDriver_.writeMessage(msg, 1));
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::setPollRate(uint8_t device_id,
                                             uint8_t unique_id,
                                             uint16_t poll_rate) {
  SoniaDeviceStatus status;
  int index;

  status = getDeviceIndex(device_id, unique_id, &index);

  if (status != SONIA_DEVICE_NOT_PRESENT) {
    devices_list_[index].device_properties.poll_rate = poll_rate;
  }

  return status;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::fetchMessages(uint8_t device_id,
                                               uint8_t unique_id,
                                               CanMessage *&buffer,
                                               uint8_t *num_of_messages) {
  SoniaDeviceStatus status;
  int index;

  status = getDeviceIndex(device_id, unique_id, &index);

  if (status != SONIA_DEVICE_NOT_PRESENT) {
    buffer = devices_list_[index].rx_buffer;
    *num_of_messages = devices_list_[index].num_of_messages;
    devices_list_[index].num_of_messages = 0;
  }

  return status;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::getDevicesProperties(
    uint8_t device_id, uint8_t unique_id, DeviceProperties *properties) {
  SoniaDeviceStatus status;
  int index;

  status = getDeviceIndex(device_id, unique_id, &index);

  if (status != SONIA_DEVICE_NOT_PRESENT)
    properties = &devices_list_[index].device_properties;

  return status;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::pushMessage(uint8_t device_id,
                                             uint8_t unique_id,
                                             uint16_t message_id,
                                             uint8_t *buffer, uint8_t ndata) {
  int index;

  CanMessage message;
  message.id = UNICAST | (device_id << DEVICE_ID_POSITION) |
               (unique_id << UNIQUE_ID_POSITION) | (message_id & 0x0FFF);
  message.flag = canMSG_EXT;
  message.dlc = ndata;
  for (int i = 0; i < ndata; i++) message.data[i] = buffer[i];

  tx_raw_buffer_.buffer[tx_raw_buffer_.num_of_messages++] = message;

  return getDeviceIndex(device_id, unique_id, &index);
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::getDeviceIndex(uint8_t device_id,
                                                uint8_t unique_id, int *index) {
  SoniaDeviceStatus status = SONIA_DEVICE_NOT_PRESENT;

  uint32_t global_address =
      (device_id << DEVICE_ID_POSITION) | (unique_id << UNIQUE_ID_POSITION);

  for (int i = 0; i < ndevices_present_; i++)
    if (devices_list_[i].global_address ==
        (global_address & DEVICE_ADDRESS_MASK)) {
      *index = i;
      i = ndevices_present_;

      if (devices_list_[i].device_fault == true) {
        status = SONIA_DEVICE_FAULT;
      } else {
        status = SONIA_DEVICE_PRESENT;
      }
    }

  if (status == SONIA_DEVICE_NOT_PRESENT) addUnknownAddress(global_address);

  return status;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::getAddressIndex(uint32_t address, int *index) {
  SoniaDeviceStatus status = SONIA_DEVICE_NOT_PRESENT;

  uint32_t global_address = address & DEVICE_ADDRESS_MASK;

  for (int i = 0; i < ndevices_present_; i++)
    if (devices_list_[i].global_address ==
        (global_address & DEVICE_ADDRESS_MASK)) {
      *index = i;
      i = ndevices_present_;

      if (devices_list_[i].device_fault == true) {
        status = SONIA_DEVICE_FAULT;
      } else {
        status = SONIA_DEVICE_PRESENT;
      }
    }

  if (status == SONIA_DEVICE_NOT_PRESENT) addUnknownAddress(global_address);

  return status;
}

//------------------------------------------------------------------------------
//
void CanDispatcher::pollDevices() {

  uint32_t actual_time_ms = actual_time_.tv_nsec / 1000000;
  uint32_t initial_time_ms = initial_time_.tv_nsec / 1000000;

  for (int i = 0; i < ndevices_present_; i++) {
    if (((actual_time_ms - initial_time_ms) %
            devices_list_[i].device_properties.poll_rate) < ((1.0/
          (float)loop_rate_)
        *1000.0)) {
      sendRTR(devices_list_[i].global_address);
    }
  }
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::clearFault(uint8_t device_id,
                                            uint8_t unique_id) {
  SoniaDeviceStatus status;
  int index;
  status = getDeviceIndex(device_id, unique_id, &index);

  if (status != SONIA_DEVICE_NOT_PRESENT)
    devices_list_[index].device_fault = false;

  return status;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::sendResetRequest(uint8_t device_id,
                                                  uint8_t unique_id) {
  uint8_t *msg;

  return (pushMessage(device_id, unique_id, RESET_REQ, msg, 0));
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::sendSleepRequest(uint8_t device_id,
                                                  uint8_t unique_id) {
  uint8_t *msg;

  return (pushMessage(device_id, unique_id, SLEEP_REQ, msg, 0));
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::sendWakeUpRequest(uint8_t device_id,
                                                   uint8_t unique_id) {
  uint8_t *msg;

  return (pushMessage(device_id, unique_id, WAKEUP_REQ, msg, 0));
}

//------------------------------------------------------------------------------
//
void CanDispatcher::sendRTR(uint32_t address) {
  CanMessage msg;

  msg.id = address;
  msg.data[0] = 0;
  msg.dlc = 0;
  msg.flag = canMSG_RTR;
  msg.time = 0;

  canDriver_.writeMessage(msg, 1);
}

//------------------------------------------------------------------------------
//
uint8_t CanDispatcher::getNumberOfDevices() { return ndevices_present_; }

//------------------------------------------------------------------------------
//
uint8_t CanDispatcher::getUnknownAddresses(uint32_t *&addresses) {
  addresses = unknown_addresses_table_;
  return nunknown_addresses_;
}

//------------------------------------------------------------------------------
//
void CanDispatcher::addUnknownAddress(uint32_t address) {
  bool add_address = true;

  for (int i = 0; i < nunknown_addresses_; i++)
    if (unknown_addresses_table_[i] == (address & DEVICE_ADDRESS_MASK)) {
      add_address = false;
      i = nunknown_addresses_;
    }

  if (add_address) unknown_addresses_table_[nunknown_addresses_++] = address;
}

//------------------------------------------------------------------------------
//
void CanDispatcher::providerCanProcess() {
  canStatus status;
  clock_gettime(CLOCK_REALTIME, &actual_time_);


  // verifying CAN errors
  if(tx_error_ >= 50 || rx_error_ >= 50){
    printf("\rToo many errors encountered (tx: %d, rx: %d, ovrr: %d). Verify "
               "KVaser connectivity. "
               "Stopping "
               "CAN until problem is "
               "solved",tx_error_,rx_error_,ovrr_error_);

    if ((actual_time_.tv_sec - error_recovery_.tv_sec) >= ERROR_RECOVERY_DELAY) {
      printf("\n\rRetrying a recovery\n");

      error_recovery_ = actual_time_;
      canDriver_.getErrorCount(&tx_error_, &rx_error_, &ovrr_error_);
    }
  }
    // normal process
  else{

    // reading from CAN bus
    status = readMessages();

    // verifying errors
    if(status < canOK){
      printf("\n\r");
      canDriver_.printErrorText(status);
      canDriver_.getErrorCount(&tx_error_, &rx_error_, &ovrr_error_);
      printf("tx: %d, rx: %d, ovrr: %d",tx_error_,rx_error_,
             ovrr_error_);
    }

    // Dispatching read messages
    dispatchMessages();

    // sending messages contained in tx_buffer
    status = sendMessages();

    // verifying errors
    if(status < canOK){
      printf("\n\r");
      canDriver_.printErrorText(status);
      canDriver_.getErrorCount(&tx_error_, &rx_error_, &ovrr_error_);
      printf("tx: %d, rx: %d, ovrr: %d",tx_error_,rx_error_,ovrr_error_);
    }

    // Sends RTR to devices if asked
    pollDevices();

    // verifying if devices where not found. if so, sends ID requests to try
    // to find undiscovered devices.
    if (discovery_tries_ <= DISCOVERY_TRIES &&
        (actual_time_.tv_sec - id_req_time_.tv_sec) >= DISCOVERY_DELAY &&
        nunknown_addresses_ != 0) {
      printf("\n\rUnknown devices found. Retrying ID Request");
      listDevices();
      discovery_tries_++;
      id_req_time_ = actual_time_;
    }
  }
}
