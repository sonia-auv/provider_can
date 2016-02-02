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
#include <unistd.h>

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

  const uint32_t CanDispatcher:: THREAD_INTERVAL_US = 10000;

  const uint32_t CanDispatcher:: PC_BUFFER_SIZE = 25;



//==============================================================================
// C / D T O R   S E C T I O N

  CanDispatcher::CanDispatcher(uint32_t device_id, uint32_t unique_id, uint32_t chan,
                               uint32_t baudrate, uint32_t loop_rate):
      can_driver_(chan, baudrate),
      main_thread_(&CanDispatcher::MainCanProcess, this)
  {
    discovery_tries_ = 0;
    loop_rate_ = loop_rate;

    master_id_ = (device_id << DEVICE_ID_POSITION) | (unique_id << UNIQUE_ID_POSITION);

    can_driver_.GetErrorCount(&tx_error_, &rx_error_, &ovrr_error_);

    clock_gettime(CLOCK_REALTIME, &actual_time_);
    clock_gettime(CLOCK_REALTIME, &initial_time_);
    clock_gettime(CLOCK_REALTIME, &id_req_time_);
    clock_gettime(CLOCK_REALTIME, &error_recovery_);


    can_driver_.FlushRxBuffer();
    can_driver_.FlushTxBuffer();

    ListDevices();



    // GetAllDevicesParamsReq(); // TODO: uncomment when implemented in ELE part
  }

//------------------------------------------------------------------------------
//
  CanDispatcher::~CanDispatcher() {

  }

//==============================================================================
// M E T H O D S   S E C T I O N

  canStatus CanDispatcher::ListDevices() {
    canStatus status;

    status = SendIdRequest();      // Ask ID from every device on CAN bus
    if (status < canOK)
      return status;

    usleep(ID_REQ_WAIT);  // Wait for all responses
    status = ReadMessages();       // Reads all messages into rx_raw_buffer_
    if (status < canOK)
      return status;

    // For each messages received during sleep,
    for (size_t j = 0; j < rx_raw_buffer_.size(); j++) {
      CanDeviceStruct new_device;
      // If the address of the message has never been seen
      if(FindDeviceWithAddress(rx_raw_buffer_[j].id) == SONIA_DEVICE_NOT_PRESENT){
    	//Apending new device to the vector
    	new_device.global_address =
    	      	          (rx_raw_buffer_[j].id & DEVICE_MAC_MASK);

    	devices_list_.push_back(new_device);
      }
    }

    DispatchMessages();  // Dispatch and saves all received messages

    unknown_addresses_table_.clear();// resets unknown addresses discovery.

    return status;
  }

//------------------------------------------------------------------------------
//
  void CanDispatcher::DispatchMessages() {
    SoniaDeviceStatus status;
    size_t index;

    // For each message contained in raw buffer

    for (size_t j = 0; j < rx_raw_buffer_.size(); j++) {
      // get the device_list index where address is located
      status = FindDeviceWithAddress(rx_raw_buffer_[j].id, &index);

      if (status != SONIA_DEVICE_NOT_PRESENT) {  // If device exists
        // If the ID received correspond to an ID request response
        // saves device's parameters.
        if (devices_list_[index].global_address == rx_raw_buffer_[j].id) {
          devices_list_[index].device_properties.firmware_version =
            (rx_raw_buffer_[j].data[1] << 8) |
            rx_raw_buffer_[j].data[0];

          devices_list_[index].device_properties.uc_signature =
            (rx_raw_buffer_[j].data[4] << 16) |
            (rx_raw_buffer_[j].data[3] << 8) |
            rx_raw_buffer_[j].data[2];

          devices_list_[index].device_properties.capabilities =
            rx_raw_buffer_[j].data[5];

          devices_list_[index].device_properties.device_data =
            rx_raw_buffer_[j].data[6];
        }
          // If the ID received correspond to a device fault
        else if ((devices_list_[index].global_address | DEVICE_FAULT) ==
                 rx_raw_buffer_[j].id) {
          devices_list_[index].device_fault = true;
          devices_list_[index].fault_message = rx_raw_buffer_[j].data;

          // printing fault received
          printf("\n\rDevice %X: Fault %C%C%C%C%C%C%C%C", devices_list_[index]
              .global_address,rx_raw_buffer_[j].data[0],rx_raw_buffer_[j]
              .data[1],rx_raw_buffer_[j].data[2],rx_raw_buffer_[j]
              .data[3],rx_raw_buffer_[j].data[4],rx_raw_buffer_[j]
              .data[5],rx_raw_buffer_[j].data[6],rx_raw_buffer_[j]
              .data[7]);


        }
          // If the ID received corresponds to a ping response
        else if ((devices_list_[index].global_address | PING) ==
                                                       rx_raw_buffer_[j].id) {
          devices_list_[index].ping_response = true;
        }
          // If the ID received corresponds to a parameter response
        else if ((devices_list_[index].global_address | GET_PARAM_REQ) ==
                                                       rx_raw_buffer_[j].id) {
          devices_list_[index].device_parameters[0] = rx_raw_buffer_[j].data[0]
                                                      | rx_raw_buffer_[j].data[1] << 8
                                                      | rx_raw_buffer_[j].data[2] << 16
                                                      | rx_raw_buffer_[j].data[3] << 24;
          devices_list_[index].device_parameters[1] = rx_raw_buffer_[j].data[4]
                                                      | rx_raw_buffer_[j].data[5] << 8
                                                      | rx_raw_buffer_[j].data[6] << 16
                                                      | rx_raw_buffer_[j].data[7] << 24;
        }
          // If the ID received correspond to any other message
        else if (devices_list_[index].global_address ==
                 (rx_raw_buffer_[j].id & DEVICE_MAC_MASK)) {
          // Avoids buffer overflow
          if (devices_list_[index].rx_buffer.size() >= DISPATCHED_RX_BUFFER_SIZE) {
            devices_list_[index].rx_buffer.clear();
            printf("\n\rDevice %X: Buffer Overflow. You must empty it faster.",
                   devices_list_[index].global_address);
          }

          // Saves message into dispatched devices' buffers.
          devices_list_[index].rx_buffer.push_back(rx_raw_buffer_[j]);
        }

      } else {  // If address is unknown
        // Adds the address to the unknown addresses table
        AddUnknownAddress(rx_raw_buffer_[j].id);
      }
    }
    rx_raw_buffer_.clear();
  }

//------------------------------------------------------------------------------
//
  canStatus CanDispatcher::ReadMessages() {
    canStatus status;
    status = can_driver_.ReadAllMessages(rx_raw_buffer_);

    return status;
  }

//------------------------------------------------------------------------------
//
  canStatus CanDispatcher::SendMessages() {
    canStatus status;

	status =
        can_driver_.WriteBuffer(tx_raw_buffer_, CAN_SEND_TIMEOUT);

	tx_raw_buffer_.clear();
    return status;
  }

//------------------------------------------------------------------------------
//
  canStatus CanDispatcher::SendIdRequest() {
    CanMessage msg;

    msg.id = 0;
    msg.data[0] = 0;
    msg.dlc = 0;
    msg.flag = canMSG_EXT;
    msg.time = 0;

    return (can_driver_.WriteMessage(msg, 1));
  }

//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::SetPollRate(uint8_t device_id,
                                               uint8_t unique_id,
                                               uint16_t poll_rate) {
    SoniaDeviceStatus status;
    size_t index;

    status = FindDevice(device_id, unique_id, &index);

    if (status != SONIA_DEVICE_NOT_PRESENT) {
      devices_list_[index].device_properties.poll_rate = poll_rate;
    }

    return status;
  }

//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::FetchMessages(uint8_t device_id,
                                                 uint8_t unique_id,
                                                 std::vector<CanMessage> &buffer) {
    SoniaDeviceStatus status;
    size_t index;

    status = FindDevice(device_id, unique_id, &index);

    if (status != SONIA_DEVICE_NOT_PRESENT) {
      buffer = devices_list_[index].rx_buffer;
      devices_list_[index].rx_buffer.clear();
    }
    else{
      buffer.clear();
    }

    return status;
  }

//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::GetDevicesProperties(
    uint8_t device_id, uint8_t unique_id, DeviceProperties &properties) {
    SoniaDeviceStatus status;
    size_t index;

    // The following lines will allow to return nothing if device isn't present
    properties.capabilities = 0;
    properties.device_data = 0;
    properties.firmware_version = 0;
    properties.poll_rate = 0;
    properties.uc_signature = 0;

    status = FindDevice(device_id, unique_id, &index);

    if (status != SONIA_DEVICE_NOT_PRESENT)
      properties = devices_list_[index].device_properties;

    return status;
  }

//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::PushUnicastMessage(uint8_t device_id,
                                                      uint8_t unique_id,
                                                      uint16_t message_id,
                                                      uint8_t *buffer, uint8_t ndata) {

    CanMessage message;

    if(ndata > 8) // DLC cannot be larger than 8!!
      ndata = 8;

    message.id = UNICAST | (device_id << DEVICE_ID_POSITION) |
                 (unique_id << UNIQUE_ID_POSITION) | (message_id & 0x0FFF);
    message.flag = canMSG_EXT;
    message.dlc = ndata;
    for (int i = 0; i < ndata; i++) message.data[i] = buffer[i];

    tx_raw_buffer_.push_back(message);

    return FindDevice(device_id, unique_id);
  }

//------------------------------------------------------------------------------
//
  void CanDispatcher::PushBroadMessage(uint16_t message_id,
                                       uint8_t *buffer, uint8_t ndata) {

    CanMessage message;

    if(ndata > 8) // DLC cannot be larger than 8!!
      ndata = 8;

    message.id = master_id_ | (message_id & 0x0FFF);
    message.flag = canMSG_EXT;
    message.dlc = ndata;
    for (int i = 0; i < ndata; i++) message.data[i] = buffer[i];

    tx_raw_buffer_.push_back(message);
  }

//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::FindDevice(uint8_t device_id,
                                                  uint8_t unique_id, size_t *index) {
    uint32_t global_address =
      (device_id << DEVICE_ID_POSITION) | (unique_id << UNIQUE_ID_POSITION);

    return FindDeviceWithAddress(global_address, index);
  }
//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::FindDevice(uint8_t device_id,
												uint8_t unique_id) {
	uint32_t global_address =
      (device_id << DEVICE_ID_POSITION) | (unique_id << UNIQUE_ID_POSITION);

    return FindDeviceWithAddress(global_address);
  }

//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::FindDeviceWithAddress(uint32_t address, size_t *index) {
    SoniaDeviceStatus status = SONIA_DEVICE_NOT_PRESENT;

    address = address & DEVICE_MAC_MASK;

    // predicate used for seeking for a device in device_list_ vector
    auto add_search_pred = [address](const CanDeviceStruct &device){
        return device.global_address == address;
    };

    auto vec_it = std::find_if(devices_list_.begin(),devices_list_.end(),
    				add_search_pred);

    // Seeking for specified address in device_list_
	if(vec_it != devices_list_.end()){
	  // Calculating the exact index
      *index = std::distance(devices_list_.begin(), vec_it);

	  if (devices_list_[*index].device_fault == true) {
		status = SONIA_DEVICE_FAULT;
	  } else {
		status = SONIA_DEVICE_PRESENT;
	  }
	}

    if (status == SONIA_DEVICE_NOT_PRESENT){
    	AddUnknownAddress(address);
    }

    return status;
  }
//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::FindDeviceWithAddress(uint32_t address) {
    SoniaDeviceStatus status = SONIA_DEVICE_NOT_PRESENT;

    address = address & DEVICE_MAC_MASK;

    // predicate used for seeking for a device in device_list_ vector
    auto add_search_pred = [address](const CanDeviceStruct &device){
    	return device.global_address == address;
    };

    // index of the device found
    size_t index;

    // Seeking for specified address in device_list_
    auto vec_it = std::find_if(devices_list_.begin(),devices_list_.end(),
    					add_search_pred);

    // If a device is found
    if(vec_it != devices_list_.end()){
      // Calculating the exact index
      index = std::distance(devices_list_.begin(), vec_it);

      // Look for device_fault
  	  if (devices_list_[index].device_fault == true) {
  		status = SONIA_DEVICE_FAULT;
  	  } else {
  		status = SONIA_DEVICE_PRESENT;
  	  }
  	}

    if (status == SONIA_DEVICE_NOT_PRESENT){
    	AddUnknownAddress(address);
    }

      return status;
  }

//------------------------------------------------------------------------------
//
/*  void CanDispatcher::PollDevices() {

    uint32_t actual_time_ms = actual_time_.tv_nsec / 1000000;
    uint32_t initial_time_ms = initial_time_.tv_nsec / 1000000;

    for (size_t i = 0; i < devices_list_.size(); i++) {
      if (((actual_time_ms - initial_time_ms) %
           devices_list_[i].device_properties.poll_rate) < ((1.0 /
                                                             (float) loop_rate_)
                                                            * 1000.0)) {
        SendRTR(devices_list_[i].global_address);
      }
    }
  }*/

//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::GetDeviceFault(uint8_t device_id,
                                                  uint8_t unique_id, uint8_t *&fault) {
    SoniaDeviceStatus status;
    size_t index;
    status = FindDevice(device_id, unique_id, &index);

    if (status != SONIA_DEVICE_NOT_PRESENT) {
      devices_list_[index].device_fault = false;
      fault = devices_list_[index].fault_message;
    }

    return status;
  }

//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::SendResetRequest(uint8_t device_id,
                                                    uint8_t unique_id) {
    uint8_t *msg;

    return (PushUnicastMessage(device_id, unique_id, RESET_REQ, msg, RESET_REQUEST_DLC));
  }

//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::SendSleepRequest(uint8_t device_id,
                                                    uint8_t unique_id) {
    uint8_t *msg;

    return (PushUnicastMessage(device_id, unique_id, SLEEP_REQ, msg, SLEEP_REQUEST_DLC));
  }


//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::PingDevice(uint8_t device_id,
                                              uint8_t unique_id) {
    uint8_t *msg;

    return (PushUnicastMessage(device_id, unique_id, PING, msg, PING_REQUEST_DLC));
  }

//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::VerifyPingResponse(uint8_t device_id,
                                                      uint8_t unique_id, bool *response) {
    SoniaDeviceStatus status;
    size_t index;
    status = FindDevice(device_id, unique_id, &index);

    if (status != SONIA_DEVICE_NOT_PRESENT)
      *response = devices_list_[index].ping_response;
    else
      *response = false;

    devices_list_[index].ping_response = false;

    return status;
  }

//------------------------------------------------------------------------------
//
  SoniaDeviceStatus CanDispatcher::SendWakeUpRequest(uint8_t device_id,
                                                     uint8_t unique_id) {
    uint8_t *msg;

    return (PushUnicastMessage(device_id, unique_id, WAKEUP_REQ, msg, WAKEUP_REQUEST_DLC));
  }

//------------------------------------------------------------------------------
//
  void CanDispatcher::SendRTR(uint32_t address) {
    CanMessage msg;

    msg.id = address;
    msg.data[0] = 0;
    msg.dlc = 0;
    msg.flag = canMSG_RTR;
    msg.time = 0;

    can_driver_.WriteMessage(msg, 1);
  }

//------------------------------------------------------------------------------
//
  uint8_t CanDispatcher::GetNumberOfDevices() {
    return devices_list_.size();
  }

//------------------------------------------------------------------------------
//
  void CanDispatcher::GetUnknownAddresses(std::vector<uint32_t> &addresses) {
    addresses = unknown_addresses_table_;
  }

//------------------------------------------------------------------------------
//
  void CanDispatcher::AddUnknownAddress(uint32_t address) {

    if(std::find(unknown_addresses_table_.begin(),
    		unknown_addresses_table_.end(), address & DEVICE_MAC_MASK) ==
    				unknown_addresses_table_.end()){

    	unknown_addresses_table_.push_back(address);
    }
  }

//------------------------------------------------------------------------------
//
  void CanDispatcher::MainCanProcess() {
    canStatus status;

    while(1) {
      clock_gettime(CLOCK_REALTIME, &actual_time_);


      // verifying CAN errors
      if (tx_error_ >= 50 || rx_error_ >= 50) {
        printf(
            "\rToo many errors encountered (tx: %d, rx: %d, ovrr: %d). Verify "
                "KVaser connectivity. "
                "Stopping "
                "CAN until problem is "
                "solved",
            tx_error_,
            rx_error_,
            ovrr_error_);

        if ((actual_time_.tv_sec - error_recovery_.tv_sec)
            >= ERROR_RECOVERY_DELAY) {
          printf("\n\rRetrying a recovery\n");

          error_recovery_ = actual_time_;
          can_driver_.GetErrorCount(&tx_error_, &rx_error_, &ovrr_error_);
        }
      }
        // normal process
      else {

        // reading from CAN bus
        status = ReadMessages();

        // verifying errors
        if (status < canOK) {
          printf("\n\r");
          can_driver_.PrintErrorText(status);
          can_driver_.GetErrorCount(&tx_error_, &rx_error_, &ovrr_error_);
          printf("tx: %d, rx: %d, ovrr: %d", tx_error_, rx_error_,
                 ovrr_error_);
        }

        // Dispatching read messages
        DispatchMessages();

        // sending messages contained in tx_buffer
        status = SendMessages();

        // verifying errors
        if (status < canOK) {
          printf("\n\r");
          can_driver_.PrintErrorText(status);
          can_driver_.GetErrorCount(&tx_error_, &rx_error_, &ovrr_error_);
          printf("tx: %d, rx: %d, ovrr: %d", tx_error_, rx_error_, ovrr_error_);
        }

        // Sends RTR to devices if asked
        // PollDevices();  // TODO: uncomment when implemented in ELE part

        // verifying if devices where not found. if so, sends ID requests to try
        // to find undiscovered devices.
        if (discovery_tries_ <= DISCOVERY_TRIES &&
            (actual_time_.tv_sec - id_req_time_.tv_sec) >= DISCOVERY_DELAY &&
            unknown_addresses_table_.size() != 0) {
          printf("\n\rUnknown devices found. Retrying ID Request");
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


bool CanDispatcher::CallDeviceMethod(Mreq &req,
                                     provider_can::Response &res){
  SoniaDeviceStatus status;
  ComputerMessage msg = {
    msg.method_number = req.method_number,
    msg.parameter_value = req.parameter_value
  };
  size_t index;
  status = FindDevice(req.device_id, req.unique_id, &index);

  if(status != SONIA_DEVICE_NOT_PRESENT)
    if(devices_list_[index].pc_messages_buffer.size() <= PC_BUFFER_SIZE)
      devices_list_[index].pc_messages_buffer.push_back(msg);
    else
      printf("\n\rDevice %X: Dedicated ROS service called to fast for the "
               "update rate. Following messages will be dropped.",
             devices_list_[index].global_address);

  res.device_status = status;
  return true;
}
//------------------------------------------------------------------------------
//

  SoniaDeviceStatus CanDispatcher::FetchComputerMessages(uint8_t device_id,
                                                 uint8_t unique_id,
                                                 std::vector<ComputerMessage> &buffer) {
    SoniaDeviceStatus status;
    size_t index;

    status = FindDevice(device_id, unique_id, &index);

    if (status != SONIA_DEVICE_NOT_PRESENT) {
      buffer = devices_list_[index].pc_messages_buffer;
      devices_list_[index].pc_messages_buffer.clear();
    }
    else{
      buffer.clear();
    }

    return status;
  }

//------------------------------------------------------------------------------
//
 /* SoniaDeviceStatus CanDispatcher::SetDeviceParameterReq(uint8_t device_id,
                                                         uint8_t unique_id,
                                                         uint8_t param_number,
                                                         uint32_t param_value) {
    uint8_t *msg;

    msg[0] = param_number;
    msg[1] = (uint8_t)(param_value >> 24);
    msg[2] = (uint8_t)(param_value >> 16);
    msg[3] = (uint8_t)(param_value >> 8);
    msg[4] = (uint8_t)(param_value);

    return (pushUnicastMessage(device_id, unique_id, SET_PARAM_REQ, msg, SET_PARAMETER_DLC));
  }*/

//------------------------------------------------------------------------------
//
  /*SoniaDeviceStatus CanDispatcher::GetDeviceParameterReq(uint8_t device_id,
                                                         uint8_t unique_id) {
    uint8_t *msg;

    return (pushUnicastMessage(device_id, unique_id, GET_PARAM_REQ, msg, SET_PARAMETER_DLC));
  }*/

//------------------------------------------------------------------------------
//
 /* void CanDispatcher::GetAllDevicesParamsReq(void) {

    for (size_t i = 0; i < devices_list_.size(); i++)
      GetDeviceParameterReq(devices_list_[i].global_address >> 20, devices_list_[i].global_address >> 12);
  }*/

//------------------------------------------------------------------------------
//
  /*SoniaDeviceStatus CanDispatcher::GetDeviceParams(uint8_t device_id,
                                                   uint8_t unique_id,
                                                   uint32_t *&params) {
    size_t index;
    SoniaDeviceStatus status;

    status = FindDevice(device_id, unique_id, &index);

    if (status != SONIA_DEVICE_NOT_PRESENT)
      params = devices_list_[index].device_parameters;

    return status;
  }*/

}
