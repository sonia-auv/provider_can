/**
 * \file	can_dispatcher.h
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

#ifndef PROVIDER_CAN_CAN_DISPATCHER_H_
#define PROVIDER_CAN_CAN_DISPATCHER_H_

#include "can_driver.h"
#include <stdint.h>
#include <sys/times.h>
#include "can_def.h"

namespace provider_can {

//============================================================================
// T Y P E D E F   A N D   E N U M

// Delay after an ID request before listing devices
const uint32_t ID_REQ_WAIT = 100000;

// Number of ID request retries
const uint8_t DISCOVERY_TRIES = 10;
// Delay between ID request retries (sec)
const uint8_t DISCOVERY_DELAY = 5;
// Delay between error recovery retries
const uint8_t ERROR_RECOVERY_DELAY = 2;
// Delay to wait for a message to be sent (ms)
const uint32_t CAN_SEND_TIMEOUT = 10;

// Table sizes
const int MAX_NUM_OF_DEVICES = 30;
const int RAW_TX_BUFFER_SIZE = 25;
const int DISPATCHED_RX_BUFFER_SIZE = 50;


typedef struct {
  uint16_t firmware_version;
  // Signature of LPC chip
  uint32_t uc_signature;
  // may be RESET, SLEEP, WAKEUP or ISP
  uint8_t capabilities;
  // any data
  uint8_t device_data;
  // RTR send rate, in ms
  uint16_t poll_rate = 500;
} DeviceProperties;

typedef struct {
  uint32_t num_of_messages = 0;
  CanMessage *buffer;
} RawBuffer;

typedef struct {
  // device global address (ex: 0x00602000 for PSU)
  uint32_t global_address;
  DeviceProperties device_properties;

  // number of messages contained in the rx buffer
  uint8_t num_of_messages;
  CanMessage rx_buffer[DISPATCHED_RX_BUFFER_SIZE];

  // if device has sent a fault
  bool device_fault = false;
  uint8_t *fault_message;

  // If device answered from a ping request
  bool ping_response = false;
} CanDevice;

typedef enum {
  SONIA_DEVICE_NOT_PRESENT = 0,
  SONIA_DEVICE_PRESENT = 1,
  SONIA_DEVICE_FAULT = -1
} SoniaDeviceStatus;

class CanDispatcher {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  //! Constructor
  // param loop_rate main process loop rate. Must be the same as ROS loop_rate
  // param device_id PC ID
  // param unique_id PC ID
  CanDispatcher(uint32_t device_id, uint32_t unique_id, uint32_t chan,
                uint32_t baudrate, uint32_t loop_rate);

  // Destructor
  ~CanDispatcher();

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
  * The function sets the poll_rate value of the DeviceProperties struct of the
  * selected device.
  *
  * This indicates how many times per second we want an RTR to be sent to the
  * device.
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \param poll_rate polling rate, in ms. max: 1 sec.
  * \return SoniaDeviceStatus enum
  */
  SoniaDeviceStatus setPollRate(uint8_t device_id, uint8_t unique_id,
                                uint16_t poll_rate);

  /**
  * The next functions sends SONIA specific messages to selected device
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \return SoniaDeviceStatus enum
  */
  SoniaDeviceStatus sendResetRequest(uint8_t device_id, uint8_t unique_id);
  SoniaDeviceStatus sendSleepRequest(uint8_t device_id, uint8_t unique_id);
  SoniaDeviceStatus sendWakeUpRequest(uint8_t device_id, uint8_t unique_id);

  SoniaDeviceStatus pingDevice(uint8_t device_id, uint8_t unique_id);// TODO: To be tested
  SoniaDeviceStatus verifyPingResponse(uint8_t device_id, uint8_t unique_id, bool *response);

  /**
  * This function allows to send a message to a specific device.
  *
  * The function verifies if the address of the message is a known device.
  * SoniaDeviceStatus will
  * indicate if so. If it is not the case, the message will be sent anyway.
  *
  * \param device_id SONIA Device ID
  * \param unique_id SONIA unique ID
  * \param message_id SONIA message ID
  * \param buffer message content
  * \param ndata message length
  * \return SoniaDeviceStatus enum
  */
  SoniaDeviceStatus pushUnicastMessage(uint8_t device_id, uint8_t unique_id,
                                uint16_t message_id, uint8_t *buffer,
                                uint8_t ndata);

  /**
  * The function allows to send broadcast messages using PC address. The messages
  * are not related to specific devices, but to anyone who reads it.
  *
  * \param message_id SONIA message ID
  * \param buffer message content
  * \param ndata message length
  */
  void pushBroadMessage(uint16_t message_id, uint8_t *buffer,// TODO: to be tested
                                         uint8_t ndata);

  /**
  * The function returns the rx_buffer of the selected device
  *
  * The rx_buffer returned contains all received messages since last call of
  * this function.
  * If the device asked does not exist, SoniaDeviceStatus will indicate it and
  * no rx_buffer will be returned.
  * rx_buffer will only contain data messages, not ID request responses nor
  * device fault messages. These are
  * filtered by dispatchMessages().
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \param buffer device's rx_buffer
  * \param num_of_messages number of messages read
  * \return SoniaDeviceStatus enum
  */
  SoniaDeviceStatus fetchMessages(uint8_t device_id, uint8_t unique_id,
                                  CanMessage *&buffer,
                                  uint8_t *num_of_messages);

  /**
  * The function returns the devices's properties
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \param properties device's properties
  * \return SoniaDeviceStatus enum
  */
  SoniaDeviceStatus getDevicesProperties(uint8_t device_id, uint8_t unique_id,
                                         DeviceProperties *properties);

  /**
  * The function clears the specified device fault flag and returns the
  * value of the fault.
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \return SoniaDeviceStatus enum
  */
  SoniaDeviceStatus getDeviceFault(uint8_t device_id, uint8_t unique_id, uint8_t *&fault);// TODO: To be tested

  uint8_t getNumberOfDevices();
  uint8_t getUnknownAddresses(uint32_t *&addresses);

  /**
  * This process has to be called periodically. It handles reading and sending
  * messages.
  *
  */
  void providerCanProcess();

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  /**
  * Allows the user to send an ID request to devices on SONIA's CAN bus.
  *
  * Every device will answer its ID and its parameters after receiving the
  * ID request. This function does not handle the responses to the ID request.
  *
  */
  canStatus sendIdRequest();

  /**
  * Sends RTR to devices at rate specified in DeviceProperties struct, for every
  * known device.
  *
  * This function loops through all known device and compares the poll_rate with
  * a timer value
  * to known if polling is required or not. use setPollRate() to specify a poll
  * rate for a
  * specific device. Default poll_rate is 100ms.
  *
  */
  void pollDevices();  // TODO: la fonctionnalité RTR devra être implémentée
                       // dans l'élé du sub

  /**
  * Sends an ID request on CAN bus and list all addresses received.
  *
  * This function creates a CanDevice struct for each device that answered to
  * the
  * ID request. this allows the users of the class to read messages from the
  * devices
  * that answered. If a device did not answer to the ID request, no struct will
  * be created
  * for it and if it sends other messages, they will be dropped.
  */
  canStatus listDevices();

  /**
  * Reads all messages received on CAN bus and stores them into rx_raw_buffer_
  */
  canStatus readMessages();

  /**
  * sends all messages contained in tx_raw_buffer_
  */
  canStatus sendMessages();

  /**
  * Dispatch all messages contained into rx_raw_buffer_ to each respective
  * CanDevice struct created by listDevices().
  * This function also filters ID_request responses and device_fault
  * messages to set DeviceProperties struct.
  */
  void dispatchMessages();

  /**
  * The function returns the device_list_ index value which contains the
  * selected address/device
  *
  * \param address address to look for
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \param index device_list_ index found
  * \return SoniaDeviceStatus enum
  */
  SoniaDeviceStatus getDeviceIndex(uint8_t device_id, uint8_t unique_id,
                                   int *index);
  SoniaDeviceStatus getAddressIndex(uint32_t address, int *index);

  /**
  * The function sends and RTR on CAN bus with selected address
  *
  * \param address ID of the RTR
  */
  void sendRTR(uint32_t address);

  /**
  * Adds an address to the unknown addresses table. If unknown
  * addresses are found, providerCanProcess will send ID requests to
  * be sure every present device are known.
  *
  * \param address
  */
  void addUnknownAddress(uint32_t address);

  //============================================================================
  // P R I V A T E   M E M B E R S

  CanDevice
      devices_list_[MAX_NUM_OF_DEVICES];  // List of devices present on CAN bus

  uint32_t unknown_addresses_table_[MAX_NUM_OF_DEVICES];
  uint8_t nunknown_addresses_;

  RawBuffer rx_raw_buffer_;  // Buffer directly taken from KVaser
  RawBuffer tx_raw_buffer_;

  uint8_t ndevices_present_;  // Number of devices detected

  CanDriver canDriver_;  // Can communication object

  timespec ticks_per_sec_;
  timespec actual_time_;
  timespec initial_time_;
  timespec id_req_time_;
  timespec error_recovery_;

  uint8_t discovery_tries_;

  uint32_t loop_rate_;  // main process loop rate

  uint32_t tx_error_;
  uint32_t rx_error_;
  uint32_t ovrr_error_;

  uint32_t master_id_;  // PC ID
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DISPATCHER_H_
