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
// TODO: Ajouter la fonctionnalité set et get_parameter

#ifndef PROVIDER_CAN_CAN_DISPATCHER_H_
#define PROVIDER_CAN_CAN_DISPATCHER_H_

#include <stdint.h>
#include <sys/times.h>
#include <vector>
#include <memory>
#include "provider_can/can_driver.h"
#include "provider_can/can_def.h"

namespace provider_can {

//============================================================================
// T Y P E D E F   A N D   E N U M

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
  uint32_t device_parameters[2];// TODO: Not yet implemented in ELE part

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


/**
 * The CanDispatcher class is the core of CAN communication. It handles all
 * the SONIA's can standard and dispatch messages for future processing. This
 * class lists every device present on can bus at startup and ensure that
 * every message from every device is correctly processed. some messages such as
 * ping responses or ID requests are processed directly in this class.
 *
 * IMPORTANT NOTE: Almost every function of this class returns SoniaDeviceStatus
 * enum. This notifies the user if the requested device exists or not or encountered
 * a fault. You must always verify the value of this return parameter to know if
 * your device is present or not. CanDispatcher will handle sending ID requests
 * to try to find an undiscovered device if it finds that something is wrong.
 *
 * How to use:
 * 1. Call the constructor specifying PC's device and unique ID, CAN channel, baudrate
 * and loop_rate (which is the number of times per second the main process is called).
 *
 * NOTE: existing Device and unique IDs are listed in can_def.h
 *
 * 2. Pass a pointer to the CanDispatcher object to every other class that
 * may want to read messages. example:
 *
    ros::Rate loop_rate(10);
    provider_can::CanDispatcher canD(controllers,on_board_pc,0, BAUD_125K, 10);
    provider_can::BottomLight bottom_light(&canD); <<<<<<<<<<<<<<<<<
 *
 *
 * 3. In the main loop, call mainCanProcess() in order to handle sending and
 * receiving messages. example:
 *
 *
    while (ros::ok()) {
      canD.mainCanProcess();
      ...
    }

 * 4. To read messages from can, call fetchMessages(). the parameters of this
 * function allows to select from which device we want to read messages received.
 *
 * 5. To send message to a specific device, call pushUnicastMessage().
 *
 * 6. To send messages using general PC address (set in the constructor),
 * use pushBroadMessage(). These messages are not related to specific devices.
 * Every device on CAN by may read it.
 *
 * 7. getDevicesProperties allows to retrieve some properties of a specific device.
 *
 * 8. Can Devices may have permanent parameters set in their flash memory. set and
 * getDeviceParams allows to read or modify these parameters. A list of possible
 * parameters is found in can_def.h.
 *
 */

class CanDispatcher {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CanDispatcher>;

  // Delay after an ID request before listing devices
  static const uint32_t ID_REQ_WAIT;

  // Number of ID request retries
  static const uint8_t DISCOVERY_TRIES;
  // Delay between ID request retries (sec)
  static const uint8_t DISCOVERY_DELAY;
  // Delay between error recovery retries
  static const uint8_t ERROR_RECOVERY_DELAY;
  // Delay to wait for a message to be sent (ms)
  static const uint32_t CAN_SEND_TIMEOUT;

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
  SoniaDeviceStatus SetPollRate(uint8_t device_id, uint8_t unique_id,
                                uint16_t poll_rate);

  /**
  * The next functions sends SONIA specific messages to selected device
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \return SoniaDeviceStatus enum
  */
  SoniaDeviceStatus SendResetRequest(uint8_t device_id, uint8_t unique_id);
  SoniaDeviceStatus SendSleepRequest(uint8_t device_id, uint8_t unique_id);
  SoniaDeviceStatus SendWakeUpRequest(uint8_t device_id, uint8_t unique_id);

  SoniaDeviceStatus PingDevice(uint8_t device_id, uint8_t unique_id);// TODO: To be tested
  SoniaDeviceStatus VerifyPingResponse(uint8_t device_id, uint8_t unique_id, bool *response);

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
  SoniaDeviceStatus PushUnicastMessage(uint8_t device_id, uint8_t unique_id,
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
  void PushBroadMessage(uint16_t message_id, uint8_t *buffer,// TODO: to be tested
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
  SoniaDeviceStatus FetchMessages(uint8_t device_id, uint8_t unique_id,
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
  SoniaDeviceStatus GetDevicesProperties(uint8_t device_id, uint8_t unique_id,
                                         DeviceProperties *properties);

  /**
  * The function clears the specified device fault flag and returns the
  * value of the fault.
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \return SoniaDeviceStatus enum
  */
  SoniaDeviceStatus GetDeviceFault(uint8_t device_id, uint8_t unique_id, uint8_t *&fault);// TODO: To be tested

  uint8_t GetNumberOfDevices();
  uint8_t GetUnknownAddresses(uint32_t *&addresses);

  /**
  * This process has to be called periodically. It handles reading and sending
  * messages.
  *
  */
  void MainCanProcess();


  /**
  * This allows the user to set permanent parameters to can devices. These parameters
  * are saved in device's flash memory and their value will be kept until it is
  * manually changed.
  *
  * the list of settable parameters is shown in can_def.h
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \param param_number use device parameters enums contained in can_def.h
  * \param param_value value to set to the parameter
  * \return SoniaDeviceStatus enum
  */
  // TODO: Not yet implemented in ELE part
  SoniaDeviceStatus SetDeviceParameterReq(uint8_t device_id, uint8_t unique_id,
                                          uint8_t param_number, uint32_t param_value);
  /**
  * This allows the user to read the permanent parameters set in devices'
  * flash memory.
  *
  * The list of readable parameters for each device is shown in can_def.h
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \param params parameters table.
  * \return SoniaDeviceStatus enum
  */
  // TODO: not yet implemented in ELE part
  SoniaDeviceStatus GetDeviceParams(uint8_t device_id, uint8_t unique_id,
                                    uint32_t *&params);



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
  canStatus SendIdRequest();

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
  void PollDevices();  // TODO: la fonctionnalité RTR devra être implémentée
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
  canStatus ListDevices();

  /**
  * Reads all messages received on CAN bus and stores them into rx_raw_buffer_
  */
  canStatus ReadMessages();

  /**
  * sends all messages contained in tx_raw_buffer_
  */
  canStatus SendMessages();

  /**
  * Dispatch all messages contained into rx_raw_buffer_ to each respective
  * CanDevice struct created by listDevices().
  * This function also filters ID_request responses and device_fault
  * messages to set DeviceProperties struct.
  */
  void DispatchMessages();

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
  SoniaDeviceStatus GetDeviceIndex(uint8_t device_id, uint8_t unique_id,
                                   int *index);
  SoniaDeviceStatus GetAddressIndex(uint32_t address, int *index);

  /**
  * The function sends and RTR on CAN bus with selected address
  *
  * \param address ID of the RTR
  */
  void SendRTR(uint32_t address);

  /**
  * Adds an address to the unknown addresses table. If unknown
  * addresses are found, providerCanProcess will send ID requests to
  * be sure every present device are known.
  *
  * \param address
  */
  void AddUnknownAddress(uint32_t address);

  /**
  * Allows to read permanent parameters set in the devices
  */
  // TODO: not yet implemented in ELE part
  void GetAllDevicesParamsReq(void);
  SoniaDeviceStatus GetDeviceParameterReq(uint8_t device_id, uint8_t unique_id);

  //============================================================================
  // P R I V A T E   M E M B E R S

  // TODO: change table to vector
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
