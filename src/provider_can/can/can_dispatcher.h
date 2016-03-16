/**
 * \file	can_dispatcher.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	07/12/2015
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

// TODO: add set and GetParameter functionnalities

#ifndef PROVIDER_CAN_CAN_DISPATCHER_H_
#define PROVIDER_CAN_CAN_DISPATCHER_H_

#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/runnable.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <stdint.h>
#include <sys/times.h>
#include <algorithm>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/runnable.h>
#include "sonia_msgs/SendCanMessage.h"
#include "usb_canII.h"
#include "can_driver.h"
#include <thread>
#include <vector>
#include "provider_can/can/can_driver.h"
#include "provider_can/can_def.h"

namespace provider_can {

//============================================================================
// T Y P E D E F   A N D   E N U M

// max number of data contained in rx buffer before overflow notification
const int DISPATCHED_RX_BUFFER_SIZE = 50;

typedef struct {
  // The number attributed to the method to be accessed
  uint8_t method_number;
  // the value to pass as a parameter to the method
  double parameter_value;
  // String to pass for additionnal parameters
  std::string string_param;
} ComputerMessage;

// This is a base structure which will be filled of
// one device's messages buffers. One struct is initialized
// For each device found.
typedef struct {
  // device's global address (ex: 0x00602000 for PSU)
  uint32_t global_address;

  std::vector<CanMessage> can_rx_buffer;
  std::vector<ComputerMessage> pc_messages_buffer;

  // RTR send rate, in ms
  uint16_t poll_rate = 500;

} CanDeviceBuffers;

typedef enum {
  SONIA_DEVICE_NOT_PRESENT = 0,
  SONIA_DEVICE_PRESENT = 1,
} SoniaDeviceStatus;

/**
 * This class dispatches can messages into buffers. Each device
 * found on can bus will have its own messages buffer initialized.
 *
 * IMPORTANT NOTE: Almost every function of this class returns
 * SoniaDeviceStatus
 * enum. This notifies the user if the requested device exists.
 * You must always verify the value of this return parameter to know
 * if your device is present or not. CanDispatcher will handle sending
 * ID requests to try to find an undiscovered device if it finds
 * that something is wrong.
 *
 * See Run() method to know what is done by this class. This method is set as
 * a thread.
 *
 * Informations:
 *
 * 1. CallDeviceMethod will store in the corresponding device structure
 * messages sent by other ROS processes. It is a service in itself.
 *
 * 2. To read messages from can, call fetchMessages(). the parameters of this
 * function allows to select from which device we want to read messages
 received.
 *
 * 3. To send message to a specific device, call pushUnicastMessage().
 *
 * 4. To send messages using general PC address (set in the constructor),
 * use pushBroadMessage(). These messages are not related to specific devices.
 * Every device on CAN by may read it.
 */

class CanDispatcher : public atlas::Runnable {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CanDispatcher>;
  using ConstPtr = std::shared_ptr<const CanDispatcher>;
  using PtrList = std::vector<CanDispatcher::Ptr>;
  using ConstPtrList = std::vector<CanDispatcher::ConstPtr>;

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
  // thread execution interval
  static const uint32_t THREAD_INTERVAL_US;
  // PC message buffer size
  static const uint32_t PC_BUFFER_SIZE;
  // Provider_can status message
  static const uint16_t PROVIDER_CAN_STATUS;
  // Messages types
  static const uint32_t UNICAST;

  // Address parameters positions
  static const uint8_t UNIQUE_ID_POSITION;
  static const uint8_t DEVICE_ID_POSITION;

  // Mac address mask
  static const uint32_t DEVICE_MAC_MASK;

  // Message type mask
  static const uint32_t DEVICE_MSG_MASK;

  //============================================================================
  // P U B L I C   C / D T O R S

  //! Constructor
  // param loop_rate main process loop rate. Must be the same as ROS loop_rate
  // param device_id PC ID
  // param unique_id PC ID
  explicit CanDispatcher(uint32_t device_id, uint32_t unique_id, uint32_t chan,
                         uint32_t baudrate,
                         const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  // Destructor
  ~CanDispatcher() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
  * The function sets the poll_rate value of the DeviceProperties struct of
  * the
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
                                uint16_t poll_rate) ATLAS_NOEXCEPT;

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
                                       uint8_t ndata) ATLAS_NOEXCEPT;

  /**
  * The function allows to send broadcast messages using PC address. The
  *messages
  * are not related to specific devices, but to anyone who reads it.
  *
  * \param message_id SONIA message ID
  * \param buffer message content
  * \param ndata message length
  */
  void PushBroadMessage(uint16_t message_id, uint8_t *buffer,
                        uint8_t ndata) ATLAS_NOEXCEPT;

  /**
  * The function returns the can_rx_buffer of the selected device
  *
  * The can_rx_buffer returned contains all received messages since last call
  * of
  * this function.
  * If the device asked does not exist, SoniaDeviceStatus will indicate it and
  * no can_rx_buffer will be returned.
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \param buffer device's can_rx_buffer
  * \return SoniaDeviceStatus enum
  */
  SoniaDeviceStatus FetchCanMessages(uint8_t device_id, uint8_t unique_id,
                                     std::vector<CanMessage> &buffer)
      ATLAS_NOEXCEPT;

  uint64_t GetNumberOfDevices() ATLAS_NOEXCEPT;

  void GetUnknownAddresses(std::vector<uint32_t> &addresses) ATLAS_NOEXCEPT;

  /**
  * This function is set as a service into ROS. Call it with the corresponding
  * method number of the device you want to set a parameter. Parameters can
  * be motor speed, light intensity, etc. Methods numbers are defined in
  * can_def for each existing device.
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \param method_number the specific number of the method to call
  * \param parameter_value parameter to pass to the method
  * \return SoniaDeviceStatus enum
  */

  bool CallDeviceMethod(sonia_msgs::SendCanMessage::Request &req,
                        sonia_msgs::SendCanMessage::Response &res)
      ATLAS_NOEXCEPT;

  /**
  * This function fetches messages received from ROS for a specific
  * device.
  *
  * \param device_id SONIA Device ID to look for
  * \param unique_id SONIA unique ID to look for
  * \param buffer messages received from computer
  * \return SoniaDeviceStatus enum
  */
  SoniaDeviceStatus FetchComputerMessages(uint8_t device_id, uint8_t unique_id,
                                          std::vector<ComputerMessage> &buffer)
      ATLAS_NOEXCEPT;

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
  SoniaDeviceStatus FindDevice(uint8_t device_id, uint8_t unique_id,
                               size_t *index) ATLAS_NOEXCEPT;
  SoniaDeviceStatus FindDevice(uint8_t device_id,
                               uint8_t unique_id) ATLAS_NOEXCEPT;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Call this process periodically in the main. Its function is to call
   * all devices processes for messages processing
   */
  void Run() ATLAS_NOEXCEPT override;

  /**
  * Allows the user to send an ID request to devices on SONIA's CAN bus.
  *
  */
  canStatus SendIdRequest() ATLAS_NOEXCEPT;

  /**
  * Sends RTR to devices at rate specified in DeviceProperties struct, for
  * every known device.
  */
  // void PollDevices();  // TODO: RTR functionnality should be implemented in
  // ELE
  // dans l'élé du sub

  /**
  * Sends an ID request on CAN bus and list all addresses received.
  *
  * This function creates a CanDevice struct for each device that answered to
  * the ID request.
  */
  canStatus ListDevices() ATLAS_NOEXCEPT;

  /**
  * Reads all messages received on CAN bus and stores them into rx_raw_buffer_
  */
  canStatus ReadMessages() ATLAS_NOEXCEPT;

  /**
  * sends all messages contained in tx_raw_buffer_
  */
  canStatus SendMessages() ATLAS_NOEXCEPT;

  /**
  * Dispatch all messages contained into rx_raw_buffer_ to each respective
  * CanDevice struct created by listDevices().
  * This function also filters ID_request responses and device_fault
  * messages to set DeviceProperties struct.
  */
  void DispatchMessages() ATLAS_NOEXCEPT;

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
  SoniaDeviceStatus FindDeviceWithAddress(uint32_t address,
                                          size_t *index) ATLAS_NOEXCEPT;
  SoniaDeviceStatus FindDeviceWithAddress(uint32_t address) ATLAS_NOEXCEPT;

  /**
  * The function sends and RTR on CAN bus with selected address
  *
  * \param address ID of the RTR
  */
  void SendRTR(uint32_t address) ATLAS_NOEXCEPT;

  /**
  * Adds an address to the unknown addresses table. If unknown
  * addresses are found, providerCanProcess will send ID requests to
  * be sure every present device are known.
  *
  * \param address
  */
  void AddUnknownAddress(uint32_t address) ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  std::vector<CanDeviceBuffers>
      devices_list_;  // List of devices present on CAN bus

  std::vector<uint32_t> unknown_addresses_table_;

  std::vector<CanMessage> rx_raw_buffer_;  // Buffer directly taken from KVaser
  std::vector<CanMessage> tx_raw_buffer_;

  std::mutex rx_raw_buffer_mutex_, tx_raw_buffer_mutex_;

  UsbCanII::Ptr can_driver_;  // Can communication object

  timespec actual_time_;
  timespec initial_time_;
  timespec id_req_time_;

  uint8_t discovery_tries_;
  std::mutex can_rx_buffer_mutex, pc_messages_buffer_mutex;

  uint32_t tx_error_;
  uint32_t rx_error_;
  uint32_t ovrr_error_;

  uint32_t master_id_;  // PC ID

  ros::NodeHandlePtr nh_;
  ros::ServiceServer call_device_srv_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DISPATCHER_H_
