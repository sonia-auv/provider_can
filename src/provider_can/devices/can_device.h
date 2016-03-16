/**
 * \file	can_device.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	04/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_CAN_DEVICE_H
#define PROVIDER_CAN_CAN_DEVICE_H

#include <lib_atlas/macros.h>
#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include "provider_can/can/can_dispatcher.h"
#include "provider_can/can_def.h"

namespace provider_can {

/**
 * This class is a frame for all CAN devices. It groups all common functions
 * for all devices. Specific devices' functions must be implemented outside
 * of this class.
 *
 * This class contains one pure virtual function: ProcessMessages(). Each device
 * must at least implement his own process for handling received messages.
 * The parameters of this function will contain the messages relative to the
 * specific device.
 *
 * ROS messages published are notices and properties. See .msg files for infos.
 */

class CanDevice {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CanDevice>;
  using ConstPtr = std::shared_ptr<const CanDevice>;
  using PtrList = std::vector<CanDevice::Ptr>;
  using ConstPtrList = std::vector<CanDevice::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit CanDevice(const DeviceClass &device_id, uint8_t unique_id,
                     const CanDispatcher::Ptr &can_dispatcher,
                     const std::string &name,
                     const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  virtual ~CanDevice() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   *  Process that handles devices common messages and that calls
   * ProcessMessages()
   *  for the device inheriting this class.ProcessMessages() will be able to
   *  process device's specific messages, such as speed for motors.
   *
   *  Call this process periodically to handle device operation.
   */
  void Process() ATLAS_NOEXCEPT;

  /**
   * returns a std string containing the name of the actual device.
   *
   * \returns std::string name of the device
   */
  const std::string &GetName() const ATLAS_NOEXCEPT;

 protected:
  //============================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * Pushes a message to the device
   *
   * \param buffer message content
   * \param ndata message length
   */
  void PushMessage(uint16_t message_id, uint8_t *buffer,
                   uint8_t ndata) const ATLAS_NOEXCEPT;

  /**
   * Method called by Process(). Use this function to process device's specific
   * messages.
   *
   * \param can_rx_buffer can messages received from bus for the actual device
   * \param pc_messages_buffer pc messages received from ROS service
   */
  virtual void ProcessMessages(
      const std::vector<CanMessage> &from_can_rx_buffer,
      const std::vector<ComputerMessage> &from_pc_rx_buffer) = 0;

  //============================================================================
  // P R O T E C T E D   M E M B E R S

  std::shared_ptr<CanDispatcher> can_dispatcher_;  // pointer to can controller

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Processes common pc messages contained in rx buffer. Common messages may be
   * ping req, get param req, etc.
   */
  void ProcessCommonPcMessages(void) ATLAS_NOEXCEPT;

  /**
   * Verifies if the device is present on the can bus.
   *
   * \returns true or false if device is present or not
   */
  bool DevicePresenceCheck() const ATLAS_NOEXCEPT;

  /**
   * Returns device's properties as a ROS msg
   */
  void SendProperties() const ATLAS_NOEXCEPT;

  /**
   * Sends a reset message to the device
   * TODO Alexi Demers: This will have to be implemented in ELE part
   */
  // void Reset();

  /**
   * Sends a sleep request to the device
   * TODO Alexi Demers: This will have to be implemented in ELE part
   */
  // void SleepMode();

  /**
   * Sends a wake up request to the device
   * TODO Alexi Demers: This will have to be implemented in ELE part
   */
  // void WakeUp();

  // TODO: to be added: SetPollRate when implemented in ELE

  //============================================================================
  // P R I V A T E   M E M B E R S

  uint8_t *fault_message_;  // Fault message if device_fault is true
  std::string name_;        // device's name

  DeviceClass device_id_;
  uint8_t unique_id_;

  // Publishers for properties and notices
  ros::Publisher properties_pub_;
  ros::Publisher device_notices_pub_;

  // Messages tables taken from can_disp that will be transmitted to devices
  std::vector<CanMessage> from_can_rx_buffer_;
  std::vector<ComputerMessage> from_pc_rx_buffer_;

  bool properties_sent_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
ATLAS_INLINE const std::string &CanDevice::GetName() const ATLAS_NOEXCEPT {
  return name_;
}
/*
//------------------------------------------------------------------------------
//ATLAS_INLINE void CanDevice::Reset() {
can_dispatcher_->SendResetRequest(device_id_, unique_id_);
}
*/

//------------------------------------------------------------------------------
//
ATLAS_INLINE bool CanDevice::DevicePresenceCheck() const ATLAS_NOEXCEPT {
  if (can_dispatcher_->FindDevice(device_id_, unique_id_) !=
      SONIA_DEVICE_NOT_PRESENT)
    return true;
  else
    return false;
}

/*
//------------------------------------------------------------------------------
//
ATLAS_INLINE void CanDevice::SleepMode() const ATLAS_NOEXCEPT {
  can_dispatcher_->SendSleepRequest(device_id_,unique_id_);
}
*/

/*
//------------------------------------------------------------------------------
//
ATLAS_INLINE void CanDevice::WakeUp() const ATLAS_NOEXCEPT {
  can_dispatcher_->SendWakeUpRequest(device_id_,unique_id_);
}
*/

//------------------------------------------------------------------------------
//
ATLAS_INLINE void CanDevice::PushMessage(uint16_t message_id, uint8_t *buffer,
                                         uint8_t ndata) const ATLAS_NOEXCEPT {
  can_dispatcher_->PushUnicastMessage(device_id_, unique_id_, message_id,
                                      buffer, ndata);
}

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DEVICE_CC_H
