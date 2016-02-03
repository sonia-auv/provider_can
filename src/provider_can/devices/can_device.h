/**
 * \file	can_device.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	25/11/2015
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

#ifndef PROVIDER_CAN_CAN_DEVICE_H
#define PROVIDER_CAN_CAN_DEVICE_H

#include <memory>
#include <vector>
#include <cstring>
#include "provider_can/can/can_dispatcher.h"
#include "provider_can/can/can_def.h"

namespace provider_can {

/**
 * This class is a frame for all CAN devices. It groups all common functions
 * for all devices. Specific devices' functions must be implemented outside
 * of this class.
 *
 * This class contains one pure virtual function: Process(). Each device must
 * at least implement his own process.
 */

class CanDevice {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CanDevice>;

  //============================================================================
  // P U B L I C   C / D T O R S

  CanDevice(DeviceClass device_class, uint8_t device_type,
            std::shared_ptr<CanDispatcher> can_dispatcher, std::string name);

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * Process common to all devices. This has to be called periodically for every
   * device.
   */
  virtual void Process() = 0;

  /**
   * returns a std string containing the name of the actual device.
   *
   * \returns std::string name of the device
   */
  std::string GetName();

  /**
   * returns a structure containing device properties
   *
   * \return DeviceProperties
   */
  DeviceProperties GetProperties();

  /**
   * returns a pointer to the device fault, if a fault has been received. If
   * not, the pointer will be NULL;
   *
   * \return pointer to the fault, 8 characters long
   */
  uint8_t *GetFault();

  /**
   * Returns if a ping response has been received or not since last call of
   * this function.
   *
   * \returns true or false if response has been received or not
   */
  bool GetPingStatus();

  /**
   * Sends a ping message to the device
   */
  void Ping();

  /**
   * Verifies if the device is present on the can bus.
   *
   * \returns true or false if device is present or not
   */
  bool DevicePresenceCheck();

  /**
   * Sends a reset message to the device
   */
  // void Reset(); // TODO: devra etre rendue fonctionnelle dans l'ELE

  /**
   * Sends a sleep request to the device
   */
  // void SleepMode(); // TODO: devra etre rendue fonctionnelle dans l'ELE

  /**
   * Sends a wake up request to the device
   */
  // void WakeUp(); // TODO: devra etre rendue fonctionnelle dans l'ELE

  // TODO: to be added: GetDeviceParams, SetDeviceParams et SetPollRate
  // lorsqu'implementees dans
  // l'ELE.

 protected:
  //============================================================================
  // P R O T E C T E D   M E M B E R S

  std::shared_ptr<CanDispatcher> can_dispatcher_;  // pointer to can controller

  /**
   * Collects messages received for that device
   *
   * \param buffer device's rx_buffer
   */
  std::vector<CanMessage> FetchMessages();

  /**
   * Collects messages received from computer for that device
   *
   * \param buffer device's rx_buffer
   */
  std::vector<ComputerMessage> FetchComputerMessages();

  /**
   * Pushes a message to the device
   *
   * \param buffer message content
   * \param ndata message length
   */
  void PushMessage(uint16_t message_id, uint8_t *buffer, uint8_t ndata);

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  uint8_t *fault_message;  // Fault message if device_fault is true
  std::string name_;       // device's name

  DeviceClass device_id_;
  uint8_t unique_id_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DEVICE_CC_H
