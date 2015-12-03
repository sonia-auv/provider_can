/**
 * \file	can_driver.h
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

#ifndef PROVIDER_CAN_CAN_DRIVER_H_
#define PROVIDER_CAN_CAN_DRIVER_H_

#include <canlib.h>
#include "exception.h"
#include <iostream>
#include <iomanip>

namespace provider_can {

//============================================================================
// T Y P E D E F   A N D   E N U M

typedef struct {
  /// Short desc.
  long id;
  /// Short desc.
  uint32_t data[8];
  /// Short desc.
  uint32_t dlc;
  /// Short desc.
  uint32_t flag;
  /// Short desc.
  unsigned long time;
} CanMessage;

typedef enum { SONIA_CAN_OK = 0, SONIA_CAN_ERR = -1 } SoniaCanStatus;

const uint32_t SONIA_CAN_BAUD_1M = 1000000;
const uint32_t SONIA_CAN_BAUD_500K = 500000;
const uint32_t SONIA_CAN_BAUD_250K = 250000;
const uint32_t SONIA_CAN_BAUD_125K = 125000;
const uint32_t SONIA_CAN_BAUD_100K = 100000;
const uint32_t SONIA_CAN_BAUD_62K = 62000;
const uint32_t SONIA_CAN_BAUD_50K = 50000;

class CanDriver {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  //! Constructor
  CanDriver(uint32_t chan, uint32_t baudrate);

  // Destructor
  ~CanDriver();

  //============================================================================
  // P U B L I C   M E T H O D S

   /**
   * Allows the user to read a CAN message through a KVaser device
   *
   * This stores read messages from the read buffer into msg struct.
   * The function allows to select a timeout delay after which the function will
   * return if no message was received.
   *
   * \param msg Struct where received message will be stored
   * \param timeout_msec time to wait for a message to be received
   * \return canStatus enum
   */
  canStatus readMessage(CanMessage *msg, uint32_t timeout_msec) ;

   /**
   * Allows the user to read a CAN message through a KVaser device
   *
   * This function takes the message to send from a msg struct and
   * allows to select a timeout delay after which the message will
   * be dropped if the device did not success in sending it.
   *
   * \param msg struct containing message informations
   * \param timeout_msec time to wait for a message to be sent
   * \return canStatus enum
   */
  canStatus writeMessage(CanMessage *msg, uint32_t timeout_msec) ;

   /**
   * Convert canStatus enum into text to show on terminal
   */
  void printErrorText(canStatus error);

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  bool initUsbDevice();

  canStatus open();

  canStatus setBusParams();

  canStatus setBusOff();

  canStatus setBusOn();

  canStatus close();


   /**
   * Adds an acceptance filter to CAN device so that we can receive
   * requested messages IDs.
   *
   * This function can set a mask to apply to the acceptance filters so
   * that all IDs fitting in it will be received, or set a specific ID to
   * receive.
   *
   * \param enveloppe mask/code to apply
   * \param flag values may be: canFILTER_SET_CODE_STD, canFILTER_SET_MASK_STD,
   *                            canFILTER_SET_CODE_EXT, canFILTER_SET_MASK_EXT
   * \return canStatus enum
   */
  canStatus setAcceptanceFilter(uint32_t enveloppe, int flag);

   /**
   * Returns number of errors encountered in different processes
   *
   * \param txErr number of transmission errors
   * \param rxErr number of reception errors
   * \param ovErr number of overrun errors
   * \return canStatus enum (canOK or canERR)
   */
  canStatus getErrorCount(uint32_t* txErr, uint32_t* rxErr, uint32_t* ovErr);

  //============================================================================
  // P R I V A T E   M E M B E R S

  uint32_t channel_;    // CAN channel used (0 or 1)

  canHandle handle_;

  uint32_t baudrate_;

  uint32_t msg_count_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DRIVER_H_
