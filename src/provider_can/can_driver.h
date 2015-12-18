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
#include <iostream>
#include <iomanip>
#include <stdint.h>
#include <memory>
#include "provider_can/can_exceptions.h"

namespace provider_can {

//============================================================================
// T Y P E D E F   A N D   E N U M

typedef struct {
  /// device identifier. see SONIA's documentation
  uint32_t id;
  /// data to send
  unsigned char data[8];
  /// number of data bytes
  uint32_t dlc;
  /// Type of the message. In transmission, possible values are: canMSG_EXT,
  /// canMSG_RTR, canMSG_STD. In reception, there are many more possibilities.
  //  see canlib documentation.
  uint32_t flag;
  /// Timestamp of the message
  uint32_t time;
} CanMessage;

typedef enum { SONIA_CAN_OK = 0, SONIA_CAN_ERR = -1 } SoniaCanStatus;

class CanDriver {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CanDriver>;

  //============================================================================
  // P U B L I C   C / D T O R S

  //! Constructor

  /**
   * Initialize KVaser device
   *
   * \param chan KVaser channel 0 or 1
   * \param baudrate possible values
   *          BAUD_1M:
              BAUD_500K:
              BAUD_250K:
              BAUD_125K:
              BAUD_100K:
              BAUD_62K:
              BAUD_50K:
   * \param ts1 Time segment 1
   * \param ts2 Time segment 2
   * \param jump The Synchronization Jump Width; can be 1,2,3, or 4.
   * \param samp The number of sampling points; can be 1 or 3.
   */
  CanDriver(uint32_t chan, uint32_t baudrate);
  CanDriver(uint32_t chan, uint32_t baudrate, uint32_t ts1, uint32_t ts2,
            uint32_t jump, uint32_t samp);

  // Destructor
  ~CanDriver();

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
  * Allows the user to read one CAN message through a KVaser device
  *
  * This stores read messages from the read buffer into msg struct.
  * The function allows to select a timeout delay after which the function will
  * return if no message was received.
  *
  * \param msg Struct where received message will be stored
  * \param timeout_msec time to wait for a message to be received
  * \return canStatus enum
  */
  canStatus ReadMessage(CanMessage* msg, uint32_t timeout_msec);

  /**
   * Allows the user to read all CAN messages received through a KVaser device
   *
   * This function verifies how many messages are actually contained in
   * the reception buffer, reads them all and returns them into a table.
   *
   * \param msg_table pointer to the messages table.
   * \param num_of_messages number of messages read (which is the size of the
   *                        returned table)
   * \return canStatus
   */
  canStatus ReadAllMessages(CanMessage*& msg_table, uint32_t* num_of_messages);

  /**
  * Allows the user to send one CAN message through a KVaser device
  *
  * This function takes the message to send from a msg struct and
  * allows to select a timeout delay after which the message will
  * be dropped if the device did not success in sending it.
  *
  * \param msg struct containing message informations
  * \param timeout_msec time to wait for a message to be sent
  * \return canStatus enum
  */
  canStatus WriteMessage(CanMessage msg, uint32_t timeout_msec);

  /**
  * Convert canStatus enum into text to show on terminal
  */
  void PrintErrorText(canStatus error);

  canStatus FlushTxBuffer();
  canStatus FlushRxBuffer();

  /**
* Returns number of errors encountered in different processes
*
* \param txErr number of transmission errors
* \param rxErr number of reception errors
* \param ovErr number of overrun errors
* \return canStatus enum (canOK or canERR)
*/
  canStatus GetErrorCount(uint32_t* tx_err, uint32_t* rx_err, uint32_t* ov_err);

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  bool InitUsbDevice();
  canStatus Open();
  canStatus SetBusParams();
  canStatus SetBusOff();
  canStatus SetBusOn();
  canStatus Close();
  canStatus GetBusParams(long* freq, unsigned int* tseg1, unsigned int* tseg2,
                         unsigned int* sjw, unsigned int* noSamp);

  /**
   * gets the number of messages contained in the selected buffer
   *
   * \param lvl number of messages
   * \return canStatus enum (canOK or canERR)
   */
  canStatus GetTxBufLevel(uint32_t* lvl);
  canStatus GetRxBufLevel(uint32_t* lvl);

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
  canStatus SetAcceptanceFilter(uint32_t enveloppe, int flag);

  //============================================================================
  // P R I V A T E   M E M B E R S

  uint32_t channel_;  // CAN channel used (0 or 1)

  canHandle handle_;

  uint32_t baudrate_;

  CanMessage* rx_buffer_;

  uint32_t tseg1_;   // Time segment 1
  uint32_t tseg2_;   // Time segment 2
  uint32_t sjw_;     // The Synchronization Jump Width; can be 1,2,3, or 4.
  uint32_t noSamp_;  // The number of sampling points; can be 1 or 3.
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DRIVER_H_
