/**
 * \file	can_driver.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	15/03/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_CAN_DRIVER_H_
#define PROVIDER_CAN_CAN_DRIVER_H_

#include <vector>
#include <memory>
#include <canlib.h>
#include <lib_atlas/macros.h>

#include <canlib.h>
#include <lib_atlas/exceptions/io_exception.h>
#include <lib_atlas/macros.h>
#include <stdint.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

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

/**
 * This class acts as an interface for all CAN devices drivers.
 */
class CanDriver {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CanDriver>;
  using ConstPtr = std::shared_ptr<const CanDriver>;
  using PtrList = std::vector<CanDriver::Ptr>;
  using ConstPtrList = std::vector<CanDriver::ConstPtr>;


  //============================================================================
  // P U B L I C   M E T H O D S

  /**
  * Allows the user to read one CAN message through a CAN device
  *
  * \param msg Struct where received message will be stored
  * \param timeout_msec time to wait for a message to be received
  * \return canStatus enum
  */
  virtual canStatus ReadMessage(CanMessage* msg,
                                uint32_t timeout_msec) const ATLAS_NOEXCEPT = 0;

  /**
   * Allows the user to read all CAN messages received
   *
   * \param msg_table pointer to the messages table.
   * \param num_of_messages number of messages read (which is the size of the
   *                        returned table)
   * \return canStatus
   */
  virtual canStatus ReadAllMessages(std::vector<CanMessage>& msg_table) const
      ATLAS_NOEXCEPT = 0;

  /**
  * Allows the user to send one CAN message through a CAN device
  *
  * \param msg struct containing message informations
  * \param timeout_msec time to wait for a message to be sent
  * \return canStatus enum
  */
  virtual canStatus WriteMessage(CanMessage msg, uint32_t timeout_msec) const
      ATLAS_NOEXCEPT = 0;
  virtual canStatus WriteBuffer(std::vector<CanMessage>& msg_table,
                                uint32_t timeout_msec) const ATLAS_NOEXCEPT = 0;

  /**
  * Convert canStatus enum into text to show on terminal
  */
  virtual void PrintErrorText(canStatus error) const ATLAS_NOEXCEPT = 0;

  /**
   * Deletes all messages in specified buffer
   */
  virtual canStatus FlushTxBuffer() const ATLAS_NOEXCEPT = 0;
  virtual canStatus FlushRxBuffer() const ATLAS_NOEXCEPT = 0;

  /**
   * Returns number of errors encountered in different processes
   *
   * \param txErr number of transmission errors
   * \param rxErr number of reception errors
   * \param ovErr number of overrun errors
   * \return canStatus enum (canOK or canERR)
   */
  virtual canStatus GetErrorCount(uint32_t* tx_err, uint32_t* rx_err,
                                  uint32_t* ov_err) const ATLAS_NOEXCEPT = 0;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DRIVER_H_
