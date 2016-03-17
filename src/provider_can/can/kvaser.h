/**
 * \file	kvaser.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	15/03/2015
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_USB_CANII_H_
#define PROVIDER_CAN_USB_CANII_H_

#include <vector>
#include <iostream>
#include <iomanip>
#include <stdint.h>
#include <memory>
#include <canlib.h>
#include "can_driver.h"
#include <lib_atlas/macros.h>
#include <lib_atlas/exceptions/io_exception.h>

namespace provider_can {

/**
 * This class is the driver for the KVaser KVaser device.
 */
class KVaser : public CanDriver {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<KVaser>;
  using ConstPtr = std::shared_ptr<const KVaser>;
  using PtrList = std::vector<KVaser::Ptr>;
  using ConstPtrList = std::vector<KVaser::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S

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

  explicit KVaser(uint32_t chan, int32_t baudrate);

  explicit KVaser(uint32_t chan, int32_t baudrate, uint32_t ts1,
                    uint32_t ts2, uint32_t jump, uint32_t samp);

  // Destructor
  ~KVaser() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
  * Allows the user to read one CAN message through a KVaser device
  *
  * \param msg Struct where received message will be stored
  * \param timeout_msec time to wait for a message to be received
  * \return canStatus enum
  */
  canStatus ReadMessage(CanMessage* msg,
                        uint32_t timeout_msec) const ATLAS_NOEXCEPT override;

  /**
   * Allows the user to read all CAN messages received through a KVaser device
   *
   * \param msg_table pointer to the messages table.
   * \param num_of_messages number of messages read (which is the size of the
   *                        returned table)
   * \return canStatus
   */
  canStatus ReadAllMessages(std::vector<CanMessage>& msg_table) const
      ATLAS_NOEXCEPT override;

  /**
  * Allows the user to send CAN messages through a KVaser device
  *
  * \param msg struct containing message informations
  * \param timeout_msec time to wait for a message to be sent
  * \return canStatus enum
  */
  canStatus WriteMessage(CanMessage msg,
                         uint32_t timeout_msec) const ATLAS_NOEXCEPT override;
  canStatus WriteBuffer(std::vector<CanMessage>& msg_table,
                        uint32_t timeout_msec) const ATLAS_NOEXCEPT override;

  /**
  * Convert canStatus enum into text to show on terminal
  */
  void PrintErrorText(canStatus error) const ATLAS_NOEXCEPT override;

  /**
   * Deletes all messages in specified buffer
   */
  canStatus FlushTxBuffer() const ATLAS_NOEXCEPT override;
  canStatus FlushRxBuffer() const ATLAS_NOEXCEPT override;

  /**
* Returns number of errors encountered in different processes
*
* \param txErr number of transmission errors
* \param rxErr number of reception errors
* \param ovErr number of overrun errors
* \return canStatus enum (canOK or canERR)
*/
  canStatus GetErrorCount(uint32_t* tx_err, uint32_t* rx_err,
                          uint32_t* ov_err) const ATLAS_NOEXCEPT override;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Initialization methods
   */
  bool InitUsbDevice() ATLAS_NOEXCEPT;
  canStatus Open() ATLAS_NOEXCEPT;
  canStatus SetBusParams() const ATLAS_NOEXCEPT;
  canStatus SetBusOff() const ATLAS_NOEXCEPT;
  canStatus SetBusOn() const ATLAS_NOEXCEPT;
  canStatus Close() const ATLAS_NOEXCEPT;
  canStatus GetBusParams(long* freq, unsigned int* tseg1, unsigned int* tseg2,
                         unsigned int* sjw,
                         unsigned int* noSamp) const ATLAS_NOEXCEPT;

  /**
   * gets the number of messages contained in the selected buffer
   *
   * \param lvl number of messages
   * \return canStatus enum (canOK or canERR)
   */
  canStatus GetTxBufLevel(uint32_t* lvl) const ATLAS_NOEXCEPT;
  canStatus GetRxBufLevel(uint32_t* lvl) const ATLAS_NOEXCEPT;

  /**
  * Adds an acceptance filter to CAN device so that we can receive
  * requested messages IDs.
  *
  * \param enveloppe mask/code to apply
  * \param flag values may be: canFILTER_SET_CODE_STD, canFILTER_SET_MASK_STD,
  *                            canFILTER_SET_CODE_EXT, canFILTER_SET_MASK_EXT
  * \return canStatus enum
  */
  canStatus SetAcceptanceFilter(uint32_t enveloppe,
                                int flag) const ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  int32_t baudrate_;
  uint32_t channel_;

  uint32_t tseg1_;  // Time segment 1
  uint32_t tseg2_;  // Time segment 2

  uint32_t sjw_;     // The Synchronization Jump Width; can be 1,2,3, or 4.
  uint32_t noSamp_;  // The number of sampling points; can be 1 or 3.

  canHandle handle_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_USB_CANII_H_
