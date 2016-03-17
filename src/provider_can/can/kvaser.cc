/**
 * \file	kvaser.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	15/03/2015
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "kvaser.h"
#include <ros/ros.h>

namespace provider_can {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
KVaser::KVaser(uint32_t chan, int32_t baudrate)
    : CanDriver(),
      baudrate_(baudrate),
      channel_(chan),
      tseg1_(0),
      tseg2_(0),
      sjw_(1),
      noSamp_(16) {
  if (!InitUsbDevice()) {
    throw atlas::IOException("KVaser device initialization");
  }
}

//------------------------------------------------------------------------------
//
KVaser::KVaser(uint32_t chan, int32_t baudrate, uint32_t ts1, uint32_t ts2,
                   uint32_t jump, uint32_t samp)
    : CanDriver(),
      baudrate_(baudrate),
      channel_(chan),
      tseg1_(ts1),
      tseg2_(ts2),
      sjw_(jump),
      noSamp_(samp) {
  if (!InitUsbDevice()) {
    throw atlas::IOException("KVaser device initialization");
  }
}

KVaser::~KVaser() ATLAS_NOEXCEPT {
  FlushRxBuffer();
  FlushTxBuffer();
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
canStatus KVaser::Open() ATLAS_NOEXCEPT {

  handle_ = canOpenChannel(channel_, canWANT_EXCLUSIVE | canWANT_EXTENDED);
  // CAN channel handler
  unsigned short version = 0;
  version = canGetVersion();

  ROS_INFO("Canlib version = %d.%d", (version >> 8), (version & 0xFF));

  return (canStatus)handle_;
}

//------------------------------------------------------------------------------
//
bool KVaser::InitUsbDevice() ATLAS_NOEXCEPT {
  canStatus status;

  status = Open();  // Open CAN channel

  if (status < canOK) {  // If open failed
    PrintErrorText(status);
    return false;
  }

  status = SetBusParams();  // Set CAN parameters

  if (status < canOK) {  // If parameters set failed
    PrintErrorText(status);
    return false;
  }

  status = SetBusOn();  // Turn on the channel

  if (status < canOK) {  // If bus can't turn on
    PrintErrorText(status);
    return false;
  }
  ROS_INFO("CAN Bus is set to on");
  return true;  // Init success
}

//------------------------------------------------------------------------------
//
canStatus KVaser::SetBusParams() const ATLAS_NOEXCEPT {
  canStatus status;

  switch (baudrate_) {
    case 1000:
      status = canSetBusParams(handle_, BAUD_1M, 0, 0, 0, 0, 0);
      break;
    case 500:
      status = canSetBusParams(handle_, BAUD_500K, 0, 0, 0, 0, 0);
      break;
    case 250:
      status = canSetBusParams(handle_, BAUD_250K, 0, 0, 0, 0, 0);
      break;
    case 125:
      status = canSetBusParams(handle_, BAUD_125K, 0, 0, 0, 0, 0);
      break;
    case 100:
      status = canSetBusParams(handle_, BAUD_100K, 0, 0, 0, 0, 0);
      break;
    case 62:
      status = canSetBusParams(handle_, BAUD_62K, 0, 0, 0, 0, 0);
      break;
    case 50:
      status = canSetBusParams(handle_, BAUD_50K, 0, 0, 0, 0, 0);
      break;
    default:
      status =
          canSetBusParams(handle_, baudrate_, tseg1_, tseg2_, sjw_, noSamp_, 0);
      break;
  }

  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::SetBusOff() const ATLAS_NOEXCEPT {
  canStatus status;

  status = canBusOff(handle_);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::SetBusOn() const ATLAS_NOEXCEPT {
  canStatus status;

  status = canBusOn(handle_);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::Close() const ATLAS_NOEXCEPT {
  canStatus status;

  status = canClose(handle_);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::WriteMessage(CanMessage msg,
                                 uint32_t timeout_msec) const ATLAS_NOEXCEPT {
  canStatus status;
  if (timeout_msec == 0) {
    status = canWrite(handle_, msg.id, msg.data, msg.dlc, msg.flag);
  } else {
    status = canWriteWait(handle_, msg.id, msg.data, msg.dlc, msg.flag,
                          timeout_msec);
  }
  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::WriteBuffer(std::vector<CanMessage> &msg_table,
                                uint32_t timeout_msec) const ATLAS_NOEXCEPT {
  canStatus status = canOK;
  for (std::vector<CanMessage>::size_type i = 0; i < msg_table.size(); i++) {
    status = WriteMessage(msg_table[i], timeout_msec);
    if (status != canOK) {
      return status;
    }
  }

  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::ReadMessage(CanMessage *msg,
                                uint32_t timeout_msec) const ATLAS_NOEXCEPT {
  canStatus status;
  long int id;
  long unsigned int time;

  if (timeout_msec == 0) {
    status = canRead(handle_, &id, &msg->data, &msg->dlc, &msg->flag, &time);
  } else {
    status = canReadWait(handle_, &id, &msg->data, &msg->dlc, &msg->flag, &time,
                         timeout_msec);
  }

  msg->id = id;
  msg->time = time;
  return status;
}

//------------------------------------------------------------------------------
//

canStatus KVaser::ReadAllMessages(std::vector<CanMessage> &msg_table) const
    ATLAS_NOEXCEPT {
  canStatus status;
  CanMessage msg;
  uint32_t num_of_messages = 0;

  status = GetRxBufLevel(&num_of_messages);  // get the reception buffer level

  if (status == canOK) {
    for (uint32_t i = 0; i < num_of_messages; i++) {  // storing data
      status = ReadMessage(&msg, 0);

      if (status != canOK) {
        return status;
      }
      msg_table.push_back(msg);
    }
  }

  return status;
}

//------------------------------------------------------------------------------
//
void KVaser::PrintErrorText(canStatus error) const ATLAS_NOEXCEPT {
  char errMsg[128];
  errMsg[0] = '\0';

  canGetErrorText(error, errMsg, sizeof(errMsg));

  if (error != canOK) {
    ROS_WARN("KVaser Error: %s", errMsg);
  } else {
    ROS_INFO("KVaser Error: %s", errMsg);
  }
}

//------------------------------------------------------------------------------
//
canStatus KVaser::SetAcceptanceFilter(uint32_t enveloppe,
                                        int flag) const ATLAS_NOEXCEPT {
  canStatus status;

  status = canAccept(handle_, enveloppe, flag);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::GetErrorCount(uint32_t *tx_err, uint32_t *rx_err,
                                  uint32_t *ov_err) const ATLAS_NOEXCEPT {
  canStatus status;

  status = canReadErrorCounters(handle_, tx_err, rx_err, ov_err);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::GetRxBufLevel(uint32_t *lvl) const ATLAS_NOEXCEPT {
  canStatus status;

  status = canIoCtl(handle_, canIOCTL_GET_RX_BUFFER_LEVEL, lvl, sizeof(lvl));

  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::GetTxBufLevel(uint32_t *lvl) const ATLAS_NOEXCEPT {
  canStatus status;

  status = canIoCtl(handle_, canIOCTL_GET_TX_BUFFER_LEVEL, lvl, sizeof(lvl));

  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::FlushRxBuffer() const ATLAS_NOEXCEPT {
  canStatus status;

  status = canIoCtl(handle_, canIOCTL_FLUSH_RX_BUFFER, 0, 0);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::FlushTxBuffer() const ATLAS_NOEXCEPT {
  canStatus status;

  status = canIoCtl(handle_, canIOCTL_FLUSH_TX_BUFFER, 0, 0);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus KVaser::GetBusParams(long *freq, unsigned int *tseg1,
                                 unsigned int *tseg2, unsigned int *sjw,
                                 unsigned int *no_samp) const ATLAS_NOEXCEPT {
  canStatus status;
  uint32_t dummy;

  status = canGetBusParams(handle_, freq, tseg1, tseg2, sjw, no_samp, &dummy);

  return status;
}
}
