/**
* \file	sonar_driver.cc
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

#include "can_driver.h"

namespace provider_can {

//==============================================================================
// C / D T O R   S E C T I O N

  CanDriver::CanDriver(uint32_t chan, uint32_t baudrate)
    : baudrate_(baudrate),
      channel_(chan),
      tseg1_(0),
      tseg2_(0),
      sjw_(1),
      noSamp_(16) {
    if (!InitUsbDevice()) {
      throw ExceptionCanDeviceNotFound();
    }
  }

  CanDriver::CanDriver(uint32_t chan, uint32_t baudrate, uint32_t ts1,
                       uint32_t ts2, uint32_t jump, uint32_t samp)
    : channel_(chan),
      baudrate_(baudrate),
      tseg1_(ts1),
      tseg2_(ts2),
      sjw_(jump),
      noSamp_(samp) {
    if (!InitUsbDevice()) {
      throw ExceptionCanDeviceNotFound();
    }
  }

//------------------------------------------------------------------------------
//
  CanDriver::~CanDriver() {
    FlushRxBuffer();
    FlushTxBuffer();
  }

//==============================================================================
// M E T H O D S   S E C T I O N
  canStatus CanDriver::Open(void) {
    handle_ = canOpenChannel(channel_, canWANT_EXCLUSIVE | canWANT_EXTENDED);
    // CAN channel handler

    unsigned short version = 0;
    version = canGetVersion();

    std::cout << "Canlib version = " << (version >> 8) << "." << (version & 0xFF)
    << std::endl;

    return (canStatus) handle_;
  }

//------------------------------------------------------------------------------
//
  bool CanDriver::InitUsbDevice(void) {
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

    return true;  // Init success
  }

//------------------------------------------------------------------------------
//
  canStatus CanDriver::SetBusParams() {
    canStatus status;

    switch (baudrate_) {
      case BAUD_1M:
      case BAUD_500K:
      case BAUD_250K:
      case BAUD_125K:
      case BAUD_100K:
      case BAUD_62K:
      case BAUD_50K:
        status = canSetBusParams(handle_, baudrate_, 0, 0, 0, 0, 0);
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
  canStatus CanDriver::SetBusOff() {
    canStatus status;

    status = canBusOff(handle_);

    return status;
  }

//------------------------------------------------------------------------------
//
  canStatus CanDriver::SetBusOn() {
    canStatus status;

    status = canBusOn(handle_);

    return status;
  }

//------------------------------------------------------------------------------
//
  canStatus CanDriver::Close() {
    canStatus status;

    status = canClose(handle_);

    return status;
  }

//------------------------------------------------------------------------------
//
  canStatus CanDriver::WriteMessage(CanMessage msg, uint32_t timeout_msec) {
    canStatus status;
    if (timeout_msec == 0)
      status = canWrite(handle_, msg.id, msg.data, msg.dlc, msg.flag);
    else
      status = canWriteWait(handle_, msg.id, msg.data, msg.dlc, msg.flag,
                            timeout_msec);
    return status;
  }

//------------------------------------------------------------------------------
//
canStatus CanDriver::WriteBuffer(std::vector<CanMessage> &msg_table,
									uint32_t timeout_msec) {
  canStatus status;
  for(std::vector<CanMessage>::size_type i = 0; i < msg_table.size(); i++){
	  status = WriteMessage(msg_table[i],timeout_msec);
	  if (status != canOK)
		  i = msg_table.size();

  }


  return status;
}

//------------------------------------------------------------------------------
//
  canStatus CanDriver::ReadMessage(CanMessage *msg, uint32_t timeout_msec) {
    canStatus status;
    long int id;
    long unsigned int time;

    if (timeout_msec == 0)
      status = canRead(handle_, &id, &msg->data, &msg->dlc, &msg->flag, &time);
    else
      status = canReadWait(handle_, &id, &msg->data, &msg->dlc, &msg->flag, &time,
                           timeout_msec);

    msg->id = id;
    msg->time = time;
    return status;
  }

//------------------------------------------------------------------------------
//

  // TODO: Use shared pointer instead of raw pointer
  canStatus CanDriver::ReadAllMessages(std::vector<CanMessage> &msg_table) {
    long int id;
    long unsigned int time;
    canStatus status;
    CanMessage msg;
    uint32_t num_of_messages = 0;

    status = GetRxBufLevel(&num_of_messages);  // get the reception buffer level

    if (status == canOK) {

      for (uint32_t i = 0; i < num_of_messages; i++) {  // storing data
        status = ReadMessage(&msg, 0);
        msg_table.push_back(msg);

        if (status != canOK) i = num_of_messages;
      }
    }

    return status;
  }

//------------------------------------------------------------------------------
//
  void CanDriver::PrintErrorText(canStatus error) {
    char errMsg[128];
    errMsg[0] = '\0';

    canGetErrorText(error, errMsg, sizeof(errMsg));

    if (error != canOK)
      std::cout << " ERROR "
      << "(" << errMsg << ")" << std::endl;
    else
      std::cout << "(" << errMsg << ")";
  }

//------------------------------------------------------------------------------
//
  canStatus CanDriver::SetAcceptanceFilter(uint32_t enveloppe, int flag) {
    canStatus status;

    status = canAccept(handle_, enveloppe, flag);

    return status;
  }

//------------------------------------------------------------------------------
//
  canStatus CanDriver::GetErrorCount(uint32_t *tx_err, uint32_t *rx_err,
                                     uint32_t *ov_err) {
    canStatus status;

    status = canReadErrorCounters(handle_, tx_err, rx_err, ov_err);

    return status;
  }

//------------------------------------------------------------------------------
//
  canStatus CanDriver::GetRxBufLevel(uint32_t *lvl) {
    canStatus status;

    status = canIoCtl(handle_, canIOCTL_GET_RX_BUFFER_LEVEL, lvl, sizeof(lvl));

    return status;
  }

//------------------------------------------------------------------------------
//
  canStatus CanDriver::GetTxBufLevel(uint32_t *lvl) {
    canStatus status;

    status = canIoCtl(handle_, canIOCTL_GET_TX_BUFFER_LEVEL, lvl, sizeof(lvl));

    return status;
  }

//------------------------------------------------------------------------------
//
  canStatus CanDriver::FlushRxBuffer() {
    canStatus status;

    status = canIoCtl(handle_, canIOCTL_FLUSH_RX_BUFFER, 0, 0);

    return status;
  }

//------------------------------------------------------------------------------
//
  canStatus CanDriver::FlushTxBuffer() {
    canStatus status;

    status = canIoCtl(handle_, canIOCTL_FLUSH_TX_BUFFER, 0, 0);

    return status;
  }

//------------------------------------------------------------------------------
//
  canStatus CanDriver::GetBusParams(long *freq, unsigned int *tseg1,
                                    unsigned int *tseg2, unsigned int *sjw,
                                    unsigned int *no_samp) {
    canStatus status;
    uint32_t dummy;

    status = canGetBusParams(handle_, freq, tseg1, tseg2, sjw, no_samp, &dummy);

    return status;
  }
}
