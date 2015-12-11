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

using namespace provider_can;

//==============================================================================
// C / D T O R   S E C T I O N

CanDriver::CanDriver(uint32_t chan, uint32_t baudrate)
    : baudrate_(baudrate), channel_(chan), tseg1_(0), tseg2_(0),
      sjw_(1), noSamp_(16) {
  if (!initUsbDevice()) throw ExceptionCanDeviceNotFound();
}

CanDriver::CanDriver(uint32_t chan, uint32_t baudrate, uint32_t ts1,
                   uint32_t ts2, uint32_t jump, uint32_t samp)
    : channel_(chan), baudrate_(baudrate), tseg1_(ts1), tseg2_(ts2),
      sjw_(jump), noSamp_(samp)

{
  if(!initUsbDevice())throw ExceptionCanDeviceNotFound();
}

//------------------------------------------------------------------------------
//
CanDriver::~CanDriver() {
  flushRxBuffer();
  flushTxBuffer();
}

//==============================================================================
// M E T H O D S   S E C T I O N
canStatus CanDriver::open(void) {
  handle_ = canOpenChannel(channel_, canWANT_EXCLUSIVE | canWANT_EXTENDED);
  // CAN channel handler

  unsigned short version = 0;
  version = canGetVersion();

  std::cout << "Canlib version = " << (version >> 8) << "." << (version & 0xFF)
            << std::endl;

  return (canStatus)handle_;
}

//------------------------------------------------------------------------------
//
bool CanDriver::initUsbDevice(void) {
  canStatus status;

  status = open();  // Open CAN channel

  if (status < canOK) {  // If open failed
     printErrorText(status);
    return false;
  }

  status = setBusParams();  // Set CAN parameters

  if (status < canOK) {  // If parameters set failed
     printErrorText(status);
    return false;
  }

  status = setBusOn();  // Turn on the channel

  if (status < canOK) {  // If bus can't turn on
     printErrorText(status);
    return false;
  }

  return true;  // Init success
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::setBusParams() {
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
       status = canSetBusParams(handle_, baudrate_, tseg1_, tseg2_, sjw_,
                                noSamp_, 0);
      break;
  }

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::setBusOff() {
  canStatus status;

  status = canBusOff(handle_);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::setBusOn() {
  canStatus status;

  status = canBusOn(handle_);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::close() {
  canStatus status;

  status = canClose(handle_);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::writeMessage(CanMessage msg, uint32_t timeout_msec) {
  canStatus status;
  if(timeout_msec == 0)
    status = canWrite(handle_, msg.id, msg.data, msg.dlc, msg.flag);
  else
    status = canWriteWait(handle_, msg.id, msg.data, msg.dlc, msg.flag,
                          timeout_msec);
  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::readMessage(CanMessage *msg, uint32_t timeout_msec) {
  canStatus status;
  long int id;
  long unsigned int time;

  if(timeout_msec == 0)
    status = canRead(handle_, &id, &msg->data, &msg->dlc, &msg->flag,
    &time);
  else
    status = canReadWait(handle_, &id, &msg->data, &msg->dlc, &msg->flag,
                         &time, timeout_msec);


  msg->id = id;
  msg->time = time;
  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::readAllMessages(CanMessage *msg_table, uint32_t
*num_of_messages) {
  long int id;
  long unsigned int time;
  canStatus status;


  status = getRxBufLevel(num_of_messages);   // get the reception buffer level

  if (status == canOK) {

    rx_buffer_ = new CanMessage[*num_of_messages]; // creates table to store
                                                  // all data

    for(uint32_t i = 0; i < *num_of_messages; i++) {// storing data
      status = readMessage(&rx_buffer_[i],0);

      if(status != canOK)
        i = *num_of_messages;
    }
  }

  msg_table = rx_buffer_;
  return status;
}

//------------------------------------------------------------------------------
//
void CanDriver::printErrorText(canStatus error) {
  char errMsg[128];
  errMsg[0] = '\0';

  canGetErrorText(error, errMsg, sizeof(errMsg));

  if(error != canOK)
    std::cout << " ERROR " << "(" << errMsg << ")" << std::endl;
  else
    std::cout << "(" << errMsg << ")";
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::setAcceptanceFilter(uint32_t enveloppe, int flag) {
  canStatus status;

  status = canAccept(handle_, enveloppe, flag);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::getErrorCount(uint32_t* txErr, uint32_t* rxErr,
                                   uint32_t* ovErr) {
  canStatus status;

  status = canReadErrorCounters(handle_, txErr, rxErr, ovErr);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::getRxBufLevel(uint32_t* lvl)
{
  canStatus status;

  status = canIoCtl(handle_, canIOCTL_GET_RX_BUFFER_LEVEL, lvl, sizeof(lvl));

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::getTxBufLevel(uint32_t* lvl)
{
  canStatus status;

  status = canIoCtl(handle_, canIOCTL_GET_TX_BUFFER_LEVEL, lvl, sizeof(lvl));

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::flushRxBuffer()
{
  canStatus status;

  status = canIoCtl(handle_, canIOCTL_FLUSH_RX_BUFFER, 0, 0);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::flushTxBuffer()
{
  canStatus status;

  status = canIoCtl(handle_, canIOCTL_FLUSH_TX_BUFFER, 0, 0);

  return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDriver::getBusParams(long* freq, unsigned int* tseg1,
                                 unsigned int* tseg2, unsigned int* sjw,
                                 unsigned int* no_samp)
{
  canStatus status;
  uint32_t dummy;

  status = canGetBusParams(handle_, freq, tseg1, tseg2, sjw, no_samp, &dummy);

  return status;
}