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

//------------------------------------------------------------------------------
//
CanDriver::CanDriver(unsigned int chan, long baudrate)
        : baudrate_(baudrate),
          channel_(chan)
{
    if (!initUsbDevice()) throw ExceptionCanDeviceNotFound();
}

//------------------------------------------------------------------------------
//
CanDriver::~CanDriver() { }

//==============================================================================
// M E T H O D S   S E C T I O N

canStatus CanDriver::open(void) {
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

bool CanDriver::initUsbDevice(void) {
    canStatus status;

    status = open();                // Open CAN channel

    if (status < canOK) {           // If open failed
        //printErrorText(status);
        return false;
    }

    status = setBusParams();        // Set CAN parameters

    if (status < canOK) {           // If parameters set failed
        //printErrorText(status);
        return false;
    }

    status = setBusOn();            // Turn on the channel

    if (status < canOK) {           // If bus can't turn on
        //printErrorText(status);
        return false;
    }

    return true;                    // Init success
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
            //status = canSetBusParams(handle_, baudrate_, tseg1, tseg2, sjw, noSamp, 0);
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

canStatus CanDriver::writeMessage(CanMessage *msg) {
    canStatus status;
    // cout << "writing id " << msg.id << endl;
    status = canWrite(handle_, msg->id, msg->data, msg->dlc, msg->flag);

    return status;
}

//------------------------------------------------------------------------------
//

canStatus CanDriver::readMessages(CanMessage *msg) {
    canStatus status;
    long int id;
    long unsigned int time;

    status = canRead(handle_, &id, &msg->data, &msg->dlc, &msg->flag, &time);

    msg->id = id;
    msg->time = time;

    return status;
}
