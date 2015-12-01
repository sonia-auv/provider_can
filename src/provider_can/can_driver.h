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

const long SONIA_CAN_BAUD_1M = 1000000;
const long SONIA_CAN_BAUD_500K = 500000;
const long SONIA_CAN_BAUD_250K = 250000;
const long SONIA_CAN_BAUD_125K = 125000;
const long SONIA_CAN_BAUD_100K = 100000;
const long SONIA_CAN_BAUD_62K = 62000;
const long SONIA_CAN_BAUD_50K = 50000;



    class CanDriver {
        //============================================================================
        // T Y P E D E F   A N D   E N U M

        typedef struct {
            /// Short desc.
            unsigned int id;
            /// Short desc.
            unsigned char data[8];
            /// Short desc.
            unsigned int dlc;
            /// Short desc.
            unsigned int flag;
            /// Short desc.
            unsigned int time;
        } CanMessage;

        typedef enum { SONIA_CAN_OK = 0, SONIA_CAN_ERR = -1 } SoniaCanStatus;


    public:
        //============================================================================
        // P U B L I C   C / D T O R S

        //! Constructor
        CanDriver(unsigned int chan, long baudrate);

        // Destructor
        ~CanDriver();

        //============================================================================
        // P U B L I C   M E T H O D S

        canStatus readMessages(CanMessage *msg);
        canStatus writeMessage(CanMessage *msg);

        //============================================================================
        // P U B L I C   M E M B E R S

    private:
        //============================================================================
        // P R I V A T E   M E T H O D S

        unsigned int channel_;
        canHandle handle_;
        unsigned int baudrate_;
        unsigned int msg_count_;
        bool initUsbDevice();
        canStatus open();
        canStatus setBusParams();
        canStatus setBusOff();
        canStatus setBusOn();
        canStatus close();

        //============================================================================
        // P R I V A T E   M E M B E R S
    };

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DRIVER_H_
