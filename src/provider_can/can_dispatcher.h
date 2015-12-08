/**
 * \file	can_dispatcher.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	07/12/2015
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

#ifndef PROVIDER_CAN_CAN_DISPATCHER_H_
#define PROVIDER_CAN_CAN_DISPATCHER_H_

#include "can_driver.h"
#include <stdint.h>

namespace provider_can {

//============================================================================
// T Y P E D E F   A N D   E N U M

    const int MAX_NUM_OF_DEVICES = 25;
    const int RAW_RX_BUFFER_SIZE = 500;
    const int RX_BUFFER_SIZE = 25;
    const uint32_t DEVICE_ADDRESS_MASK = 0xFFFFF000;

    typedef struct {
        uint8_t firmware_version;
        uint8_t uc_signature;
        uint8_t capabilities;
        uint8_t poll_rate = 10;
    }DeviceProperties;

    typedef struct {
        uint32_t num_of_messages;
        CanMessage buffer[RAW_RX_BUFFER_SIZE];
    }RawBuffer;

    typedef struct {
        uint32_t global_address;
        DeviceProperties device_properties;
        uint8_t num_of_messages;
        CanMessage buffer[RX_BUFFER_SIZE];
    }CanDeviceBuffer;

    class CanDispatcher {
    public:
        //============================================================================
        // P U B L I C   C / D T O R S

        //! Constructor
        CanDispatcher(uint32_t chan, uint32_t baudrate);

        // Destructor
        ~CanDispatcher();

        //============================================================================
        // P U B L I C   M E T H O D S

        uint32_t setPollRate(uint32_t address, uint8_t poll_rate);
        uint8_t fetchMessages(uint32_t address, CanMessage *buffer);
        void sendMessage(CanMessage *message);
        uint32_t getDevicesProperties(uint32_t address, DeviceProperties *properties);



    private:
        //============================================================================
        // P R I V A T E   M E T H O D S

        void sendIdRequest();
        void pollDevices(); // TODO: la fonctionnalité RTR devra être implémentée dans l'élé du sub
        void listDevices();
        void readMessages();
        void dispatchMessages();

        //============================================================================
        // P R I V A T E   M E M B E R S

        CanDeviceBuffer *rx_dispatched_buffer_;

        RawBuffer rx_raw_buffer_;
        RawBuffer tx_raw_buffer_;

        uint8_t nDevicesPresent;

        CanDriver canDriver_;

    };

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DISPATCHER_H_
