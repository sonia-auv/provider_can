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

    // Devices capabilities
    const uint8_t ISP = 0x01;
    const uint8_t RESET = 0x02;
    const uint8_t SLEEP = 0x04;

    const uint32_t CAN_SEND_TIMEOUT = 10;

    const int MAX_NUM_OF_DEVICES = 25;
    const int RAW_RX_BUFFER_SIZE = 500;
    const int DISPATCHED_RX_BUFFER_SIZE = 25;
    const uint32_t DEVICE_ADDRESS_MASK = 0xFFFFF000;

    typedef struct {
        uint16_t firmware_version;
        uint32_t uc_signature;
        uint8_t capabilities;
        uint8_t device_data;
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
        CanMessage rx_buffer[DISPATCHED_RX_BUFFER_SIZE];
        bool device_fault = false;
    }CanDevice;

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

        void setPollRate(uint32_t address, uint8_t poll_rate);

        void sendResetRequest(uint32_t address);
        void sendSleepRequest(uint32_t address);
        void sendWakeUpRequest(uint32_t address);
        void pushMessage(CanMessage *message);

        uint8_t fetchMessages(uint32_t address, CanMessage *buffer);
        void getDevicesProperties(uint32_t address, DeviceProperties *properties);



    private:
        //============================================================================
        // P R I V A T E   M E T H O D S

        /**
        * Allows the user to send an ID request to devices on SONIA's CAN bus.
        *
        * Every device will answer its ID and its parameters after receiving the
        * ID request. This function does not handle the responses to the ID request.
        *
        */
        void sendIdRequest();


        void pollDevices(); // TODO: la fonctionnalité RTR devra être implémentée dans l'élé du sub

        /**
        * Sends an ID request on CAN bus and list all addresses received.
        *
        * This function creates a CanDevice struct for each device that answered to the
        * ID request. this allows the users of the class to read messages from the devices
        * that answered. If a device did not answer to the ID request, no struct will be created
        * for it and if it sends other messages, they will be dropped.
        */
        void listDevices();

        /**
        * Reads all messages received on CAN bus and stores them into rx_raw_buffer_
        */
        void readMessages();

        /**
        * sends all messages contained in tx_raw_buffer_
        */
        void sendMessages();

        /**
        * Dispatch all messages contained into rx_raw_buffer_ to each respective CanDevice
        * struct created by listDevices().
        */
        void dispatchMessages();

        //============================================================================
        // P R I V A T E   M E M B E R S

        CanDevice *devices_list_;       // List of devices present on CAN bus

        RawBuffer rx_raw_buffer_;       // Buffer directly taken from KVaser
        RawBuffer tx_raw_buffer_;

        uint8_t nDevicesPresent;        // Number of devices detected

        CanDriver canDriver_;           // Can communication object

    };

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DISPATCHER_H_
