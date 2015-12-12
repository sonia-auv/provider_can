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
#include <sys/times.h>

namespace provider_can {

//============================================================================
// T Y P E D E F   A N D   E N U M

    // Devices capabilities
    const uint8_t ISP = 0x01;// TODO: put these defines in CAN.h
    const uint8_t RESET = 0x02;
    const uint8_t SLEEP = 0x04;

    const uint32_t UNICAST = 0x10000000;
    const uint32_t RESET_REQ = 0xfe;
    const uint32_t WAKEUP_REQ = 0xf1;
    const uint32_t SLEEP_REQ = 0xf0;

    const uint8_t UNIQUE_ID_POSITION = 12;
    const uint8_t DEVICE_ID_POSITION = 20;

    const uint32_t ID_REQ_WAIT = 100000;
    const uint8_t DISCOVERY_TRIES = 10;
    const uint8_t DISCOVERY_DELAY = 5;

    const uint32_t CAN_SEND_TIMEOUT = 10;

    const uint32_t ID_REQ_INTERVAL_MS = 5000;

    const int MAX_NUM_OF_DEVICES = 30;
    const int RAW_TX_BUFFER_SIZE = 25;
    const int DISPATCHED_RX_BUFFER_SIZE = 50;
    const uint32_t DEVICE_ADDRESS_MASK = 0x7FFFF000;

    typedef struct {
        uint16_t firmware_version;
        uint32_t uc_signature;
        uint8_t capabilities;
        uint8_t device_data;
        uint8_t poll_rate = 100;
    }DeviceProperties;

    typedef struct {
        uint32_t num_of_messages = 0;
        CanMessage *buffer;
    }RawBuffer;

    typedef struct {
        uint32_t global_address;
        DeviceProperties device_properties;
        uint8_t num_of_messages;
        CanMessage rx_buffer[DISPATCHED_RX_BUFFER_SIZE];
        bool device_fault = false;
    }CanDevice;

    typedef enum { SONIA_DEVICE_NOT_PRESENT = 0, SONIA_DEVICE_PRESENT = 1, SONIA_DEVICE_FAULT = -1}
            SoniaDeviceStatus;

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

        /**
        * The function sets the poll_rate value of the DeviceProperties struct of the
        * selected device.
        *
        * This indicates how many times per second we want an RTR to be sent to the device.
        *
        * \param device_id SONIA Device ID to look for
        * \param unique_id SONIA unique ID to look for
        * \param poll_rate polling rate, in ms
        * \return SoniaDeviceStatus enum
        */
        SoniaDeviceStatus setPollRate(uint8_t device_id,uint8_t unique_id,
                                      uint8_t poll_rate);

        /**
        * The next functions sends SONIA specific messages to selected device
        *
        * \param device_id SONIA Device ID to look for
        * \param unique_id SONIA unique ID to look for
        * \return SoniaDeviceStatus enum
        */
        SoniaDeviceStatus sendResetRequest(uint8_t device_id,uint8_t unique_id);
        SoniaDeviceStatus sendSleepRequest(uint8_t device_id,uint8_t unique_id);
        SoniaDeviceStatus sendWakeUpRequest(uint8_t device_id,uint8_t unique_id);

        /**
        * The function puts a new message in the tx_queue. It will be sent later in the process
        *
        * The function verifies if the address of the message is a known device. SoniaDeviceStatus will
        * indicate if so. If it is not the case, the message will be sent anyway.
        *
        * \param device_id SONIA Device ID
        * \param unique_id SONIA unique ID
        * \param message_id SONIA message ID
        * \param buffer message content
        * \param ndata message length
        * \return SoniaDeviceStatus enum
        */
        SoniaDeviceStatus pushMessage(uint8_t device_id, uint8_t
        unique_id, uint16_t message_id ,uint8_t *buffer, uint8_t ndata);

        /**
        * The function returns the rx_buffer of the selected device
        *
        * The rx_buffer returned contains all received messages since last call of this function.
        * If the device asked does not exist, SoniaDeviceStatus will indicate it and no rx_buffer will be returned.
        * rx_buffer will only contain data messages, not ID request responses nor device fault messages. These are
        * filtered by dispatchMessages().
        *
        * \param device_id SONIA Device ID to look for
        * \param unique_id SONIA unique ID to look for
        * \param buffer device's rx_buffer
        * \param num_of_messages number of messages read
        * \return SoniaDeviceStatus enum
        */
        SoniaDeviceStatus fetchMessages(uint8_t device_id,
                                        uint8_t unique_id,
                                        CanMessage *&buffer,
                                        uint8_t *num_of_messages);

        /**
        * The function returns the devices's properties
        *
        * \param device_id SONIA Device ID to look for
        * \param unique_id SONIA unique ID to look for
        * \param properties device's properties
        * \return SoniaDeviceStatus enum
        */
        SoniaDeviceStatus getDevicesProperties(uint8_t device_id,
                                               uint8_t unique_id,
                                               DeviceProperties *properties);

        /**
        * The function clears the specified device fault flag. This avoids SoniaDeviceStatus to always take
        * SONIA_DEVICE_FAULT value for every functions of this class when a fault has been received.
        *
        * \param device_id SONIA Device ID to look for
        * \param unique_id SONIA unique ID to look for
        * \return SoniaDeviceStatus enum
        */
        SoniaDeviceStatus clearFault(uint8_t device_id,uint8_t unique_id);



        uint8_t getNumberOfDevices();
        uint8_t getUnknownAddresses(uint32_t *&addresses);

        /**
        * This process has to be called periodically. It handles reading and sending messages.
        *
        */
        void providerCanProcess();

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
        canStatus sendIdRequest();


        /**
        * Sends RTR to devices at rate specified in DeviceProperties struct, for every
        * known device.
        *
        * This function loops through all known device and compares the poll_rate with a timer value
        * to known if polling is required or not. use setPollRate() to specify a poll rate for a
        * specific device. Default poll_rate is 100ms.
        *
        */
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
        canStatus readMessages();

        /**
        * sends all messages contained in tx_raw_buffer_
        */
        canStatus sendMessages();

        /**
        * Dispatch all messages contained into rx_raw_buffer_ to each respective
        * CanDevice struct created by listDevices().
        * This function also filters ID_request responses and device_fault
        * messages to set DeviceProperties struct.
        */
        void dispatchMessages();

        /**
        * The function returns the device_list_ index value which contains the
        * selected address/device
        *
        * \param address address to look for
        * \param device_id SONIA Device ID to look for
        * \param unique_id SONIA unique ID to look for
        * \param index device_list_ index found
        * \return SoniaDeviceStatus enum
        */
        SoniaDeviceStatus getDeviceIndex(uint8_t device_id, uint8_t
        unique_id, int *index);
        SoniaDeviceStatus getAddressIndex(uint32_t address, int
        *index);

        /**
        * The function sends and RTR on CAN bus with selected address
        *
        * \param address ID of the RTR
        */
        void sendRTR(uint32_t address);

        /**
        * Adds an address to the unknown addresses table. If unknown
        * addresses are found, providerCanProcess will send ID requests to
        * be sure every present device are known.
        *
        * \param address
        */
        void addUnknownAddress(uint32_t address);

        //============================================================================
        // P R I V A T E   M E M B E R S

        CanDevice devices_list_[MAX_NUM_OF_DEVICES]; // List of devices present on CAN bus

        uint32_t unknown_addresses_table_[MAX_NUM_OF_DEVICES];
        uint8_t nunknown_addresses_;

        RawBuffer rx_raw_buffer_;           // Buffer directly taken from KVaser
        RawBuffer tx_raw_buffer_;

        uint8_t ndevices_present_;          // Number of devices detected

        CanDriver canDriver_;               // Can communication object

        timespec ticks_per_sec_;
        timespec actual_time_;
        timespec initial_time_;
        timespec id_req_time_;

        uint8_t discovery_tries_;


    };

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DISPATCHER_H_
