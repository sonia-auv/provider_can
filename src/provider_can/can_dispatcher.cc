/**
* \file	can_dispatcher.cc
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

#include "can_dispatcher.h"
#include "can_driver.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>

using namespace provider_can;

//==============================================================================
// C / D T O R   S E C T I O N

CanDispatcher::CanDispatcher(uint32_t chan, uint32_t baudrate):
        canDriver_(chan,baudrate)
{
    canDriver_.flushRxBuffer();
    canDriver_.flushTxBuffer();
    listDevices();
}

//------------------------------------------------------------------------------
//
CanDispatcher::~CanDispatcher() {}

//==============================================================================
// M E T H O D S   S E C T I O N

void CanDispatcher::listDevices(){

    bool new_device_found = true;
    uint32_t devicesAddresses[MAX_NUM_OF_DEVICES];

    sendIdRequest();        // Ask ID from every device on CAN bus
    usleep(1000);           // Wait for all responses
    readMessages();         // Reads all messages into rx_raw_buffer_

    for(int j = 0; j < rx_raw_buffer_.num_of_messages; j++) {   // For each messages received during sleep,
        for (int i = 0; i < nDevicesPresent; i++) {             // For each device found until now
            if ((rx_raw_buffer_.buffer[j].id & DEVICE_ADDRESS_MASK) == devicesAddresses[i])// Verify if the ID has
                new_device_found = false;                       // already been seen. If so, avoids adding it again.
        }
        if(new_device_found)                                    // If the messages contains a new address
            devicesAddresses[nDevicesPresent++] = (rx_raw_buffer_.buffer[j].id & DEVICE_ADDRESS_MASK);// adds a device

        new_device_found = true;                                // Ready for next loop
    }

    devices_list_ = new CanDevice[nDevicesPresent];

    for(int i = 0; i < nDevicesPresent; i++)                    // Saves address for each dispatched device
        devices_list_[i].global_address = devicesAddresses[i];

    dispatchMessages();         // Dispatch and saves all received messages
}

//------------------------------------------------------------------------------
//
void CanDispatcher::dispatchMessages(){

    for(int j = 0; j < rx_raw_buffer_.num_of_messages; j++){    // For each message contained in raw buffer
        for(int i = 0; i < nDevicesPresent; i++){               // For each detected device
            if(devices_list_[i].global_address == rx_raw_buffer_.buffer[j].id){
                // If the ID receiveds correspond to an ID request response
                // saves device's parameters.
                devices_list_[i].device_properties.firmware_version =
                        (rx_raw_buffer_.buffer[j].data[1] << 8) | rx_raw_buffer_.buffer[j].data[0];

                devices_list_[i].device_properties.uc_signature =
                        (rx_raw_buffer_.buffer[j].data[4] << 16) | (rx_raw_buffer_.buffer[j].data[3] << 8) |
                                rx_raw_buffer_.buffer[j].data[2];

                devices_list_[i].device_properties.capabilities =
                        rx_raw_buffer_.buffer[j].data[5];

                devices_list_[i].device_properties.device_data =
                        rx_raw_buffer_.buffer[j].data[6];
            }
            else if(devices_list_[i].global_address | 0xFF == rx_raw_buffer_.buffer[j].id){
                // If the ID received correspond to a device fault
                devices_list_[i].device_fault = true;
            }
            else if(devices_list_[i].global_address == (rx_raw_buffer_.buffer[j].id & DEVICE_ADDRESS_MASK)) {
                // If the ID received correspond to any other message
                if(devices_list_[i].num_of_messages >= DISPATCHED_RX_BUFFER_SIZE)
                    devices_list_[i].num_of_messages = 0;// Avoids buffer overflow

                // Saves message into dispatched devices' buffers.
                devices_list_[i].rx_buffer[devices_list_[i].num_of_messages++] =
                        rx_raw_buffer_.buffer[j];

                i = nDevicesPresent;
            }
        }
    }
    rx_raw_buffer_.num_of_messages = 0;         // indicates that all messages have been read and dispatched.
}

//------------------------------------------------------------------------------
//
void CanDispatcher::readMessages(){
    canStatus status;
    CanMessage *buffer;
    buffer = canDriver_.readAllMessages(&status, &rx_raw_buffer_.num_of_messages);// TODO: Verify if read was successful

    memcpy(rx_raw_buffer_.buffer, buffer, rx_raw_buffer_.num_of_messages);
}

//------------------------------------------------------------------------------
//
void CanDispatcher::sendMessages(){
    for(int i = 0; i < tx_raw_buffer_.num_of_messages; i++)
        canDriver_.writeMessage(tx_raw_buffer_.buffer[i],CAN_SEND_TIMEOUT);// TODO: Verify if write was successful

    tx_raw_buffer_.num_of_messages = 0;
}

//------------------------------------------------------------------------------
//
void CanDispatcher::sendIdRequest(){
    CanMessage msg;

    msg.id = 0;
    msg.data[0] = 0;
    msg.dlc = 0;
    msg.flag = canMSG_EXT;
    msg.time = 0;

    canDriver_.writeMessage(msg,1);
}

//------------------------------------------------------------------------------
//
void CanDispatcher::setPollRate(uint32_t address, uint8_t poll_rate){
    for(int i = 0; i < nDevicesPresent; i++)
        if(devices_list_[i].global_address == (address & DEVICE_ADDRESS_MASK)){
            devices_list_[i].device_properties.poll_rate = poll_rate;
            i = nDevicesPresent;
        }

}

//------------------------------------------------------------------------------
//
uint8_t CanDispatcher::fetchMessages(uint32_t address, CanMessage *buffer){

    uint8_t num_of_messages;

    for(int i = 0; i < nDevicesPresent; i++)
        if(devices_list_[i].global_address == (address & DEVICE_ADDRESS_MASK)){
            buffer = devices_list_[i].rx_buffer;
            i = nDevicesPresent;
            num_of_messages = devices_list_[i].num_of_messages;
            devices_list_[i].num_of_messages = 0;
        }
    return num_of_messages;
}

//------------------------------------------------------------------------------
//
void CanDispatcher::getDevicesProperties(uint32_t address, DeviceProperties *properties){
    for(int i = 0; i < nDevicesPresent; i++)
        if(devices_list_[i].global_address == (address & DEVICE_ADDRESS_MASK)){
            properties = &devices_list_[i].device_properties;
            i = nDevicesPresent;
        }
}

//------------------------------------------------------------------------------
//
void CanDispatcher::pushMessage(CanMessage *message){
    tx_raw_buffer_.buffer[tx_raw_buffer_.num_of_messages++] = *message;
}

//------------------------------------------------------------------------------
//
void CanDispatcher::pollDevices(){

}

//------------------------------------------------------------------------------
//
void CanDispatcher::sendResetRequest(uint32_t address){

}

//------------------------------------------------------------------------------
//
void CanDispatcher::sendSleepRequest(uint32_t address){

}

//------------------------------------------------------------------------------
//
void CanDispatcher::sendWakeUpRequest(uint32_t address){

}