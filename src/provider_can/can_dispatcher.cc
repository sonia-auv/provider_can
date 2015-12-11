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
#include <unistd.h>
#include <string.h>

using namespace provider_can;

//==============================================================================
// C / D T O R   S E C T I O N

CanDispatcher::CanDispatcher(uint32_t chan, uint32_t baudrate):
        canDriver_(chan,baudrate)
{

    clock_gettime(CLOCK_REALTIME, &actual_time_);
    clock_gettime(CLOCK_REALTIME, &initial_time_);

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

    sendIdRequest();        // Ask ID from every device on CAN bus
    usleep(1000);           // Wait for all responses
    readMessages();         // Reads all messages into rx_raw_buffer_

    for(int j = 0; j < rx_raw_buffer_.num_of_messages; j++) {   // For each messages received during sleep,
        for (int i = 0; i < ndevices_present_; i++) {             // For each device found until now
            if ((rx_raw_buffer_.buffer[j].id & DEVICE_ADDRESS_MASK) == devices_list_[i].global_address)// Verify if the ID has
                new_device_found = false;                       // already been seen. If so, avoids adding it again.
        }
        if(new_device_found)                                    // If the messages contains a new address
            devices_list_[ndevices_present_++].global_address = (rx_raw_buffer_.buffer[j].id & DEVICE_ADDRESS_MASK);// adds a device

        new_device_found = true;                                // Ready for next loop
    }

    dispatchMessages();         // Dispatch and saves all received messages
}

//------------------------------------------------------------------------------
//
void CanDispatcher::dispatchMessages(){

    SoniaDeviceStatus status;
    int index;

    for(int j = 0; j < rx_raw_buffer_.num_of_messages; j++){    // For each message contained in raw buffer

        status = getDeviceIndex(rx_raw_buffer_.buffer[j].id, &index);   // get the device_list index where address is
                                                                        // located

        if(status != SONIA_DEVICE_NOT_PRESENT) {        // If device exists
            if (devices_list_[index].global_address == rx_raw_buffer_.buffer[j].id) {
                // If the ID received correspond to an ID request response
                // saves device's parameters.
                devices_list_[index].device_properties.firmware_version =
                        (rx_raw_buffer_.buffer[j].data[1] << 8) | rx_raw_buffer_.buffer[j].data[0];

                devices_list_[index].device_properties.uc_signature =
                        (rx_raw_buffer_.buffer[j].data[4] << 16) | (rx_raw_buffer_.buffer[j].data[3] << 8) |
                        rx_raw_buffer_.buffer[j].data[2];

                devices_list_[index].device_properties.capabilities =
                        rx_raw_buffer_.buffer[j].data[5];

                devices_list_[index].device_properties.device_data =
                        rx_raw_buffer_.buffer[j].data[6];
            }
            else if (devices_list_[index].global_address | 0xFF == rx_raw_buffer_.buffer[j].id) {
                // If the ID received correspond to a device fault
                devices_list_[index].device_fault = true;
            }
            else if (devices_list_[index].global_address == (rx_raw_buffer_.buffer[j].id & DEVICE_ADDRESS_MASK)) {
                // If the ID received correspond to any other message
                if (devices_list_[index].num_of_messages >= DISPATCHED_RX_BUFFER_SIZE)
                    devices_list_[index].num_of_messages = 0;// Avoids buffer overflow

                // Saves message into dispatched devices' buffers.
                devices_list_[index].rx_buffer[devices_list_[index].num_of_messages++] =
                        rx_raw_buffer_.buffer[j];
            }
        }
        else{   // If address is unknown
                // Adds the address to the unknown addresses table
            unknown_addresses_table_[nunknown_addresses_++] = (rx_raw_buffer_.buffer[j].id & DEVICE_ADDRESS_MASK);
            for(int i = 0; i < nunknown_addresses_; i++)
                if(devices_list_[i].global_address == (rx_raw_buffer_.buffer[j].id & DEVICE_ADDRESS_MASK)) {
                    nunknown_addresses_--;
                    i = nunknown_addresses_;
                }
        }
    }
    rx_raw_buffer_.num_of_messages = 0;         // indicates that all messages have been read and dispatched.
}

//------------------------------------------------------------------------------
//
canStatus CanDispatcher::readMessages(){
    canStatus status;
    CanMessage *buffer;
    status = canDriver_.readAllMessages(buffer, &rx_raw_buffer_.num_of_messages);

    memcpy(rx_raw_buffer_.buffer, buffer, rx_raw_buffer_.num_of_messages);

    return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDispatcher::sendMessages(){
    canStatus status;

    for(int i = 0; i < tx_raw_buffer_.num_of_messages; i++) {
        status = canDriver_.writeMessage(tx_raw_buffer_.buffer[i], CAN_SEND_TIMEOUT);
        if(status != canOK)
            i = tx_raw_buffer_.num_of_messages;
    }

    tx_raw_buffer_.num_of_messages = 0;

    return status;
}

//------------------------------------------------------------------------------
//
canStatus CanDispatcher::sendIdRequest(){
    CanMessage msg;

    msg.id = 0;
    msg.data[0] = 0;
    msg.dlc = 0;
    msg.flag = canMSG_EXT;
    msg.time = 0;

    return(canDriver_.writeMessage(msg,1));
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::setPollRate(uint32_t address, uint8_t poll_rate){
    SoniaDeviceStatus status;
    int index;

    status = getDeviceIndex(address, &index);

    if(status != SONIA_DEVICE_NOT_PRESENT){
        devices_list_[index].device_properties.poll_rate = poll_rate;
    }

    return status;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::fetchMessages(uint32_t address, CanMessage *buffer, uint8_t *num_of_messages){

    SoniaDeviceStatus status;
    int index;

    status = getDeviceIndex(address, &index);

    if(status != SONIA_DEVICE_NOT_PRESENT){
        buffer = devices_list_[index].rx_buffer;
        *num_of_messages = devices_list_[index].num_of_messages;
        devices_list_[index].num_of_messages = 0;
    }

    return status;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::getDevicesProperties(uint32_t address, DeviceProperties *properties){
    SoniaDeviceStatus status;
    int index;

    status = getDeviceIndex(address, &index);

    if(status != SONIA_DEVICE_NOT_PRESENT)
        properties = &devices_list_[index].device_properties;

    return status;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::pushMessage(CanMessage *message){

    int index;

    tx_raw_buffer_.buffer[tx_raw_buffer_.num_of_messages++] = *message;

    return getDeviceIndex(message->id, &index);
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::getDeviceIndex(uint32_t address, int *index){
    SoniaDeviceStatus status = SONIA_DEVICE_NOT_PRESENT;

    for(int i = 0; i < ndevices_present_; i++)
        if(devices_list_[i].global_address == (address & DEVICE_ADDRESS_MASK)){
            *index = i;
            i = ndevices_present_;

            if(devices_list_[i].device_fault == true) {
                status = SONIA_DEVICE_FAULT;
            }
            else{
                status = SONIA_DEVICE_PRESENT;
            }
        }

    return status;
}

//------------------------------------------------------------------------------
//
void CanDispatcher::pollDevices(){// TODO: does not work

    for(int i = 0; i < ndevices_present_; i++){
        if((int)((actual_time_.tv_nsec - initial_time_.tv_nsec) %
                   ((1/devices_list_[i].device_properties.poll_rate)*1000000000)) == 0);
            sendRTR(devices_list_[i].global_address);
    }


}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::clearFault(uint32_t address){
    SoniaDeviceStatus status;
    int index;
    status = getDeviceIndex(address, &index);

    if(status != SONIA_DEVICE_NOT_PRESENT)
        devices_list_[index].device_fault = false;

    return status;
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::sendResetRequest(uint32_t address){
    CanMessage msg;

    msg.id = UNICAST | (address & DEVICE_ADDRESS_MASK) | RESET_REQ;
    msg.data[0] = 0;
    msg.dlc = 0;
    msg.flag = canMSG_EXT;
    msg.time = 0;

    canDriver_.writeMessage(msg,1);

    int index;
    return(getDeviceIndex(address, &index));
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::sendSleepRequest(uint32_t address){
    CanMessage msg;

    msg.id = UNICAST | (address & DEVICE_ADDRESS_MASK) | SLEEP_REQ;
    msg.data[0] = 0;
    msg.dlc = 0;
    msg.flag = canMSG_EXT;
    msg.time = 0;

    canDriver_.writeMessage(msg,1);

    int index;
    return(getDeviceIndex(address, &index));
}

//------------------------------------------------------------------------------
//
SoniaDeviceStatus CanDispatcher::sendWakeUpRequest(uint32_t address){
    CanMessage msg;

    msg.id = UNICAST | (address & DEVICE_ADDRESS_MASK) | WAKEUP_REQ;
    msg.data[0] = 0;
    msg.dlc = 0;
    msg.flag = canMSG_EXT;
    msg.time = 0;

    canDriver_.writeMessage(msg,1);

    int index;
    return(getDeviceIndex((address & DEVICE_ADDRESS_MASK), &index));
}

//------------------------------------------------------------------------------
//
void CanDispatcher::sendRTR(uint32_t address){
    CanMessage msg;

    msg.id = address;
    msg.data[0] = 0;
    msg.dlc = 0;
    msg.flag = canMSG_RTR;
    msg.time = 0;

    canDriver_.writeMessage(msg,1);
}
//------------------------------------------------------------------------------
//
uint8_t CanDispatcher::getNumberOfDevices(){
    return ndevices_present_;
}

//------------------------------------------------------------------------------
//
uint8_t CanDispatcher::getUnknownAddresses(uint32_t *addresses){
    addresses = unknown_addresses_table_;
    return nunknown_addresses_;
}

//------------------------------------------------------------------------------
//
void CanDispatcher::providerCanProcess(){
    clock_gettime(CLOCK_REALTIME, &actual_time_);

    readMessages();
    dispatchMessages();
    sendMessages();
    pollDevices();

}