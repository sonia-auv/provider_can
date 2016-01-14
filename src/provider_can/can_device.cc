/**
 * \file	can_device.h
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

#include "can_device.h"
#include "can_dispatcher.h"

namespace provider_can {

//==============================================================================
// C / D T O R   S E C T I O N

CanDevice::CanDevice(DeviceClass device_id,
                     uint8_t unique_id,
                     std::shared_ptr<CanDispatcher> can_dispatcher,
                     std::string name){
  device_id_ = device_id;
  unique_id_ = unique_id;
  can_dispatcher_ = can_dispatcher;
  name_ = name;
}

//==============================================================================
// M E T H O D S   S E C T I O N

/*void CanDevice::Reset() {
  can_dispatcher_->SendResetRequest(device_id_, unique_id_);
}*/

//------------------------------------------------------------------------------
//
DeviceProperties CanDevice::GetProperties() {
  DeviceProperties properties;

  can_dispatcher_->GetDevicesProperties(device_id_,unique_id_,&properties);

  return properties;

}

//------------------------------------------------------------------------------
//
std::string CanDevice::GetName() {
  return name_;
}

//------------------------------------------------------------------------------
//
uint8_t* CanDevice::GetFault(){
    uint8_t *fault;
  if(can_dispatcher_->GetDeviceFault(device_id_,unique_id_,fault)
     == SONIA_DEVICE_FAULT){
    return fault;
  }
  else
    return NULL;
}

//------------------------------------------------------------------------------
//
bool CanDevice::GetPingStatus(){
  bool response;
  if(can_dispatcher_->VerifyPingResponse(device_id_,unique_id_,&response)
     != SONIA_DEVICE_NOT_PRESENT)
    return response;
  else
    return false;
}

//------------------------------------------------------------------------------
//

void CanDevice::Ping(){
  can_dispatcher_->PingDevice(device_id_,unique_id_);
}

//------------------------------------------------------------------------------
//

bool CanDevice::DevicePresenceCheck(){
  if(can_dispatcher_->FindDevice(device_id_,unique_id_)
            != SONIA_DEVICE_NOT_PRESENT)
    return true;
  else
    return false;
}

//------------------------------------------------------------------------------
//

/*void CanDevice::SleepMode(){
  can_dispatcher_->SendSleepRequest(device_id_,unique_id_);
}*/

//------------------------------------------------------------------------------
//

/*void CanDevice::WakeUp(){
  can_dispatcher_->SendWakeUpRequest(device_id_,unique_id_);
}*/

//------------------------------------------------------------------------------
//
void CanDevice::PushMessage(uint16_t message_id, uint8_t *buffer, uint8_t ndata)  {
    can_dispatcher_->PushUnicastMessage(device_id_,unique_id_,message_id,buffer,ndata);
}

//------------------------------------------------------------------------------
//
  std::vector<CanMessage> CanDevice::FetchMessages(){
  std::vector<CanMessage> buffer;
  can_dispatcher_->FetchMessages(device_id_,unique_id_,buffer);
  return buffer;
}

}