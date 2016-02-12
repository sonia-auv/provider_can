/**
 * \file	can_device.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	04/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_can/devices/can_device.h"

namespace provider_can {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CanDevice::CanDevice(const DeviceClass &device_id, uint8_t unique_id,
                     const CanDispatcher::Ptr &can_dispatcher,
                     const std::string &name) ATLAS_NOEXCEPT :
    device_id_(device_id),
    unique_id_(unique_id),
    can_dispatcher_(can_dispatcher),
    name_(name)
{}

//------------------------------------------------------------------------------
//
CanDevice::~CanDevice() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D S   S E C T I O N

/*
//------------------------------------------------------------------------------
//void CanDevice::Reset() {
  can_dispatcher_->SendResetRequest(device_id_, unique_id_);
}
 */

//------------------------------------------------------------------------------
//
DeviceProperties CanDevice::GetProperties() {
  DeviceProperties properties;

  can_dispatcher_->GetDevicesProperties(device_id_, unique_id_, properties);

  return properties;
}

//------------------------------------------------------------------------------
//
uint8_t *CanDevice::GetFault() {
  uint8_t *fault;
  if (can_dispatcher_->GetDeviceFault(device_id_, unique_id_, fault) ==
      SONIA_DEVICE_FAULT) {
    return fault;
  } else
    return NULL;
}

//------------------------------------------------------------------------------
//
bool CanDevice::GetPingStatus() {
  bool response;
  if (can_dispatcher_->VerifyPingResponse(device_id_, unique_id_, &response) !=
      SONIA_DEVICE_NOT_PRESENT)
    return response;
  else
    return false;
}

//------------------------------------------------------------------------------
//
void CanDevice::Ping() { can_dispatcher_->PingDevice(device_id_, unique_id_); }

//------------------------------------------------------------------------------
//
bool CanDevice::DevicePresenceCheck() {
  if (can_dispatcher_->FindDevice(device_id_, unique_id_) !=
      SONIA_DEVICE_NOT_PRESENT)
    return true;
  else
    return false;
}

/*
//------------------------------------------------------------------------------
//
void CanDevice::SleepMode(){
  can_dispatcher_->SendSleepRequest(device_id_,unique_id_);
}
*/

/*
//------------------------------------------------------------------------------
//
void CanDevice::WakeUp(){
  can_dispatcher_->SendWakeUpRequest(device_id_,unique_id_);
}
*/

//------------------------------------------------------------------------------
//
void CanDevice::PushMessage(uint16_t message_id, uint8_t *buffer,
                            uint8_t ndata) {
  can_dispatcher_->PushUnicastMessage(device_id_, unique_id_, message_id,
                                      buffer, ndata);
}

//------------------------------------------------------------------------------
//
std::vector<CanMessage> CanDevice::FetchMessages() {
  std::vector<CanMessage> buffer;
  can_dispatcher_->FetchMessages(device_id_, unique_id_, buffer);
  return buffer;
}

//------------------------------------------------------------------------------
//
std::vector<ComputerMessage> CanDevice::FetchComputerMessages() {
  std::vector<ComputerMessage> buffer;
  can_dispatcher_->FetchComputerMessages(device_id_, unique_id_, buffer);
  return buffer;
}

}  // namespace provider_can
