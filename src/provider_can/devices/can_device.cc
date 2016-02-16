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
                     const std::string &name) ATLAS_NOEXCEPT
    : device_id_(device_id),
      unique_id_(unique_id),
      can_dispatcher_(can_dispatcher),
      name_(name) {}

//------------------------------------------------------------------------------
//
CanDevice::~CanDevice() ATLAS_NOEXCEPT {}

}  // namespace provider_can
