/**
 * \file	ekf_configuration.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	07/02/2016
 *
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

#include "provider_can/can_configuration.h"

namespace provider_can {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
CanConfiguration::CanConfiguration(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : baudrate(-4),
      device_id(1),
      unique_id(1),
      loop_rate(10),
      channel(0),
      nh_(nh) {}

//------------------------------------------------------------------------------
//
CanConfiguration::CanConfiguration(const CanConfiguration &rhs) ATLAS_NOEXCEPT {
  baudrate = rhs.baudrate;
  device_id = rhs.device_id;
  unique_id = rhs.unique_id;
  loop_rate = rhs.loop_rate;
  channel = rhs.channel;
  nh_ = rhs.nh_;
}

//------------------------------------------------------------------------------
//
CanConfiguration::~CanConfiguration() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CanConfiguration::DeserializeConfiguration() ATLAS_NOEXCEPT {
  FindParameter("driver/baudrate", baudrate);
  FindParameter("driver/device_id", device_id);
  FindParameter("driver/unique_id", unique_id);
  FindParameter("driver/loop_rate", loop_rate);
  FindParameter("driver/channel", channel);
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
void CanConfiguration::FindParameter(const std::string &str,
                                     Tp_ &p) ATLAS_NOEXCEPT {
  if (nh_->hasParam(str)) {
    nh_->getParam("provider_can/" + str, p);
  } else {
    ROS_WARN_STREAM("Did not find provider_can/" << str << ". Using default value instead.");
  }
}

}  // namespace provider_can
