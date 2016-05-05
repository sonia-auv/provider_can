/**
 * \file	can_configuration.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	16/02/2016
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

#ifndef PROVIDER_CAN_DEVICES_CONFIGURATION_H_
#define PROVIDER_CAN_DEVICES_CONFIGURATION_H_

#include <lib_atlas/macros.h>
#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include "provider_can/devices/hydrophones.h"
#include "provider_can/devices/power_supply.h"

namespace provider_can {

class DevicesConfiguration {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DevicesConfiguration>;
  using ConstPtr = std::shared_ptr<const DevicesConfiguration>;
  using PtrList = std::vector<DevicesConfiguration::Ptr>;
  using ConstPtrList = std::vector<DevicesConfiguration::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  DevicesConfiguration(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  virtual ~DevicesConfiguration() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E M B E R S

  bool barometer_en_, bottom_light_en_, diver_interface_en_, droppers_en_,
      grabber_en_, hydrophones_en_, led_indicator_en_, mission_switch_en_,
      power_supply_en_, thruster_starboard_en_, thruster_port_en_,
      thruster_back_depth_en_, torpedo_launcher_en_, thruster_front_depth_en_,
      thruster_back_head_en_, thruster_front_head_en_;

  provider_can::InitialHydrosParams hydros_params_;
  provider_can::InitialPsuParams psu_params_;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  void DeserializeConfiguration() ATLAS_NOEXCEPT;

  template <typename Tp_>
  void FindParameter(const std::string &str, Tp_ &p) ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_DEVICES_CONFIGURATION_H_
