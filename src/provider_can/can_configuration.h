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

#ifndef PROVIDER_CAN_CAN_CONFIGURATION_H_
#define PROVIDER_CAN_CAN_CONFIGURATION_H_

#include <memory>
#include <vector>
#include <ros/ros.h>
#include <lib_atlas/macros.h>

namespace provider_can {

class CanConfiguration {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CanConfiguration>;
  using ConstPtr = std::shared_ptr<const CanConfiguration>;
  using PtrList = std::vector<CanConfiguration::Ptr>;
  using ConstPtrList = std::vector<CanConfiguration::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  CanConfiguration(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  /**
   * We want to define copy constructor here for optimization purpose.
   * Do not copy the members if is a rvalue.
   */
  CanConfiguration(const CanConfiguration &rhs) ATLAS_NOEXCEPT;

  virtual ~CanConfiguration() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E M B E R S

  int baudrate;
  int device_id;
  int unique_id;
  int channel;

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

#endif  // PROVIDER_CAN_CAN_CONFIGURATION_H_
