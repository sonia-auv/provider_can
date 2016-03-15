/**
 * \file	droppers.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	11/03/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_DROPPERS_H_
#define PROVIDER_CAN_DROPPERS_H_

#include <ros/ros.h>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>
#include "provider_can/can_def.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

class Droppers : public CanDevice {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Droppers>;
  using ConstPtr = std::shared_ptr<const Droppers>;
  using PtrList = std::vector<Droppers::Ptr>;
  using ConstPtrList = std::vector<Droppers::ConstPtr>;

  // Transmittable CAN messages
  static const uint16_t DROP_MSG;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Droppers(const CanDispatcher::Ptr &can_dispatcher,
                    const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  virtual ~Droppers();

 protected:
  //============================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * reimplemented method from CanDevice class
   */
  void ProcessMessages(const std::vector<CanMessage> &from_can_rx_buffer,
                       const std::vector<ComputerMessage> &from_pc_rx_buffer)
      ATLAS_NOEXCEPT override;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  void Drop(uint8_t dropper_to_drop) const ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  const static std::string NAME;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_DROPPERS_H_
