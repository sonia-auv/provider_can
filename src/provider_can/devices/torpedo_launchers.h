/**
 * \file	torpedo_launchers.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	11/03/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_TORPEDO_LAUNCHERS_H_
#define PROVIDER_CAN_TORPEDO_LAUNCHERS_H_

#include <memory>
#include <vector>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "sonia_msgs/TorpedoLaunchersMsg.h"
#include "provider_can/can_def.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

class TorpedoLaunchers : public CanDevice {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<TorpedoLaunchers>;
  using ConstPtr = std::shared_ptr<const TorpedoLaunchers>;
  using PtrList = std::vector<TorpedoLaunchers::Ptr>;
  using ConstPtrList = std::vector<TorpedoLaunchers::ConstPtr>;

  // Receivable CAN messages
  static const uint16_t PRESS_MSG;

  // Transmittable CAN messages
  static const uint16_t STARBOARD_TARGET;
  static const uint16_t LAUNCH_MSG;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit TorpedoLaunchers(const CanDispatcher::Ptr &can_dispatcher,
                            const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  virtual ~TorpedoLaunchers();

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

  void Launch() const ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  const static std::string NAME;

  ros::Publisher torpedo_launchers_pub_;

  sonia_msgs::TorpedoLaunchersMsg ros_msg_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_TORPEDO_LAUNCHERS_H_
