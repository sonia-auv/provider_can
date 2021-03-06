/**
 * \file	bottom_light.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	04/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_BOTTOM_LIGHT_H_
#define PROVIDER_CAN_BOTTOM_LIGHT_H_

#include <ros/ros.h>
#include <sonia_msgs/BottomLightMsg.h>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>
#include "provider_can/can_def.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

class BottomLight : public CanDevice {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BottomLight>;
  using ConstPtr = std::shared_ptr<const BottomLight>;
  using PtrList = std::vector<BottomLight::Ptr>;
  using ConstPtrList = std::vector<BottomLight::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit BottomLight(const CanDispatcher::Ptr &can_dispatcher,
                       const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  virtual ~BottomLight();

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

  /**
   * unique light device functions
   */
  void SetLevel(uint8_t level) ATLAS_NOEXCEPT;

  uint8_t GetLevel() const ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  const static uint16_t SET_LIGHT_MSG;
  const static uint8_t SET_LIGHT_DLC;
  const static std::string NAME;

  uint8_t actual_light_level_;  // Light actual state
  uint8_t asked_light_level_;   // set by setLightLevel()

  ros::Publisher bottom_light_pub_;

  sonia_msgs::BottomLightMsg ros_msg_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_BOTTOM_LIGHT_H_
