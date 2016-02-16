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

#include <memory>
#include <vector>
#include <cstring>
#include <iostream>
#include "provider_can/can/can_def.h"
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

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * reimplemented method from CanDevice class
   */
  void Process();

  /**
   * unique light device functions
   */
  void SetLevel(uint8_t level);

  uint8_t GetLevel() const;

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  const static uint16_t SET_LIGHT_MSG;
  const static uint16_t SET_LIGHT_DLC;
  const static std::string NAME;

  uint8_t actual_light_level_;  // Light actual state
  uint8_t asked_light_level_;   // set by setLightLevel()

  ros::Publisher bottom_light_pub_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DRIVER_H_
