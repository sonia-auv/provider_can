/**
 * \file	led_indicator.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	29/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_LED_INDICATOR_H_
#define PROVIDER_CAN_LED_INDICATOR_H_

#include <memory>
#include <vector>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "provider_can/can_def.h"
#include "provider_can/can/can_dispatcher.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

class LedIndicator : public CanDevice {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<LedIndicator>;
  using ConstPtr = std::shared_ptr<const LedIndicator>;
  using PtrList = std::vector<LedIndicator::Ptr>;
  using ConstPtrList = std::vector<LedIndicator::ConstPtr>;

  // transmittable CAN messages
  static const uint16_t SET_MODE_MSG;
  static const uint16_t SET_COLOR_MSG;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit LedIndicator(const CanDispatcher::Ptr &can_dispatcher,
                        const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  virtual ~LedIndicator();

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
   * Sets the display color of the led indicator
   *
   * \param color color to set (0 to 7).
   * table:{BLACK,RED,YELLOW,CYAN,GREEN,WHITE}
   */
  void SetColor(uint8_t color) ATLAS_NOEXCEPT;

  /**
   * Sets the mode of operation of the led indicator
   *
   * \param mode mode to set (0 to 2). table:{OFF, BLINK, ON, RAINBOW}
   */
  void SetMode(uint8_t mode) ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  const static std::string NAME;

  uint8_t color_;
  uint8_t mode_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_LED_INDICATOR_H_
