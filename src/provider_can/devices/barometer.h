/**
 * \file	barometer.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	21/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_BAROMETER_H_
#define PROVIDER_CAN_BAROMETER_H_

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sonia_msgs/BarometerMsg.h>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>
#include "provider_can/can_def.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

class Barometer : public CanDevice {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Barometer>;
  using ConstPtr = std::shared_ptr<const Barometer>;
  using PtrList = std::vector<Barometer::Ptr>;
  using ConstPtrList = std::vector<Barometer::ConstPtr>;

  // Receivable CAN messages
  static const uint16_t INTERNAL_PRESS_MSG;
  static const uint16_t RELATIVE_PRESS_MSG;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Barometer(const CanDispatcher::Ptr &can_dispatcher,
                     const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  virtual ~Barometer();

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
  // P R I V A T E   M E M B E R S

  const static std::string NAME;

  uint32_t seq_id_;
  timespec actual_time_;

  ros::Publisher barometer_intern_press_pub_;
  ros::Publisher barometer_fluid_press_pub_;

  sonia_msgs::BarometerMsg intern_press_msg_;
  sensor_msgs::FluidPressure fluid_press_msg_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_BAROMETER_H_
