/**
 * \file	thruster.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	21/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_THRUSTER_H_
#define PROVIDER_CAN_THRUSTER_H_

#include <ros/ros.h>
#include <sonia_msgs/ThrusterMsg.h>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>
#include "provider_can/can_def.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

class Thruster : public CanDevice {
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Thruster>;
  using ConstPtr = std::shared_ptr<const Thruster>;
  using PtrList = std::vector<Thruster::Ptr>;
  using ConstPtrList = std::vector<Thruster::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S
 public:
  explicit Thruster(const CanDispatcher::Ptr &can_dispatcher,
                    const ros::NodeHandlePtr &nh, std::string thruster_name,
                    Actuators unique_id) ATLAS_NOEXCEPT;

  virtual ~Thruster();

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
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  // Receivable CAN messages
  static const uint16_t THRUSTER_STATE_MSG;
  // transmittable CAN messages
  static const uint16_t SET_SPEED_MSG;

  //============================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Set motor speed vector(between -100 and 100, in %)
   *
   * \param speed motor speed
   */
  void SetSpeed(int8_t speed) const ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  const static std::string NAME;
  std::string thruster_specific_name_;

  ros::Publisher thruster_pub_;

  sonia_msgs::ThrusterMsg ros_msg;

  bool properties_sent_;
};

} /* namespace provider_can */

#endif /* PROVIDER_CAN_THRUSTER_H_ */
