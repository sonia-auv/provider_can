/**
 * \file	diver_interface.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	04/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_DIVER_INTERFACE_H_
#define PROVIDER_CAN_DIVER_INTERFACE_H_

#include <memory>
#include <vector>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "sonia_msgs/MissionSwitchMsg.h"
#include "provider_can/can_def.h"
#include "provider_can/can/can_dispatcher.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

class DiverInterface : public CanDevice {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DiverInterface>;
  using ConstPtr = std::shared_ptr<const DiverInterface>;
  using PtrList = std::vector<DiverInterface::Ptr>;
  using ConstPtrList = std::vector<DiverInterface::ConstPtr>;

  // transmittable CAN messages
  static const uint16_t SET_STATE_MSG;
  static const uint16_t SET_MISSION_MSG;

  // receivable can messages
  static const uint16_t MISSION_SWITCH_STATE;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit DiverInterface(const CanDispatcher::Ptr &can_dispatcher,
                          const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  virtual ~DiverInterface();

 protected:
  //============================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * reimplemented method from CanDevice class
   */
  void ProcessMessages(const std::vector<CanMessage> &rx_buffer,
                       const std::vector<ComputerMessage> &pc_messages_buffer)
      ATLAS_NOEXCEPT override;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  /**
   * These functions are used to send new string to display on the diver
   * interface.
   *
   * \param string string to display
   */
  void SendStateString(std::string string) const ATLAS_NOEXCEPT;
  void SendMissionString(std::string string) const ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  const static std::string NAME;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_DIVER_INTERFACE_H_
