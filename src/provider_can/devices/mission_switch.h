/**
 * \file	mission_switch.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	29/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_MISSION_SWITCH_H_
#define PROVIDER_CAN_MISSION_SWITCH_H_

#include <memory>
#include <vector>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "sonia_msgs/MissionSwitchMsg.h"
#include "provider_can/can/can_def.h"
#include "provider_can/can/can_dispatcher.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

  class MissionSwitch : public CanDevice {
  public:
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<MissionSwitch>;
    using ConstPtr = std::shared_ptr<const MissionSwitch>;
    using PtrList = std::vector<MissionSwitch::Ptr>;
    using ConstPtrList = std::vector<MissionSwitch::ConstPtr>;

    // receivable can messages
    static const uint16_t MISSION_SWITCH_STATE;

    //============================================================================
    // P U B L I C   C / D T O R S

    explicit MissionSwitch(const CanDispatcher::Ptr &can_dispatcher,
                         const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

    virtual ~MissionSwitch();

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
    // P R I V A T E   M E M B E R S

    const static std::string NAME;

    ros::Publisher mission_switch_pub_;

    sonia_msgs::MissionSwitchMsg ros_msg_;

  };

}  // namespace provider_can

#endif  // PROVIDER_CAN_MISSION_SWITCH_H_
