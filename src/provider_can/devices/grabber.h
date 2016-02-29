/**
 * \file	barometer.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	21/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_GRABBER_H_
#define PROVIDER_CAN_GRABBER_H_

#include <memory>
#include <vector>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "sonia_msgs/GrabberMsg.h"
#include "provider_can/can/can_def.h"
#include "provider_can/can/can_dispatcher.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

  class Grabber : public CanDevice {
  public:
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<Grabber>;
    using ConstPtr = std::shared_ptr<const Grabber>;
    using PtrList = std::vector<Grabber::Ptr>;
    using ConstPtrList = std::vector<Grabber::ConstPtr>;

    // Receivable CAN messages
    static const uint16_t STATE_MSG;
    static const uint16_t PRESS_MSG;

    // Transmittable CAN messages
    static const uint16_t STARBOARD_TARGET;
    static const uint16_t PORT_TARGET;

    //============================================================================
    // P U B L I C   C / D T O R S

    explicit Grabber(const CanDispatcher::Ptr &can_dispatcher,
                       const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

    virtual ~Grabber();

  protected:
    //============================================================================
    // P R O T E C T E D   M E T H O D S

    /**
     * reimplemented method from CanDevice class
     */
    void ProcessMessages(const std::vector<CanMessage> &rx_buffer,
                         const std::vector<ComputerMessage> &pc_messages_buffer)
    ATLAS_NOEXCEPT override;

    void StarSetTarget(uint8_t target);
    void PortSetTarget(uint8_t target);

  private:
    //============================================================================
    // P R I V A T E   M E M B E R S

    const static std::string NAME;

    ros::Publisher grabber_pub_;

    sonia_msgs::GrabberMsg ros_msg_;
  };

}  // namespace provider_can

#endif  // PROVIDER_CAN_GRABBEr_H_
