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

#include <memory>
#include <vector>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "provider_can/can/can_def.h"
#include "provider_can/can/can_dispatcher.h"
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

    //============================================================================
    // P U B L I C   M E T H O D S

    /**
     * reimplemented method from CanDevice class
     */
    void Process() ATLAS_NOEXCEPT override;

  private:

    //============================================================================
    // P R I V A T E   M E M B E R S

    const static std::string NAME;

    bool properties_sent_;

    ros::Publisher barometer_pub_;
    ros::Publisher barometer_properties_pub_;
  };

}  // namespace provider_can

#endif  // PROVIDER_CAN_BAROMETER_H_
