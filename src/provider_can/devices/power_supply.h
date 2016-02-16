/**
 * \file	power_supply.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	16/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_POWER_SUPPLY_H_
#define PROVIDER_CAN_POWER_SUPPLY_H_

#include <memory>
#include <vector>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "provider_can/can/can_def.h"
#include "provider_can/can/can_dispatcher.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

class PowerSupply : public CanDevice {
	//==========================================================================
	// T Y P E D E F   A N D   E N U M

	using Ptr = std::shared_ptr<PowerSupply>;
	using ConstPtr = std::shared_ptr<const PowerSupply>;
	using PtrList = std::vector<PowerSupply::Ptr>;
	using ConstPtrList = std::vector<PowerSupply::ConstPtr>;

	//============================================================================
	// P U B L I C   C / D T O R S
  public:
	explicit PowerSupply(const CanDispatcher::Ptr &can_dispatcher,
					   const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

	virtual ~PowerSupply();

	//============================================================================
	// P U B L I C   M E T H O D S

	/**
	 * reimplemented method from CanDevice class
	 */
	void Process()ATLAS_NOEXCEPT override;

  private:
	//============================================================================
	// P R I V A T E   M E M B E R S

	const static std::string NAME;

	ros::Publisher power_supply_pub_;
};

}/* namespace provider_can */

#endif /* PROVIDER_CAN_POWER_SUPPLY_H_ */
