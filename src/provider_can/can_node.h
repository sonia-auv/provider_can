/**
 * \file	can_node.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	26/01/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_CAN_NODE_H_
#define PROVIDER_CAN_CAN_NODE_H_

#include <memory>
#include <vector>
#include <cstring>
#include <iostream>
#include <ros/node_handle.h>
#include <lib_atlas/ros/service_server_manager.h>
#include "provider_can/can/can_def.h"
#include "provider_can/can/can_dispatcher.h"
#include "provider_can/devices/can_device.h"
#include "provider_can/devices/bottom_light.h"

namespace provider_can {

/**
 * This class contains the main process for can_provider
 */
class CanNode {
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CanNode>;

 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  /**
   * Initialize all new devices objects  in this constructor
   */
  CanNode(std::shared_ptr<ros::NodeHandle> nh);

  ~CanNode();

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * Call this process periodically in the main. Its function is to call
   * all devices processes for messages processing
   */
  void ProcessMessages(void);

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  CanDispatcher::Ptr can_ptr_;

  std::vector<CanDevice::Ptr> can_devices_vector_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_NODE_H_
