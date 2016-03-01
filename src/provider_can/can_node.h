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
#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/runnable.h>
#include <lib_atlas/ros/service_server_manager.h>
#include "provider_can/can_configuration.h"
#include "provider_can/can/can_dispatcher.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

/**
 * This class contains the main process for can_provider. it calls all devices'
 * messages processing.
 */
class CanNode : public atlas::Runnable {
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CanNode>;
  using ConstPtr = std::shared_ptr<const CanNode>;
  using PtrList = std::vector<CanNode::Ptr>;
  using ConstPtrList = std::vector<CanNode::ConstPtr>;

  // thread execution interval
  static const uint32_t THREAD_INTERVAL_US;

 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  /**
   * Initialize all new devices objects  in this constructor
   */
  explicit CanNode(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  ~CanNode() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * Call this process periodically in the main. Its function is to call
   * all devices processes for messages processing
   */
  void Run() ATLAS_NOEXCEPT override;

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;

  CanConfiguration conf_;

  CanDispatcher::Ptr can_ptr_;

  std::vector<CanDevice::Ptr> can_devices_vector_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_NODE_H_
