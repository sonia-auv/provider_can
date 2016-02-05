/**
 * \file	main.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	26/01/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <memory>
#include <vector>
#include <ros/ros.h>
#include "provider_can/can_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "provider_can");

  auto nh = std::make_shared<ros::NodeHandle>();
  auto can_node_ptr = std::make_shared<provider_can::CanNode>(nh);

  ros::Rate loop_rate(20);

  while (ros::ok()) {
    can_node_ptr->ProcessMessages();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
