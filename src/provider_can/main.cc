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
#include <ros/ros.h>
#include "provider_can/can_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "provider_can");

  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
  provider_can::CanNode can_node_ptr(nh);
  can_node_ptr.Start();

  int8_t test = -8;
  printf("%d",(uint8_t)test);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
