/**
 * \file	main.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	26/01/2016
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <memory>
#include <ros/ros.h>
#include "can_node.h"
#include <memory>
#include <vector>

int main(int argc, char** argv) {

  ros::init(argc, argv, "provider_can");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);

  auto can_node_ptr = std::make_shared<provider_can::CanNode>();

  while (ros::ok()) {
    can_node_ptr->ProcessMessages();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return (0);
}
