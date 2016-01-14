/**
 * \file	main.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	30/11/2015
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
#include "can_dispatcher.h"
#include "bottom_light.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "provider_can");

  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  auto can_ptr = std::make_shared<provider_can::CanDispatcher>
  	  	  	  	  	  (controllers,on_board_pc,0, BAUD_125K, 10);

  provider_can::BottomLight bottom_light(can_ptr);

  uint8_t test = 2;

  while (ros::ok()) {

    can_ptr->MainCanProcess();
    can_ptr->PushUnicastMessage(1, 1, 0xF00, &test, 1);

    bottom_light.Process();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return (0);
}
