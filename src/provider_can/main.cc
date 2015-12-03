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

#include <ros/ros.h>
#include "can_driver.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "provider_can");

  ros::NodeHandle nh;
  printf("yes1");

  provider_can::CanDriver can(0, provider_can::SONIA_CAN_BAUD_100K);

  provider_can::CanMessage msg;
 // can.readMessage(&msg, 0);

  msg.data[0] = 0x33;
  msg.dlc = 1;
  msg.flag = 1;
  msg.id = 0x01;

  can.printErrorText(can.writeMessage(&msg, 0));

  printf("yes2");

  ros::spin();

  return (0);
}
