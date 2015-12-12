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
//#include "can_driver.h"
#include "can_dispatcher.h"

int main(int argc, char** argv) {
  provider_can::CanMessage* msg;

  uint8_t message[2] = {0x3, 0x1};

  uint8_t num;

  ros::init(argc, argv, "provider_can");

  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  provider_can::CanDispatcher canD(0, BAUD_125K);

  printf("test: %d  ", canD.pushMessage(7, 1, 0xF08, message, 2));

  while (ros::ok()) {
    canD.providerCanProcess();

    canD.fetchMessages(6, 2, msg, &num);
    printf("Device: %X, ndata: %d \n\r", msg[0].id, num);

    canD.fetchMessages(2, 7, msg, &num);
    printf("Device: %X, ndata: %d \n\r", msg[0].id, num);

    canD.fetchMessages(3, 5, msg, &num);
    printf("Device: %X, ndata: %d \n\r", msg[0].id, num);

    canD.fetchMessages(5,3,msg,&num);
    printf("Device: %X, ndata: %d \n\r",msg[0].id, num);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return (0);
}
