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
  ros::init(argc, argv, "provider_can");

  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  provider_can::CanDispatcher canD(0, BAUD_125K);

  while (ros::ok())
  {
    canD.providerCanProcess();

    ros::spinOnce();
    loop_rate.sleep();
  }
/*
  provider_can::CanDriver can(0, BAUD_125K);
  provider_can::CanMessage msg;
  provider_can::CanMessage *msgrd;
  canStatus status;
  uint32_t messages_read;

  msg.data[0] = 0x04;
  msg.data[1] = 0x01;

  msg.dlc = 2;
  msg.flag = canMSG_EXT;
  msg.id = 0x10701F08;

  can.writeMessage(msg, 10);

  while (ros::ok())
  {

    status = can.readAllMessages(msgrd, &messages_read);
    can.printErrorText(status);

    if(messages_read!=0){
      printf("%X       ", msgrd[0].id);
      printf("%X       ", msgrd[1].id);
      printf("%X       ", msgrd[2].id);

      printf("%d       \n\r", messages_read);
    }


    ros::spinOnce();
    loop_rate.sleep();
  }
*/


  return (0);
}
