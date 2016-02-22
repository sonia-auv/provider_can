/**
 * \file	can_def.h
 * \author	Alexi Demers
 * \date	04/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_CAN_DEF_H_
#define PROVIDER_CAN_CAN_DEF_H_

//============================================================================
// D E F I N E S

// Devices capabilities
const uint8_t ISP = 0x01;
const uint8_t RESET = 0x02;
const uint8_t SLEEP = 0x04;

// Messages types
const uint32_t UNICAST = 0x10000000;
const uint8_t RESET_REQ = 0xfe;
const uint8_t WAKEUP_REQ = 0xf1;
const uint8_t SLEEP_REQ = 0xf0;
const uint8_t DEVICE_FAULT = 0xff;
const uint8_t PING = 0x01;
const uint8_t SET_PARAM_REQ = 0x21;
const uint8_t GET_PARAM_REQ = 0x20;

// Address parameters positions
const uint8_t UNIQUE_ID_POSITION = 12;
const uint8_t DEVICE_ID_POSITION = 20;

// Mac address mask
const uint32_t DEVICE_MAC_MASK = 0x0FFFF000;

// Message type mask
const uint32_t DEVICE_MSG_MASK = 0x00000FFF;

// Messages DLCs
const uint8_t IDENTIFY_REPLY_DLC = 7;
const uint8_t DEVICE_FAULT_DLC = 8;
const uint8_t PING_REQUEST_DLC = 0;
const uint8_t PING_REPLY_DLC = 0;
const uint8_t GET_PARAMETER_DLC = 1;
const uint8_t SET_PARAMETER_DLC = 5;
const uint8_t SLEEP_REQUEST_DLC = 0;
const uint8_t WAKEUP_REQUEST_DLC = 0;
const uint8_t RESET_REQUEST_DLC = 0;

enum MessageType { global = 0x0, application = 0xf };

//============================================================================
// D E V I C E S   P A R A M E T E R S

// These are the parameter numbers for each device. It may be passed to
// setParameterReq function or it is used to select device_parameters table
// index for each param.
enum MotorsParams {

};

enum DepthMeterParams { voltage_offset_abs = 0, voltage_offset_rel };

enum DroppersParams { droppers_drop_time = 0 };

enum GrabberParams {

};

enum LedIndicatorParams {

};

enum LightParams {

};

enum SonarRotatorParams {

};

enum TorpedoLauncherParams { torpedo_launch_time = 0, torpedo_max_pressure };

//============================================================================
// D E V I C E   I D

enum DeviceClass {
  controllers = 1,
  actuators,
  markers,
  sonars,
  sensors,
  power,
  interfaces,
  lights,
  can2rs232,
};

//============================================================================
// D E V I C E S  U N I Q U E   I D

enum Controllers { on_board_pc = 1, pid_controller };

enum Actuators {
  port_motor = 1,
  starboard_motor,
  front_depth_motor,
  back_depth_motor,
  front_heading_motor,
  back_heading_motor,
  grabber,
  sonar_rotator
};

enum Markers { dropper = 1, launcher };

enum Sonars { passive = 1, active };

enum Sensors { depth_meter = 1, light_sensor, leak_sensor };

enum Power {
  power_distribution = 2,
};

enum Interfaces {
  mission = 1,
  diver_interface,
  led_indicator,
  carte_navigation_exception
};

enum Lights { front_light = 1, bottom_light = 5 };

enum Can2Rs232 { micron_dst_modem = 1 };

//============================================================================
// D E V I C E S   M E T H O D S   N U M B E R S

enum CommonMethods {
  ping_req = 100,
  presence_check = 101,
  get_properties = 102,
};

enum BotLightMethods { set_level = 0 };

enum PSUMethods {
  pc_reset = 0,
  remote_kill = 1,
  set_channel = 2,
  clr_channel = 3
};

enum ThrustersMethods {
  set_speed = 0,
};
#endif
