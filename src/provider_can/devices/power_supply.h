/**
 * \file	power_supply.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	16/02/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_POWER_SUPPLY_H_
#define PROVIDER_CAN_POWER_SUPPLY_H_

#include <ros/ros.h>
#include <sonia_msgs/PowerSupplyMsg.h>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>
#include "provider_can/can_def.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

// initialisation parameters
struct InitialPsuParams{
  bool volt_bus1_state;
  bool volt_bus2_state;
  bool pc_state;
  bool motor_bus1_state;
  bool motor_bus2_state;
  bool motor_bus3_state;
  bool dvl_state;
  bool actuator_bus_state;
  bool light_state;
};


class PowerSupply : public CanDevice {
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<PowerSupply>;
  using ConstPtr = std::shared_ptr<const PowerSupply>;
  using PtrList = std::vector<PowerSupply::Ptr>;
  using ConstPtrList = std::vector<PowerSupply::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S
 public:
  explicit PowerSupply(const CanDispatcher::Ptr &can_dispatcher,
                       const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  virtual ~PowerSupply();

 protected:
  //============================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * reimplemented method from CanDevice class
   */
  void ProcessMessages(const std::vector<CanMessage> &from_can_rx_buffer,
                       const std::vector<ComputerMessage> &from_pc_rx_buffer)
      ATLAS_NOEXCEPT override;

 private:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  // Receivable CAN messages
  static const uint16_t KILL_STATE_MSG;
  static const uint16_t VOLT1_MSG;
  static const uint16_t VOLT2_MSG;
  static const uint16_t VOLT3_MSG;
  static const uint16_t CURR1_MSG;
  static const uint16_t CURR2_MSG;
  static const uint16_t CURR3_MSG;
  static const uint16_t STATES1_MSG;
  static const uint16_t STATES2_MSG;
  // transmittable CAN messages
  static const uint16_t PC_RST_MSG;
  static const uint16_t SET_CHANNEL_MSG;
  static const uint16_t REMOTE_KILL_MSG;

  //============================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Resets onboard PC power
   */
  void PcReset() const ATLAS_NOEXCEPT;

  /**
   * Enables or disables remote controlled killswitch
   *
   * \param state state of the remote control (on or off)
   */
  void RemoteKill(uint8_t state) const ATLAS_NOEXCEPT;

  /**
   * Enables the selected power channel
   *
   * \param channel selected channel
   */
  void SetChannel(uint8_t channel) const ATLAS_NOEXCEPT;

  /**
   * Disables the selected power channel
   *
   * \param channel selected channel
   */
  void ClrChannel(uint8_t channel) const ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  const static std::string NAME;

  ros::Publisher power_supply_pub_;

  sonia_msgs::PowerSupplyMsg ros_msg_;
};

} /* namespace provider_can */

#endif /* PROVIDER_CAN_POWER_SUPPLY_H_ */
