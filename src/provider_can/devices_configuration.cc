/**
 * \file	ekf_configuration.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	07/02/2016
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
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

#include "provider_can/devices_configuration.h"

namespace provider_can {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
DevicesConfiguration::DevicesConfiguration(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : barometer_en_(true),bottom_light_en_(true), diver_interface_en_(true),
      droppers_en_(true), grabber_en_(true), hydrophones_en_(true),
      led_indicator_en_(true), mission_switch_en_(true), power_supply_en_(true),
      thruster_starboard_en_(true), thruster_port_en_(true),
      thruster_back_depth_en_(true), torpedo_launcher_en_(true),
      thruster_front_depth_en_(true), thruster_back_head_en_(true),
      thruster_front_head_en_(true),
      nh_(nh) {
}

//------------------------------------------------------------------------------
//
DevicesConfiguration::~DevicesConfiguration() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void DevicesConfiguration::DeserializeConfiguration() ATLAS_NOEXCEPT {

  FindParameter("barometer/enable", barometer_en_);
  FindParameter("bottom_light/enable", bottom_light_en_);
  FindParameter("diver_interface/enable", diver_interface_en_);
  FindParameter("droppers/enable", droppers_en_);
  FindParameter("grabber/enable", grabber_en_);
  FindParameter("hydrophones/enable", hydrophones_en_);
  FindParameter("led_indicator/enable", led_indicator_en_);
  FindParameter("mission_switch/enable", mission_switch_en_);
  FindParameter("power_supply/enable", power_supply_en_);
  FindParameter("thruster_starboard/enable", thruster_starboard_en_);
  FindParameter("thruster_port/enable", thruster_port_en_);
  FindParameter("thruster_back_depth/enable", thruster_back_depth_en_);
  FindParameter("torpedo_launcher/enable", torpedo_launcher_en_);
  FindParameter("thruster_front_depth/enable", thruster_front_depth_en_);
  FindParameter("thruster_back_head/enable", thruster_back_head_en_);
  FindParameter("thruster_front_head/enable", thruster_front_head_en_);

  FindParameter("hydrophones/acq_threshold", hydros_params_.acq_threshold);
  FindParameter("hydrophones/acq_thrs_mode", hydros_params_.acq_thrs_mode);
  FindParameter("hydrophones/continuous_filter_freq", hydros_params_.continuous_filter_freq);
  FindParameter("hydrophones/fft_bandwidth", hydros_params_.fft_bandwidth);
  FindParameter("hydrophones/fft_enable", hydros_params_.fft_enable);
  FindParameter("hydrophones/fft_prefilter", hydros_params_.fft_prefilter);
  FindParameter("hydrophones/fft_prefilter_type", hydros_params_.fft_prefilter_type);
  FindParameter("hydrophones/fft_threshold", hydros_params_.fft_threshold);
  FindParameter("hydrophones/fft_trig_mode_Param", hydros_params_.fft_trig_mode_Param);
  FindParameter("hydrophones/hydro_enable", hydros_params_.hydro_enable);
  FindParameter("hydrophones/phase_calc_alg", hydros_params_.phase_calc_alg);
  FindParameter("hydrophones/pinger_freq", hydros_params_.pinger_freq);
  FindParameter("hydrophones/sample_count", hydros_params_.sample_count);
  FindParameter("hydrophones/set_cutoff_freq", hydros_params_.set_cutoff_freq);
  FindParameter("hydrophones/set_preamp_gain", hydros_params_.set_preamp_gain);
  FindParameter("hydrophones/wave_enable", hydros_params_.wave_enable);
  FindParameter("hydrophones/filter_threshold", hydros_params_.filter_threshold);
  FindParameter("hydrophones/filter_gain", hydros_params_.gain);

  FindParameter("power_supply/actuator_bus_state", psu_params_.actuator_bus_state);
  FindParameter("power_supply/dvl_state", psu_params_.dvl_state);
  FindParameter("power_supply/light_state", psu_params_.light_state);
  FindParameter("power_supply/motor_bus1_state", psu_params_.motor_bus1_state);
  FindParameter("power_supply/motor_bus2_state", psu_params_.motor_bus2_state);
  FindParameter("power_supply/motor_bus3_state", psu_params_.motor_bus3_state);
  FindParameter("power_supply/pc_state", psu_params_.pc_state);
  FindParameter("power_supply/volt_bus1_state", psu_params_.volt_bus1_state);
  FindParameter("power_supply/motor_bus3_state", psu_params_.volt_bus2_state);

}

//------------------------------------------------------------------------------
//
template <typename Tp_>
void DevicesConfiguration::FindParameter(const std::string &str,
                                     Tp_ &p) ATLAS_NOEXCEPT {
  if (nh_->hasParam(str)) {
    nh_->getParam("provider_can/" + str, p);
  } else {
    ROS_WARN_STREAM("Did not find provider_can/"
                    << str << ". Using default value instead.");
  }
}

}  // namespace provider_can
