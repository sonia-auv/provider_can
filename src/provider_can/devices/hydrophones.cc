/**
 * \file	hydrophones.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	10/03/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <provider_can/can/can_dispatcher.h>
#include "hydrophones.h"

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const std::string Hydrophones::NAME = "hydrophones";
const uint32_t Hydrophones::HYDRO_ENABLE_PARAM = 10;
const uint32_t Hydrophones::WAVE_ENABLE_PARAM = 20;
const uint32_t Hydrophones::PINGER_FREQ_PARAM = 30;
const uint32_t Hydrophones::GAIN_PARAM = 40;
const uint32_t Hydrophones::ACQ_THRESHOLD_PARAM = 60;
const uint32_t Hydrophones::FILTER_THRESHOLD_PARAM = 70;
const uint32_t Hydrophones::CONTINUOUS_FILTER_FREQ_PARAM = 80;
const uint32_t Hydrophones::SAMPLE_COUNT_PARAM = 90;
const uint32_t Hydrophones::ACQ_THRS_MODE_PARAM = 100;
const uint32_t Hydrophones::PHASE_CALC_ALG_PARAM = 110;
const uint32_t Hydrophones::SET_FREQ_CUTOFF_PARAM = 120;
const uint32_t Hydrophones::SET_PREAMP_GAIN_PARAM = 130;
const uint32_t Hydrophones::FFT_ENABLE_PARAM = 140;
const uint32_t Hydrophones::FFT_THRESHOLD_PARAM = 150;
const uint32_t Hydrophones::FFT_PREFILTER_PARAM = 160;
const uint32_t Hydrophones::FFT_PREFILTER_TYPE_PARAM = 170;
const uint32_t Hydrophones::FFT_BANDWIDTH_PARAM = 180;
const uint32_t Hydrophones::FFT_TRIG_MODE_PARAM = 190;

const uint32_t Hydrophones::PARAM_TYPES_TABLE[18] = {
    HYDRO_ENABLE_PARAM, WAVE_ENABLE_PARAM, PINGER_FREQ_PARAM, GAIN_PARAM,
    ACQ_THRESHOLD_PARAM, FILTER_THRESHOLD_PARAM, CONTINUOUS_FILTER_FREQ_PARAM,
    SAMPLE_COUNT_PARAM, ACQ_THRS_MODE_PARAM, PHASE_CALC_ALG_PARAM,
    SET_FREQ_CUTOFF_PARAM, SET_PREAMP_GAIN_PARAM, FFT_ENABLE_PARAM,
    FFT_THRESHOLD_PARAM, FFT_PREFILTER_PARAM, FFT_PREFILTER_TYPE_PARAM,
    FFT_BANDWIDTH_PARAM, FFT_TRIG_MODE_PARAM};

// Receivable CAN messages
const uint16_t Hydrophones::SCOPE_MSG = 0xF02;
const uint16_t Hydrophones::DEPHASAGE_MSG = 0xF03;
const uint16_t Hydrophones::DEPHASAGE2_MSG = 0xF04;
const uint16_t Hydrophones::FREQ_MSG = 0xF05;
const uint16_t Hydrophones::GET_PARAM_RESPONSE_MSG = 0x020;
const uint16_t Hydrophones::SET_PARAM_RESPONSE_MSG = 0x021;
const uint16_t Hydrophones::FFT_MAGNITUDE_MSG = 0xF07;

// Transmittable CAN messages
const uint16_t Hydrophones::GET_PARAM_MSG = 0x020;
const uint16_t Hydrophones::SET_PARAM_MSG = 0x021;
const uint16_t Hydrophones::SEND_DATA_REQUEST_MSG = 0xF06;

const uint16_t Hydrophones::MAX_SCOPE_SAMPLES = 256;
const uint16_t Hydrophones::MAX_MAGNITUDE_SAMPLES = 16;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
Hydrophones::Hydrophones(const CanDispatcher::Ptr &can_dispatcher,
                         const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(sonars, hydrophones, can_dispatcher, NAME, nh),
      scope_samples_count_(MAX_SCOPE_SAMPLES) {
  active_sonar_pub_ =
      nh->advertise<sonia_msgs::HydrophonesMsg>(NAME + "_msgs", 10);
}

//------------------------------------------------------------------------------
//
Hydrophones::~Hydrophones() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void Hydrophones::ProcessMessages(
    const std::vector<CanMessage> &from_can_rx_buffer,
    const std::vector<ComputerMessage> &from_pc_rx_buffer) ATLAS_NOEXCEPT {
  bool message_rcvd = false;
  bool clr_scope_samples = false;
  uint32_t index;
  uint16_t address;

  ros_msg_.hydro_freq_updated = (uint8_t) false;
  ros_msg_.magn_samples_updated = (uint8_t) false;
  ros_msg_.dephasage1_updated = (uint8_t) false;
  ros_msg_.dephasage2_updated = (uint8_t) false;
  ros_msg_.params_updated = (uint8_t) false;
  ros_msg_.scope_samples_updated = (uint8_t) false;

  // if messages have been received
  // loops through all barometer messages received
  for (auto &can_message : from_can_rx_buffer) {
    switch (can_message.id & DEVICE_MSG_MASK) {
      case SCOPE_MSG:

        // Scope msg will be sent MAX_SAMPLES times by hydrophones. once all
        // samples are
        // received, this method will send a ROS msg.
        address = (can_message.data[0] << 8) + can_message.data[1];
        index = can_message.data[3] - 1;

        // resize the table for the number of samples to be received
        if (ros_msg_.scope_values.size() != scope_samples_count_) {
          ros_msg_.scope_values.resize(scope_samples_count_);
        }

        // there is only 4 values per sample collected. Index should not be
        // higher than 3
        if (index < 4 && address < scope_samples_count_) {
          ros_msg_.scope_values[address].samples[index] =
              can_message.data[4] + (can_message.data[5] << 8) +
              (can_message.data[6] << 16) + (can_message.data[7] << 24);
        }

        if (address == (scope_samples_count_ - 1)) clr_scope_samples = true;
        message_rcvd = true;
        ros_msg_.scope_samples_updated = (uint8_t) true;
        break;

      case FFT_MAGNITUDE_MSG:
        // magnitude msg will be sent MAX_SAMPLES times by hydrophones. once all
        // samples are
        // received, this method will send a ROS msg.
        index = can_message.data[0] - 1;

        // there is only 4 values per sample collected. Index should not be
        // higher than 3
        if (index < MAX_MAGNITUDE_SAMPLES) {
          ros_msg_.magnitude_values[index] =
              can_message.data[1] + (can_message.data[2] << 8) +
              (can_message.data[3] << 16) + (can_message.data[4] << 24);
        }

        if (index == (MAX_MAGNITUDE_SAMPLES - 1)) {
          ros_msg_.magn_samples_updated = (uint8_t) true;
          message_rcvd = true;
        }
        break;

      case SET_PARAM_RESPONSE_MSG:
      case GET_PARAM_RESPONSE_MSG:
        if (can_message.dlc == 8) {
          index = can_message.data[0] + (can_message.data[1] << 8) +
                  (can_message.data[2] << 16) + (can_message.data[3] << 24);

          // Index are coded by steps of 10. see constants under header
          if (((index / 10) - 1) < sizeof(PARAM_TYPES_TABLE)) {
            ros_msg_.parameters_values[index / 10 - 1] =
                can_message.data[4] + (can_message.data[5] << 8) +
                (can_message.data[6] << 16) + (can_message.data[7] << 24);
          }
          message_rcvd = true;
          ros_msg_.params_updated = (uint8_t) true;
        } else {
          ROS_WARN("Hydrophones: parameter address %d does not exist",
                   (can_message.data[0] + (can_message.data[1] << 8) +
                    (can_message.data[2] << 16) + (can_message.data[3] << 24)));
        }

        break;

      case DEPHASAGE_MSG:
        ros_msg_.dephasage1_d1 =
            can_message.data[0] + (can_message.data[1] << 8);
        ros_msg_.dephasage1_d2 =
            can_message.data[2] + (can_message.data[3] << 8);
        ros_msg_.dephasage1_d3 =
            can_message.data[4] + (can_message.data[5] << 8);
        ros_msg_.dephasage1_pinger_freq =
            can_message.data[6] + (can_message.data[7] << 8);
        ros_msg_.dephasage1_updated = (uint8_t) true;
        message_rcvd = true;
        break;
      case DEPHASAGE2_MSG:
        ros_msg_.dephasage2_d1 =
            can_message.data[0] + (can_message.data[1] << 8);
        ros_msg_.dephasage2_d2 =
            can_message.data[2] + (can_message.data[3] << 8);
        ros_msg_.dephasage2_d3 =
            can_message.data[4] + (can_message.data[5] << 8);
        ros_msg_.dephasage2_updated = (uint8_t) true;
        message_rcvd = true;
        break;

      case FREQ_MSG:
        ros_msg_.hydro_freq_index = can_message.data[0];
        ros_msg_.frequency = can_message.data[1] + (can_message.data[2] << 8) +
                             (can_message.data[3] << 16) +
                             (can_message.data[4] << 24);
        message_rcvd = true;
        ros_msg_.hydro_freq_updated = (uint8_t) true;
        break;
      default:
        break;
    }
  }

  // loops through all PC messages received
  for (auto &pc_message : from_pc_rx_buffer) {
    switch (pc_message.method_number) {
      case get_params:
        GetParams();
        break;
      case send_data_req:
        SendDataReq();
        break;
      default:
        SetParam((HydrophonesMethods)pc_message.method_number,
                 (int32_t)pc_message.parameter_value);
        break;
    }
  }

  if (message_rcvd) active_sonar_pub_.publish(ros_msg_);

  if (clr_scope_samples) {
    ros_msg_.scope_values.clear();
  }
}

//------------------------------------------------------------------------------
//

void Hydrophones::SendDataReq() const ATLAS_NOEXCEPT {
  uint8_t *msg = nullptr;
  PushMessage(SEND_DATA_REQUEST_MSG, msg, 0);
}

//------------------------------------------------------------------------------
//

void Hydrophones::GetParams() const ATLAS_NOEXCEPT {
  uint8_t *msg = nullptr;
  PushMessage(GET_PARAM_MSG, msg, 0);
}

//------------------------------------------------------------------------------
//

void Hydrophones::SetParam(HydrophonesMethods param,
                           int32_t value) const ATLAS_NOEXCEPT {
  // if param number exists
  if (param <= (sizeof(PARAM_TYPES_TABLE) / sizeof(uint32_t))) {
    uint64_t concat_message =
        ((uint64_t)value << 32) + PARAM_TYPES_TABLE[param];
    uint8_t *msg = (uint8_t *)&(concat_message);
    PushMessage(SET_PARAM_MSG, msg, 8);
  } else {
    ROS_WARN("Hydrophones: method number %d does not exist", (uint32_t)param);
  }
}

//------------------------------------------------------------------------------
//

}  // namespace provider_can
