/**
 * \file	hydrophones.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	10/03/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "hydrophones.h"
#include <provider_can/can/can_dispatcher.h>
#include <provider_can/can/can_driver.h>

namespace provider_can {

//==============================================================================
// S T A T I C   M E M B E R S

const std::string Hydrophones::NAME = "hydrophones";
const uint32_t Hydrophones::HYDRO_ENABLE_PARAM = 10;
const uint32_t Hydrophones::WAVE_ENABLE_PARAM = 20;
const uint32_t Hydrophones::PINGER_FREQ_PARAM = 30;
const uint32_t Hydrophones::GAIN_PARAM = 40;
const uint32_t Hydrophones::NO_PARAM = 50;
const uint32_t Hydrophones::HYDROS_2012_ACQ_THRESHOLD = 60;
const uint32_t Hydrophones::FILTER_THRESHOLD_PARAM = 70;
const uint32_t Hydrophones::CONTINUOUS_FILTER_FREQ_PARAM = 80;
const uint32_t Hydrophones::SAMPLE_COUNT_PARAM = 90;
const uint32_t Hydrophones::HYDROS_2012_ACQ_THRESHOLD_MODE = 100;
const uint32_t Hydrophones::HYDROS_2012_PHASE_CALC_ALGO = 110;
const uint32_t Hydrophones::SET_FREQ_CUTOFF_PARAM = 120;
const uint32_t Hydrophones::SET_PREAMP_GAIN_PARAM = 130;
const uint32_t Hydrophones::HYDROS_2012_FFT_ENABLE = 140;
const uint32_t Hydrophones::FFT_THRESHOLD_PARAM = 150;
const uint32_t Hydrophones::FFT_PREFILTER_PARAM = 160;
const uint32_t Hydrophones::FFT_PREFILTER_TYPE_PARAM = 170;
const uint32_t Hydrophones::FFT_BANDWIDTH_PARAM = 180;
const uint32_t Hydrophones::HYDROS_2012_TRIG_MODE_PARAM = 190;

const uint32_t Hydrophones::PARAM_TYPES_TABLE[19] = {
    HYDRO_ENABLE_PARAM, WAVE_ENABLE_PARAM, PINGER_FREQ_PARAM, GAIN_PARAM,
    NO_PARAM, HYDROS_2012_ACQ_THRESHOLD, FILTER_THRESHOLD_PARAM, CONTINUOUS_FILTER_FREQ_PARAM,
    SAMPLE_COUNT_PARAM, HYDROS_2012_ACQ_THRESHOLD_MODE, HYDROS_2012_PHASE_CALC_ALGO, SET_FREQ_CUTOFF_PARAM,
    SET_PREAMP_GAIN_PARAM, HYDROS_2012_FFT_ENABLE, FFT_THRESHOLD_PARAM, FFT_PREFILTER_PARAM,
    FFT_PREFILTER_TYPE_PARAM, FFT_BANDWIDTH_PARAM,HYDROS_2012_TRIG_MODE_PARAM};

// Receivable CAN messages
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
const uint16_t Hydrophones::MAX_MAGNITUDE_SAMPLES = 128;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
Hydrophones::Hydrophones(const CanDispatcher::Ptr &can_dispatcher,
                         const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : CanDevice(sonars, hydrophones, can_dispatcher, NAME, nh),
      get_params_sent_(false),
      scope_samples_count_(MAX_SCOPE_SAMPLES),
      get_params_index_(0) {
  hydro_pub_ = nh->advertise<sonia_msgs::HydrophonesMsg>(NAME + "_msgs", 10);

  hydro_params_pub_ =
      nh->advertise<sonia_msgs::HydrophonesParams>(NAME + "_params", 10);
}

//------------------------------------------------------------------------------
//
Hydrophones::Hydrophones(const CanDispatcher::Ptr &can_dispatcher,
                         const ros::NodeHandlePtr &nh,
                         const InitialHydrosParams params) ATLAS_NOEXCEPT
    : CanDevice(sonars, hydrophones, can_dispatcher, NAME, nh),
      get_params_sent_(false),
      scope_samples_count_(MAX_SCOPE_SAMPLES) {
  hydro_pub_ = nh->advertise<sonia_msgs::HydrophonesMsg>(NAME + "_msgs", 10);

  hydro_params_pub_ =
      nh->advertise<sonia_msgs::HydrophonesParams>(NAME + "_params", 10);

  // used to go through the struct using index
  int32_t *struct_indexing = (int32_t *)&params;

  // In "sizeof(PARAM_TYPES_TABLE)-2", the "-1" is used because there is one
  // dummy parameter in hydrophones. The PARAM_TYPES_TABLE is forced
  // to initialize it because it exists, but we cant set its value.
  for (uint16_t i = 0; i < (sizeof(PARAM_TYPES_TABLE) / sizeof(uint32_t)); i++)
    SetParam((HydrophonesMethods)i, struct_indexing[i]);
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

  ros_msg_.hydro_freq_updated = (uint8_t) false;
  ros_msg_.magn_samples_updated = (uint8_t) false;
  ros_msg_.dephasage1_updated = (uint8_t) false;
  ros_msg_.dephasage2_updated = (uint8_t) false;
  ros_msg_.scope_samples_updated = (uint8_t) false;

  uint32_t conversion_var;
  float *conversion_var_float;
  int16_t *conversion_var_int16;
  uint16_t deph_raw1, deph_raw2, deph_raw3;
  // if messages have been received
  // loops through all messages received
  for (auto &can_message : from_can_rx_buffer) {
    switch (can_message.id) {
      case FFT_MAGNITUDE_MSG:
        message_rcvd = ProcessMagnitudeMsgs(can_message);
        break;

      case SET_PARAM_RESPONSE_MSG:
      case GET_PARAM_RESPONSE_MSG:
        ProcessParamsMsgs(can_message);
        break;

      case DEPHASAGE_MSG:
        deph_raw1 = can_message.data[0] + (can_message.data[1] << 8);
        conversion_var_int16 = (int16_t*)&deph_raw1;
        ros_msg_.dephasage1_d1 = *conversion_var_int16;

        deph_raw2 = can_message.data[2] + (can_message.data[3] << 8);
        conversion_var_int16 = (int16_t*)&deph_raw2;
        ros_msg_.dephasage1_d2 = *conversion_var_int16;

        deph_raw3 = can_message.data[4] + (can_message.data[5] << 8);
        conversion_var_int16 = (int16_t*)&deph_raw3;
        ros_msg_.dephasage1_d3 = *conversion_var_int16;
        ros_msg_.dephasage1_pinger_freq = can_message.data[6] + (can_message.data[7] << 8);
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

        conversion_var = can_message.data[1] + (can_message.data[2] << 8) +
                         (can_message.data[3] << 16) +
                         (can_message.data[4] << 24);
        conversion_var_float = (float *)&conversion_var;
        ros_msg_.amplitude = *conversion_var_float;

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
        get_params_sent_ = true;
        GetParams(true);
        break;
      case send_data_req:
        SendDataReq();
        break;
      default:
        if (pc_message.method_number < 100)
          SetParam((HydrophonesMethods)pc_message.method_number,
                   (int32_t)pc_message.parameter_value);
        break;
    }
  }

  if (get_params_sent_) GetParams(false);

  if (message_rcvd) hydro_pub_.publish(ros_msg_);

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

void Hydrophones::GetParams(bool reset) ATLAS_NOEXCEPT {
  static uint32_t local_index = 0;
  static bool index_0_done = false;

  // this allows the user to resend a get param if last call failed
  if (reset) {
    local_index = 0;
    index_0_done = false;
  }

  // the "/ 10" is because indexes are coded in steps of 10
  // in the hydrophones board.
  if (!index_0_done) {
    // sends the first get params on can bus
    index_0_done = true;
    uint8_t *msg = (uint8_t *)&(PARAM_TYPES_TABLE[0]);
    PushMessage(GET_PARAM_MSG, msg, 4);
    // "-1" is because indexes starts at 10 instead of 0
  } else if ((get_params_index_ / 10) - 1 == local_index) {
    // if last get param has been received, sends the next one
    local_index++;
    uint8_t *msg = (uint8_t *)&(PARAM_TYPES_TABLE[local_index]);
    PushMessage(GET_PARAM_MSG, msg, 4);
    if (local_index == (sizeof(PARAM_TYPES_TABLE) / sizeof(uint32_t)) - 1) {
      // if we reached the last param to send, stops sending get params
      local_index = 0;
      index_0_done = false;
      get_params_sent_ = false;
    }
  }
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

bool Hydrophones::ProcessMagnitudeMsgs(const CanMessage &can_message)
    ATLAS_NOEXCEPT {
  uint32_t index;

  // magnitude msg will be sent MAX_SAMPLES times by hydrophones. once all
  // samples are
  // received, this method will send a ROS msg.
  index = can_message.data[0] - 1;
  uint32_t conversion_var;
  float *conversion_var_float;

  // there is only 4 values per sample collected. Index should not be
  // higher than 3
  if (index < MAX_MAGNITUDE_SAMPLES) {
    conversion_var = (can_message.data[1] << 24) + (can_message.data[2] << 16) +
                     (can_message.data[3] << 8) + (can_message.data[4]);

    conversion_var_float = (float *)&conversion_var;
    ros_msg_.magnitude_values[index] = *conversion_var_float;
  }

  if (index == (MAX_MAGNITUDE_SAMPLES - 1)) {
    ros_msg_.magn_samples_updated = (uint8_t) true;
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//

void Hydrophones::ProcessParamsMsgs(const CanMessage &can_message)
    ATLAS_NOEXCEPT {
  get_params_index_ = can_message.data[0] + (can_message.data[1] << 8) +
                      (can_message.data[2] << 16) + (can_message.data[3] << 24);

  // When dlc == 8, it correspond to a successfull param read
  if (can_message.dlc == 8) {
    // Index are coded by steps of 10. see constants under header.
    // verifying the index not to step out of the buffer
    if (get_params_index_ <= HYDROS_2012_TRIG_MODE_PARAM) {
      int32_t data = can_message.data[4] + (can_message.data[5] << 8) +
                     (can_message.data[6] << 16) + (can_message.data[7] << 24);

      switch (get_params_index_) {
        case HYDRO_ENABLE_PARAM:
          ros_param_msg_.hydro_enable = data;
          break;
        case WAVE_ENABLE_PARAM:
          ros_param_msg_.wave_enable = data;
          break;
        case PINGER_FREQ_PARAM:
          ros_param_msg_.pinger_freq = data;
          break;
        case GAIN_PARAM:
          ros_param_msg_.gain = data;
          break;
        case FILTER_THRESHOLD_PARAM:
          ros_param_msg_.filter_threshold = data;
          break;
        case CONTINUOUS_FILTER_FREQ_PARAM:
          ros_param_msg_.continuous_filter_freq = data;
          break;
        case SAMPLE_COUNT_PARAM:
          ros_param_msg_.sample_count = data;
          break;
        case SET_FREQ_CUTOFF_PARAM:
          ros_param_msg_.set_cutoff_freq = data;
          break;
        case SET_PREAMP_GAIN_PARAM:
          ros_param_msg_.set_preamp_gain = data;
          break;
        case FFT_THRESHOLD_PARAM:
          ros_param_msg_.fft_threshold = data;
          break;
        case FFT_PREFILTER_PARAM:
          ros_param_msg_.fft_prefilter = data;
          break;
        case FFT_PREFILTER_TYPE_PARAM:
          ros_param_msg_.fft_prefilter_type = data;
          break;
        case FFT_BANDWIDTH_PARAM:
          ros_param_msg_.fft_bandwidth = data;
          break;
        case HYDROS_2012_ACQ_THRESHOLD_MODE:
          ros_param_msg_.acq_thrs_mode = data;
          break;
        case HYDROS_2012_PHASE_CALC_ALGO:
          ros_param_msg_.phase_calc_alg = data;
          break;
        case HYDROS_2012_TRIG_MODE_PARAM:
          ros_param_msg_.fft_trig_mode_Param = data;
          break;
        case HYDROS_2012_FFT_ENABLE:
          ros_param_msg_.fft_enable = data;
          break;
        case HYDROS_2012_ACQ_THRESHOLD:
          ros_param_msg_.acq_threshold = data;
          break;
      }
    }

    // avoids sending parameters 19 times after GetParams() is called
    if (get_params_sent_) {
      if ((get_params_index_ / 10) ==
          (sizeof(PARAM_TYPES_TABLE) / sizeof(uint32_t))-1) {
        hydro_params_pub_.publish(ros_param_msg_);
      }
    } else {
      hydro_params_pub_.publish(ros_param_msg_);
    }

    // if DLC != 8, unsuccessfull param read
  } else {
    ROS_WARN("Hydrophones: parameter address %d does not exist",
             get_params_index_);
  }
}

//------------------------------------------------------------------------------
//

}  // namespace provider_can
