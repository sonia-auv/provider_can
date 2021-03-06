/**
 * \file	hydrophones.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	09/03/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_CAN_HYDROPHONES_H_
#define PROVIDER_CAN_HYDROPHONES_H_

#include <ros/ros.h>
#include <sonia_msgs/HydrophonesMsg.h>
#include <sonia_msgs/HydrophonesParams.h>
#include <memory>
#include <vector>
#include "provider_can/can_def.h"
#include "provider_can/devices/can_device.h"

namespace provider_can {

// initialisation parameters
struct InitialHydrosParams {
  int32_t hydro_enable;
  int32_t wave_enable;
  int32_t pinger_freq;
  int32_t gain;
  int32_t no_param;
  int32_t hydros_2012_acq_threshold;
  int32_t filter_threshold;
  int32_t continuous_filter_freq;
  int32_t sample_count;
  int32_t hydros_2012_acq_threshold_mode;
  int32_t hydros_2012_phase_calc_algo;
  int32_t set_cutoff_freq;
  int32_t set_preamp_gain;
  int32_t hydros_2012_fft_enable;
  int32_t fft_threshold;
  int32_t fft_prefilter;
  int32_t fft_prefilter_type;
  int32_t fft_bandwidth;
  int32_t hydros_2012_trig_mode_param;
};

class Hydrophones : public CanDevice {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Hydrophones>;
  using ConstPtr = std::shared_ptr<const Hydrophones>;
  using PtrList = std::vector<Hydrophones::Ptr>;
  using ConstPtrList = std::vector<Hydrophones::ConstPtr>;

  // settable parameters (using SetParam())
  static const uint32_t HYDRO_ENABLE_PARAM;
  static const uint32_t WAVE_ENABLE_PARAM;
  static const uint32_t PINGER_FREQ_PARAM;
  static const uint32_t GAIN_PARAM;
  static const uint32_t NO_PARAM;
  static const uint32_t HYDROS_2012_ACQ_THRESHOLD;
  static const uint32_t FILTER_THRESHOLD_PARAM;
  static const uint32_t CONTINUOUS_FILTER_FREQ_PARAM;
  static const uint32_t SAMPLE_COUNT_PARAM;
  static const uint32_t HYDROS_2012_ACQ_THRESHOLD_MODE;
  static const uint32_t HYDROS_2012_PHASE_CALC_ALGO;
  static const uint32_t SET_FREQ_CUTOFF_PARAM;
  static const uint32_t SET_PREAMP_GAIN_PARAM;
  static const uint32_t HYDROS_2012_FFT_ENABLE;
  static const uint32_t FFT_THRESHOLD_PARAM;
  static const uint32_t FFT_PREFILTER_PARAM;
  static const uint32_t FFT_PREFILTER_TYPE_PARAM;
  static const uint32_t FFT_BANDWIDTH_PARAM;
  static const uint32_t HYDROS_2012_TRIG_MODE_PARAM;

  static const uint32_t PARAM_TYPES_TABLE[19];

  // Receivable CAN messages
  static const uint16_t SCOPE_MSG;
  static const uint16_t DEPHASAGE_MSG;
  static const uint16_t DEPHASAGE2_MSG;
  static const uint16_t FREQ_MSG;
  static const uint16_t GET_PARAM_RESPONSE_MSG;
  static const uint16_t SET_PARAM_RESPONSE_MSG;
  static const uint16_t FFT_MAGNITUDE_MSG;

  // Transmittable CAN messages
  static const uint16_t GET_PARAM_MSG;
  static const uint16_t SET_PARAM_MSG;
  static const uint16_t SEND_DATA_REQUEST_MSG;

  static const uint16_t MAX_SCOPE_SAMPLES;
  static const uint16_t MAX_MAGNITUDE_SAMPLES;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Hydrophones(const CanDispatcher::Ptr &can_dispatcher,
                       const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  explicit Hydrophones(const CanDispatcher::Ptr &can_dispatcher,
                       const ros::NodeHandlePtr &nh,
                       const InitialHydrosParams params) ATLAS_NOEXCEPT;

  virtual ~Hydrophones();

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
  //============================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Sends a request to the hydrophones. Hydros will answer with a table of
   * scope and magnitude samples collected
   */
  void SendDataReq() const ATLAS_NOEXCEPT;

  /**
   * Asks the hydros to sends their config parameters
   */
  void GetParams(bool reset) ATLAS_NOEXCEPT;

  /**
   * Change a config parameter of the hydrophones
   *
   * \param param the parameter number (defined as methods in can_def)
   * \param value the value of the config parameter
   */
  void SetParam(HydrophonesMethods param, int32_t value) const ATLAS_NOEXCEPT;

  /**
   * processing different complicated messages and sending ros msg with data
   *
   * \param can_message can message containing specific message
   */
  bool ProcessMagnitudeMsgs(const CanMessage &can_message) ATLAS_NOEXCEPT;
  void ProcessParamsMsgs(const CanMessage &can_message) ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  const static std::string NAME;

  bool get_params_sent_;

  uint32_t scope_samples_count_;

  ros::Publisher hydro_pub_;
  ros::Publisher hydro_params_pub_;

  sonia_msgs::HydrophonesMsg ros_msg_;
  sonia_msgs::HydrophonesParams ros_param_msg_;

  uint32_t get_params_index_;
};

}  // namespace provider_can

#endif  // PROVIDER_CAN_HYDROPHONES_H_
