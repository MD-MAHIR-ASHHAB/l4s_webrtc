/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_CONGESTION_CONTROLLER_GOOG_CC_GOOG_CC_NETWORK_CONTROL_H_
#define MODULES_CONGESTION_CONTROLLER_GOOG_CC_GOOG_CC_NETWORK_CONTROL_H_

#include <stdint.h>

#include <deque>
#include <memory>
#include <optional>
#include <vector>
#include <fstream>

#include "api/environment/environment.h"
#include "api/network_state_predictor.h"
#include "api/transport/network_control.h"
#include "api/transport/network_types.h"
#include "api/units/data_rate.h"
#include "api/units/data_size.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/congestion_controller/goog_cc/acknowledged_bitrate_estimator_interface.h"
#include "modules/congestion_controller/goog_cc/alr_detector.h"
#include "modules/congestion_controller/goog_cc/congestion_window_pushback_controller.h"
#include "modules/congestion_controller/goog_cc/delay_based_bwe.h"
#include "modules/congestion_controller/goog_cc/loss_based_bwe_v2.h"
#include "modules/congestion_controller/goog_cc/probe_bitrate_estimator.h"
#include "modules/congestion_controller/goog_cc/probe_controller.h"
#include "modules/congestion_controller/goog_cc/send_side_bandwidth_estimation.h"
#include "rtc_base/experiments/field_trial_parser.h"
#include "rtc_base/experiments/rate_control_settings.h"
#include "modules/congestion_controller/goog_cc/ecn_based_bwe.h"
#include "api/numerics/samples_stats_counter.h"
#include "api/test/metrics/global_metrics_logger_and_exporter.h"
#include "api/test/metrics/metrics_logger.h"


namespace webrtc {
struct GoogCcConfig {
  std::unique_ptr<NetworkStateEstimator> network_state_estimator = nullptr;
  std::unique_ptr<NetworkStatePredictor> network_state_predictor = nullptr;
    // Metrics collection configuration
  bool enable_metrics_collection = true;
  std::string test_case_name = "gcc_vs_l4s_comparison";
};
namespace test {
class MetricsLogger;
}

// GCC Metrics Collector for comprehensive performance analysis
class GCCMetricsCollector {
 public:
  GCCMetricsCollector(test::MetricsLogger* logger, 
                      const std::string& test_case_name,
                      Clock* clock);

    ~GCCMetricsCollector();
  
  // Time-series metrics logging
  void LogBandwidthMetrics(Timestamp at_time,
                           DataRate target_bitrate,
                           DataRate actual_bitrate,
                           std::optional<DataRate> acked_bitrate,
                           std::optional<DataRate> send_rate);
  void LogDelayMetrics(Timestamp at_time, TimeDelta rtt, TimeDelta one_way_delay, 
                      TimeDelta jitter = TimeDelta::Zero());
   void LogLossMetrics(Timestamp at_time, double loss_fraction, int packets_lost);
  

  // Periodic summary metrics
  void LogPeriodicSummary(Timestamp at_time);
  
  // Utility methods for stats tracking
  void UpdateAckedRateStats(DataRate acked_rate);
  void UpdateDelayStats(TimeDelta rtt, TimeDelta one_way_delay);
  void UpdateLossStats(double loss_fraction);
  void ExportToJsonFile(const std::string& filename);

  
 private:
  test::MetricsLogger* logger_;
  std::string test_case_name_;
  Clock* clock_;
  
  // Statistics tracking
  SamplesStatsCounter acked_rate_stats_;
  SamplesStatsCounter rtt_stats_;
  SamplesStatsCounter delay_stats_;
  SamplesStatsCounter loss_stats_;
  
  // Last logged values to prevent spam
  Timestamp last_acked_rate_log_ = Timestamp::MinusInfinity();
  Timestamp last_delay_log_ = Timestamp::MinusInfinity();
  Timestamp last_loss_log_ = Timestamp::MinusInfinity();
  Timestamp last_summary_log_ = Timestamp::MinusInfinity();


  
  // Minimum intervals between logs
  static constexpr TimeDelta kAckedRateLogInterval = TimeDelta::Millis(50);
  static constexpr TimeDelta kDelayLogInterval = TimeDelta::Millis(50);
  static constexpr TimeDelta kLossLogInterval = TimeDelta::Millis(50);
  static constexpr TimeDelta kSummaryLogInterval = TimeDelta::Millis(1000);


  int export_call_count_ = 0; // For tracking how many times we've exported to JSON
};



class GoogCcNetworkController : public NetworkControllerInterface {
 public:
  GoogCcNetworkController(NetworkControllerConfig config,
                          GoogCcConfig goog_cc_config,
                          test::MetricsLogger* metrics_logger = nullptr);

  GoogCcNetworkController() = delete;
  GoogCcNetworkController(const GoogCcNetworkController&) = delete;
  GoogCcNetworkController& operator=(const GoogCcNetworkController&) = delete;

  ~GoogCcNetworkController() override;

  // NetworkControllerInterface
  NetworkControlUpdate OnNetworkAvailability(NetworkAvailability msg) override;
  NetworkControlUpdate OnNetworkRouteChange(NetworkRouteChange msg) override;
  NetworkControlUpdate OnProcessInterval(ProcessInterval msg) override;
  NetworkControlUpdate OnRemoteBitrateReport(RemoteBitrateReport msg) override;
  NetworkControlUpdate OnRoundTripTimeUpdate(RoundTripTimeUpdate msg) override;
  NetworkControlUpdate OnSentPacket(SentPacket msg) override;
  NetworkControlUpdate OnReceivedPacket(ReceivedPacket msg) override;
  NetworkControlUpdate OnStreamsConfig(StreamsConfig msg) override;
  NetworkControlUpdate OnTargetRateConstraints(
      TargetRateConstraints msg) override;
  NetworkControlUpdate OnTransportLossReport(TransportLossReport msg) override;
  NetworkControlUpdate OnTransportPacketsFeedback(
      TransportPacketsFeedback msg) override;
  NetworkControlUpdate OnNetworkStateEstimate(
      NetworkStateEstimate msg) override;

  NetworkControlUpdate GetNetworkState(Timestamp at_time) const;

 private:
     // Add at the top of the class definition (private section)
  std::ofstream googcc_metrics_log_;
  bool googcc_metrics_log_initialized_ = false;

  void LogGoogCcMetrics(Timestamp at_time, webrtc::DataRate target_bitrate, webrtc::TimeDelta rtt);
  
  friend class GoogCcStatePrinter;
  std::vector<ProbeClusterConfig> ResetConstraints(
      TargetRateConstraints new_constraints);
  void ClampConstraints();
  void MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update,
                                    Timestamp at_time);
  void UpdateCongestionWindowSize();
  PacerConfig GetPacingRates(Timestamp at_time) const;
  void SetNetworkStateEstimate(std::optional<NetworkStateEstimate> estimate);

  const Environment env_;
  const bool packet_feedback_only_;
  FieldTrialFlag safe_reset_on_route_change_;
  FieldTrialFlag safe_reset_acknowledged_rate_;
  const bool use_min_allocatable_as_lower_bound_;
  const bool ignore_probes_lower_than_network_estimate_;
  const bool limit_probes_lower_than_throughput_estimate_;
  const RateControlSettings rate_control_settings_;
  const bool limit_pacingfactor_by_upper_link_capacity_estimate_;

  const std::unique_ptr<ProbeController> probe_controller_;
  const std::unique_ptr<CongestionWindowPushbackController>
      congestion_window_pushback_controller_;

  std::unique_ptr<SendSideBandwidthEstimation> bandwidth_estimation_;
  std::unique_ptr<AlrDetector> alr_detector_;
  std::unique_ptr<EcnBasedBwe> ecn_based_bwe_; 
  std::unique_ptr<ProbeBitrateEstimator> probe_bitrate_estimator_;
  std::unique_ptr<NetworkStateEstimator> network_estimator_;
  std::unique_ptr<NetworkStatePredictor> network_state_predictor_;
  std::unique_ptr<DelayBasedBwe> delay_based_bwe_;
  std::unique_ptr<AcknowledgedBitrateEstimatorInterface>
      acknowledged_bitrate_estimator_;
  std::optional<NetworkControllerConfig> initial_config_;

  DataRate min_target_rate_ = DataRate::Zero();
  DataRate min_data_rate_ = DataRate::Zero();
  DataRate max_data_rate_ = DataRate::PlusInfinity();
  std::optional<DataRate> starting_rate_;

  bool first_packet_sent_ = false;

  std::optional<NetworkStateEstimate> estimate_;

  Timestamp next_loss_update_ = Timestamp::MinusInfinity();
  int lost_packets_since_last_loss_update_ = 0;
  int expected_packets_since_last_loss_update_ = 0;

  std::deque<int64_t> feedback_max_rtts_;

  DataRate last_loss_based_target_rate_;
  DataRate last_pushback_target_rate_;
  DataRate last_stable_target_rate_;
  LossBasedState last_loss_base_state_;

  std::optional<uint8_t> last_estimated_fraction_loss_ = 0;
  TimeDelta last_estimated_round_trip_time_ = TimeDelta::PlusInfinity();

  double pacing_factor_;
  DataRate min_total_allocated_bitrate_;
  DataRate max_padding_rate_;

  bool previously_in_alr_ = false;

  std::optional<DataSize> current_data_window_;


  
  
  // Bandwidth estimation tracking
  DataRate last_target_rate_ = DataRate::Zero();
  DataRate last_acknowledged_rate_ = DataRate::Zero();
  DataRate last_delay_based_estimate_ = DataRate::Zero();
  
  std::deque<std::pair<Timestamp, int64_t>> send_rate_window_;
  webrtc::DataRate last_send_rate_ = webrtc::DataRate::Zero();
  
  // Metrics collection
  std::unique_ptr<GCCMetricsCollector> metrics_collector_;
  bool metrics_enabled_ = true;
  Timestamp metrics_last_logged_ = Timestamp::MinusInfinity();
  static constexpr TimeDelta kMetricsLoggingInterval = TimeDelta::Millis(50);
  
  std::deque<std::pair<Timestamp, int>> throughput_window_;

  // Performance tracking for metrics
  DataRate last_actual_bitrate_ = DataRate::Zero();
  DataRate last_target_bitrate_ = DataRate::Zero();
  TimeDelta last_rtt_ = TimeDelta::PlusInfinity();
  TimeDelta jitter_ = TimeDelta::Zero();
  double rfc3550_jitter_ = 0.0;
  double last_loss_fraction_ = 0.0;
  int last_packets_lost_ = 0;
  std::string current_active_controller_ = "initializing";

  // Helper methods for metrics
  void LogPeriodicMetrics(Timestamp at_time);

};

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_GOOG_CC_GOOG_CC_NETWORK_CONTROL_H_
