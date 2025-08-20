#ifndef MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_
#define MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_

#include <deque>
#include <memory>
#include <optional>
#include <string>

#include "api/environment/environment.h"
#include "api/network_state_predictor.h"
#include "api/rtc_event_log/rtc_event_log.h"
#include "api/transport/network_control.h"
#include "api/transport/network_types.h"
#include "api/units/data_rate.h"
#include "api/units/data_size.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/congestion_controller/goog_cc/acknowledged_bitrate_estimator.h"
#include "modules/congestion_controller/goog_cc/alr_detector.h"
#include "modules/congestion_controller/goog_cc/delay_based_bwe.h"
#include "modules/congestion_controller/goog_cc/probe_controller.h"
#include "api/numerics/samples_stats_counter.h"
#include "system_wrappers/include/clock.h"

// Metrics collection
#include "api/test/metrics/metrics_logger.h"

namespace webrtc {

// Forward declarations
class RtcEventLog;

// Configuration for L4S network controller
struct L4SControllerConfig {
  // ECN Configuration
  bool use_ect1_marking = true;
  bool fallback_to_gcc = true;
  
  // Metrics and Testing
  bool enable_metrics_collection = true;
  std::string test_case_name = "l4s_network_test";
  
  // Bandwidth Estimation Components
  bool enable_probing = true;
  bool enable_delay_estimation = true;
  bool enable_acked_estimation = true;
  bool enable_alr_detection = true;
  TimeDelta probe_interval = TimeDelta::Seconds(5);
  
  // Confidence Thresholds
  double ecn_confidence_threshold = 0.8;
  double probe_confidence_threshold = 0.7;
  double delay_confidence_threshold = 0.6;
  double acked_confidence_threshold = 0.5;
};

// Prague DCTCP-style capacity estimator with ECN feedback
class PragueCapacityEstimator {
public:
  static constexpr TimeDelta kDecayInterval = TimeDelta::Seconds(30);
  static constexpr size_t kHistoryWindowSize = 100;

  PragueCapacityEstimator(DataRate starting_rate, DataRate min_rate, DataRate max_rate);
  ~PragueCapacityEstimator();

  // Prague DCTCP algorithm implementation
  void UpdateFromCongestionSignal(DataRate current_rate, double ce_ratio, Timestamp current_time);
  void UpdateEcnActivity(Timestamp current_time);  // Track any ECN activity (ECT or CE)
  void UpdateFromRtt(TimeDelta rtt);
  void OnPacketLoss(DataRate current_rate, Timestamp current_time);
  void OnTimeUpdate(Timestamp current_time);

  DataRate GetCurrentEstimate() const;
  double GetAlpha() const { return alpha_; }
  int GetDirectionFlag() const { return direction_flag_; }
  int GetNonCePacketCount() const { return non_ce_packet_count_; }
  double GetConfidence(Timestamp now) const;

private:
  DataRate congestion_based_estimate_;
  DataRate min_target_rate_;
  DataRate max_target_rate_;
  TimeDelta current_rtt_;
  double alpha_ = 0.0;  // DCTCP alpha parameter
  Timestamp last_update_time_;
  Timestamp last_congestion_signal_;
  Timestamp last_ecn_feedback_;  // Track any ECN activity (ECT or CE)
  
  // State machine for direction control
  int direction_flag_ = 1;  // 1 = increasing, -1 = reducing
  int non_ce_packet_count_ = 0;  // Count of consecutive non-CE packets
  static constexpr int kNonCeThreshold = 7;  // Threshold to switch to additive mode
};

// Bandwidth source fusion engine
class L4SBandwidthFusion {
public:
  struct BandwidthSources {
    DataRate ecn_estimate = DataRate::Zero();
    DataRate delay_estimate = DataRate::Zero();
    DataRate probe_estimate = DataRate::Zero();
    DataRate acked_estimate = DataRate::Zero();
    DataRate alr_estimate = DataRate::Zero();
    
    double ecn_confidence = 0.0;
    double delay_confidence = 0.0;
    double probe_confidence = 0.0;
    double acked_confidence = 0.0;
    double alr_confidence = 0.0;
    
    Timestamp last_ecn_update = Timestamp::MinusInfinity();
    Timestamp last_delay_update = Timestamp::MinusInfinity();
    Timestamp last_probe_update = Timestamp::MinusInfinity();
    Timestamp last_acked_update = Timestamp::MinusInfinity();
    Timestamp last_alr_update = Timestamp::MinusInfinity();
  };

  explicit L4SBandwidthFusion(const L4SControllerConfig& config);
  ~L4SBandwidthFusion();

  void UpdateEcnEstimate(DataRate estimate, double confidence, Timestamp now);
  void UpdateDelayEstimate(DataRate estimate, double confidence, Timestamp now);
  void UpdateProbeEstimate(DataRate estimate, double confidence, Timestamp now);
  void UpdateAckedEstimate(DataRate estimate, double confidence, Timestamp now);
  void UpdateAlrEstimate(DataRate estimate, double confidence, Timestamp now);

  DataRate GetFusedEstimate(Timestamp now) const;
  BandwidthSources GetCurrentSources() const { return sources_; }

private:
  DataRate GetMostConfidentEstimate(Timestamp now) const;
  DataRate ValidateWithOtherSources(DataRate primary_estimate, const BandwidthSources& sources) const;
  bool IsRecentlyUpdated(Timestamp last_update, Timestamp now) const;

  BandwidthSources sources_;
  L4SControllerConfig config_;
};

// L4S Metrics Collector
class L4SMetricsCollector {
public:
  static constexpr TimeDelta kBandwidthLogInterval = TimeDelta::Millis(500);
  static constexpr TimeDelta kDelayLogInterval = TimeDelta::Millis(100);
  static constexpr TimeDelta kLossLogInterval = TimeDelta::Millis(1000);
  static constexpr TimeDelta kSummaryLogInterval = TimeDelta::Seconds(10);

  L4SMetricsCollector(test::MetricsLogger* logger, 
                           const std::string& test_case_name,
                           Clock* clock);
  ~L4SMetricsCollector();

  void LogBandwidthMetrics(Timestamp at_time, DataRate target_bitrate, DataRate actual_bitrate);
  void LogDelayMetrics(Timestamp at_time, TimeDelta rtt, TimeDelta one_way_delay, TimeDelta jitter);
  void LogLossMetrics(Timestamp at_time, double loss_fraction, int packets_lost);
  void LogCongestionMetrics(Timestamp at_time, int ce_count, int ect_count, double congestion_ratio);
  void LogFusionMetrics(Timestamp at_time, const L4SBandwidthFusion::BandwidthSources& sources, DataRate fused_rate);
  void LogPeriodicSummary(Timestamp at_time);

  void ExportToJsonFile(const std::string& filename);

private:
  void UpdateThroughputStats(DataRate actual_bitrate);
  void UpdateDelayStats(TimeDelta rtt);
  void UpdateLossStats(double loss_fraction);

  test::MetricsLogger* logger_;
  std::string test_case_name_;
  Clock* clock_;

  // Logging rate limiting
  Timestamp last_bandwidth_log_ = Timestamp::MinusInfinity();
  Timestamp last_delay_log_ = Timestamp::MinusInfinity();
  Timestamp last_loss_log_ = Timestamp::MinusInfinity();
  Timestamp last_summary_log_ = Timestamp::MinusInfinity();

  // Statistics tracking
  webrtc::SamplesStatsCounter throughput_stats_;
  webrtc::SamplesStatsCounter delay_stats_;
  webrtc::SamplesStatsCounter loss_stats_;
};

// Main L4S Prague Network Controller
class L4SNetworkController : public NetworkControllerInterface {
public:
  L4SNetworkController(NetworkControllerConfig config,
                      L4SControllerConfig l4s_config,
                      test::MetricsLogger* metrics_logger = nullptr);
  ~L4SNetworkController() override;

  // NetworkControllerInterface implementation
  NetworkControlUpdate OnNetworkAvailability(NetworkAvailability msg) override;
  NetworkControlUpdate OnNetworkRouteChange(NetworkRouteChange msg) override;
  NetworkControlUpdate OnProcessInterval(ProcessInterval msg) override;
  NetworkControlUpdate OnRemoteBitrateReport(RemoteBitrateReport msg) override;
  NetworkControlUpdate OnRoundTripTimeUpdate(RoundTripTimeUpdate msg) override;
  NetworkControlUpdate OnSentPacket(SentPacket msg) override;
  NetworkControlUpdate OnReceivedPacket(ReceivedPacket msg) override;
  NetworkControlUpdate OnStreamsConfig(StreamsConfig msg) override;
  NetworkControlUpdate OnTargetRateConstraints(TargetRateConstraints msg) override;
  NetworkControlUpdate OnTransportLossReport(TransportLossReport msg) override;
  NetworkControlUpdate OnTransportPacketsFeedback(TransportPacketsFeedback msg) override;
  NetworkControlUpdate OnNetworkStateEstimate(NetworkStateEstimate msg) override;

private:
  // Initialization
  void InitializeBandwidthEstimators();

  // Core processing methods
  void UpdateAllBandwidthEstimators(const TransportPacketsFeedback& feedback);
  void ProcessEcnFeedback(const TransportPacketsFeedback& feedback, DataRate current_fused_rate);
  DataRate DetermineBottleneckAwareTarget(DataRate fused_rate, Timestamp now);
  void UpdateDelayBasedEstimator(const TransportPacketsFeedback& feedback);
  void UpdateAckedBitrateEstimator(const TransportPacketsFeedback& feedback);
  void ProcessProbeResults(const TransportPacketsFeedback& feedback);
  
  // Bandwidth fusion methods
  DataRate GetBaseFusedEstimate(Timestamp now);

  // Probing logic
  void HandlePeriodicProbing(Timestamp now, NetworkControlUpdate* update);
  bool ShouldProbeNow(Timestamp now) const;
  void InitiateProbing(Timestamp now, NetworkControlUpdate* update);

  // ALR detection
  bool IsApplicationLimited() const;
  void UpdateAlrDetector(const TransportPacketsFeedback& feedback);

  // Confidence calculation
  double CalculateEcnConfidence(Timestamp now) const;
  double CalculateDelayConfidence(Timestamp now) const;
  double CalculateProbeConfidence(Timestamp now) const;
  double CalculateAckedConfidence(Timestamp now) const;

  // Rate control
  DataRate FuseBandwidthEstimates(Timestamp now);
  NetworkControlUpdate CreateRateUpdate(Timestamp at_time) const;
  void MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update, Timestamp at_time);

  // State management
  bool IsL4SActive() const;
  bool HasRecentCongestionSignals(Timestamp now) const;
  bool IsEcnFeedbackFresh(Timestamp now) const;
  bool EstimatesAreDiverging() const;
  bool IsRttStable() const;

  // Throughput calculation
  void UpdateThroughputWindow(const TransportPacketsFeedback& feedback);

  // Metrics
  void LogPeriodicMetrics(Timestamp at_time);

  // Environment and configuration
  const Environment env_;
  L4SControllerConfig config_;

  // Bandwidth estimation components
  std::unique_ptr<PragueCapacityEstimator> prague_estimator_;
  std::unique_ptr<DelayBasedBwe> delay_estimator_;
  std::unique_ptr<ProbeController> probe_controller_;
  std::unique_ptr<AcknowledgedBitrateEstimator> acked_estimator_;
  std::unique_ptr<AlrDetector> alr_detector_;
  std::unique_ptr<L4SBandwidthFusion> bandwidth_fusion_;

  // State tracking
  std::optional<DataRate> target_rate_;
  std::optional<DataRate> starting_rate_;
  std::optional<DataRate> min_target_rate_;
  std::optional<DataRate> max_target_rate_;

  // ECN state
  bool ecn_supported_ = false;
  bool ecn_capable_network_ = false;
  int ect_count_ = 0;
  int ce_count_ = 0;
  Timestamp last_congestion_signal_ = Timestamp::MinusInfinity();

  // RTT tracking
  TimeDelta last_rtt_ = TimeDelta::PlusInfinity();
  TimeDelta last_estimated_round_trip_time_ = TimeDelta::Millis(50);

  // Loss tracking
  double last_loss_fraction_ = 0.0;
  int last_packets_lost_ = 0;

  // Probing state
  Timestamp last_probe_time_ = Timestamp::MinusInfinity();
  DataRate last_probe_estimate_ = DataRate::Zero();

  // Throughput calculation
  std::deque<std::pair<Timestamp, int64_t>> throughput_window_;
  DataRate last_actual_bitrate_ = DataRate::Zero();

  // Metrics
  bool metrics_enabled_ = true;
  std::unique_ptr<L4SMetricsCollector> metrics_collector_;
  Timestamp metrics_last_logged_ = Timestamp::MinusInfinity();
  static constexpr TimeDelta kMetricsLoggingInterval = TimeDelta::Millis(500);
};

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_
