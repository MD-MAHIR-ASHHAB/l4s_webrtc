#ifndef MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_
#define MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <cstdint>

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
#include "modules/congestion_controller/goog_cc/probe_controller.h"
#include "modules/congestion_controller/goog_cc/probe_bitrate_estimator.h"
#include "api/numerics/samples_stats_counter.h"

// Metrics collection
#include "api/test/metrics/metrics_logger.h"

namespace webrtc {

namespace test {
class L4SNetworkControllerTest;
}

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
  bool enable_acked_estimation = true;
  bool enable_alr_detection = true;
  TimeDelta probe_interval = TimeDelta::Seconds(8);
  TimeDelta recovery_probe_interval = TimeDelta::Seconds(5);
  double recovery_probe_rtt_factor = 5.0;
  double probe_hold_rtt_factor = 5.0;
  TimeDelta min_rtt_scaled_interval = TimeDelta::Seconds(1);

  // Probe pacing aggressiveness.
  double probe_multiplier = 1.2;
  double alr_probe_multiplier = 1.35;
  double recovery_probe_multiplier = 1.4;
  double recovery_alr_probe_multiplier = 1.7;
  double min_useful_probe_uplift = 1.05;

  // Avoid probing/recovery at very low rates where probe samples are often distorted.
  DataRate periodic_probe_min_rate = DataRate::KilobitsPerSec(400);
  DataRate recovery_probe_min_rate = DataRate::KilobitsPerSec(600);

  // Probe gating and confidence behavior.
  double discovery_probe_block_confidence = 0.92;
  double steady_probe_block_confidence = 0.88;
  double probe_confidence_fresh = 0.75;
  double probe_confidence_recent = 0.55;

  // Recovery-entry stability guards.
  int recovery_min_clean_packets = 40;
  TimeDelta recovery_min_clean_duration = TimeDelta::Seconds(2);
  TimeDelta recovery_reentry_cooldown = TimeDelta::Seconds(2);
  
  // Confidence Thresholds
  double ecn_confidence_threshold = 0.8;
  double probe_confidence_threshold = 0.7;
  double acked_confidence_threshold = 0.5;
};

class PragueCapacityEstimator {
 public:
  PragueCapacityEstimator(DataRate starting_rate, DataRate min_rate, DataRate max_rate);
  ~PragueCapacityEstimator();

  void UpdateFromCongestionSignal(DataRate current_rate, double ce_ratio, int window_packet_count, Timestamp current_time, DataRate historical_max);
  void OnPacketLoss(DataRate current_rate, Timestamp current_time, int lost_packets, int total_packets);
  void OnAckedUpdate(Timestamp current_time, bool is_app_limited, DataRate actual_throughput, TimeDelta rtt_bloat);
  void EnterAdditiveMode(Timestamp current_time);
  void UpdateFromRtt(TimeDelta rtt);
  void UpdateEcnActivity(Timestamp current_time);
  void OnTimeUpdate(Timestamp current_time, bool is_app_limited);

  DataRate GetCurrentEstimate() const;
  void SetCurrentEstimate(DataRate rate);
  
  bool IsDiscoveryModeActive() const { return discovery_mode_active_; }
  void ExitDiscoveryMode(const std::string& reason);
  
  void SetProbeConstraint(DataRate probe_estimate, Timestamp now);
  bool HasFreshProbeCeiling(Timestamp now) const;
  void ClearProbeConstraint();
  void SetAdditiveHoldUntil(Timestamp hold_until);
  bool HasConvergedWithProbe() const;
  double GetConfidence(Timestamp now) const;

 private:
  int ComputeAdaptiveNonCeThreshold() const;
  void DecayAlpha(Timestamp current_time);
  double CalculateDiscoveryStep(double current_bps, double elapsed_s) const;
  double CalculateRecoveryStep(double current_bps, double target_probe_bps, double elapsed_s) const;
  double CalculateStableStep(double current_bps, double elapsed_s) const;
  DataRate ApplyThroughputTether(DataRate proposed_rate, DataRate actual_throughput) const;

  DataRate congestion_based_estimate_;
  DataRate min_target_rate_;
  DataRate max_target_rate_;
  DataRate pre_loss_target_; // For Fast Convergence memory
  
  TimeDelta current_rtt_;
  TimeDelta baseline_rtt_ = TimeDelta::PlusInfinity();
  
  Timestamp last_update_time_;
  Timestamp last_feedback_time_;
  Timestamp last_congestion_signal_;
  Timestamp last_md_time_ = Timestamp::MinusInfinity();
  Timestamp last_ai_update_time_ = Timestamp::MinusInfinity();
  Timestamp additive_hold_until_ = Timestamp::MinusInfinity();
  Timestamp last_ecn_feedback_;
  Timestamp last_hard_loss_time_;

  double alpha_;
  double ai_bits_accumulator_;
  int non_ce_packet_count_;
  int accumulated_lost_packets_;
  int accumulated_expected_packets_;
  bool discovery_mode_active_;
  bool first_ce_mark_detected_;
  DataRate probe_constraint_ = DataRate::Zero();
  Timestamp probe_constraint_time_ = Timestamp::MinusInfinity();

  static constexpr int kNonCeThresholdBase = 50;
  static constexpr int kNonCeThresholdMin = 20;
  static constexpr int kNonCeThresholdMax = 200;
};

// L4S Metrics Collector
class L4SMetricsCollector {
public:
  // Keep logging as fine-grained as possible while preserving timestamp order.
  static constexpr TimeDelta kBandwidthLogInterval = TimeDelta::Millis(50);
  static constexpr TimeDelta kDelayLogInterval = TimeDelta::Millis(50);
  static constexpr TimeDelta kLossLogInterval = TimeDelta::Millis(50);
  static constexpr TimeDelta kSummaryLogInterval = TimeDelta::Millis(1000);

  L4SMetricsCollector(test::MetricsLogger* logger, 
                           const std::string& test_case_name);
  ~L4SMetricsCollector();


  void LogBandwidthMetrics(Timestamp at_time,
                           DataRate target_bitrate,
                           DataRate actual_bitrate,
                           std::optional<DataRate> acked_bitrate,
                           std::optional<DataRate> send_rate);

  void LogDelayMetrics(Timestamp at_time, TimeDelta rtt, TimeDelta one_way_delay, TimeDelta jitter);
  void LogLossMetrics(Timestamp at_time, double loss_fraction, int packets_lost);
  void LogCongestionMetrics(Timestamp at_time, int ce_count, int ect_count, double congestion_ratio);
  // void LogFusionMetrics(Timestamp at_time, const L4SBandwidthFusion::BandwidthSources& sources, DataRate fused_rate);
  void LogPeriodicSummary(Timestamp at_time);

  void ExportToJsonFile(const std::string& filename);

private:
  void UpdateThroughputStats(DataRate actual_bitrate);
  void UpdateDelayStats(TimeDelta rtt, TimeDelta one_way_delay);
  void UpdateLossStats(double loss_fraction);

  test::MetricsLogger* logger_;
  std::string test_case_name_;

  // Logging rate limiting
  Timestamp last_bandwidth_log_ = Timestamp::MinusInfinity();
  Timestamp last_delay_log_ = Timestamp::MinusInfinity();
  Timestamp last_loss_log_ = Timestamp::MinusInfinity();
  Timestamp last_summary_log_ = Timestamp::MinusInfinity();

  // Statistics tracking
  webrtc::SamplesStatsCounter throughput_stats_;
  webrtc::SamplesStatsCounter rtt_stats_;
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
  friend class test::L4SNetworkControllerTest;
  // Initialization
  void InitializeBandwidthEstimators();

  // Core processing methods
  void UpdateAllBandwidthEstimators(const TransportPacketsFeedback& feedback);
  void ProcessEcnFeedback(const TransportPacketsFeedback& feedback, DataRate current_fused_rate);
  void UpdateAckedBitrateEstimator(const TransportPacketsFeedback& feedback);
  void ProcessRealProbeResults(const TransportPacketsFeedback& feedback);
  std::optional<DataRate> GetLastProbeResult();
  std::optional<webrtc::ProbeClusterConfig>CreateCustomProbe(Timestamp now, DataRate target_rate);

  
  // Probing logic
  void HandlePeriodicProbing(Timestamp now, NetworkControlUpdate* update);
  bool ShouldProbeNow(Timestamp now) const;
  void InitiateStatefulProbe(Timestamp now, NetworkControlUpdate* update, double base_multiplier, const std::string& state_name);

  // Returns true if encoder needs more headroom for probing (periodic/recovery)
  bool EncoderNeedsMoreHeadroom(double multiplier) const;
  TimeDelta GetRttScaledInterval() const;
  void StartProbeHold(Timestamp now);
  
  // Convergence detection
  // Recovery entry is intentionally simple: wait for 5 RTTs of CE silence,
  // and never enter while discovery mode is active.
  bool ShouldExitDiscoveryMode(Timestamp now) const;
  
  // Recovery detection
  void HandleRecoveryDetection(int ce_count, Timestamp now);

  // ALR detection
  bool IsApplicationLimited() const;
  void UpdateAlrDetector(const TransportPacketsFeedback& feedback);

  // Confidence calculation
  bool IsProbeDataValid(Timestamp now) const;

  // Rate control
  DataRate FuseBandwidthEstimates(Timestamp now);
  NetworkControlUpdate CreateRateUpdate(Timestamp at_time) const;
  void MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update, Timestamp at_time);

  // State management
  bool CanEnterRecoveryState(Timestamp now) const;
  void LogStateSnapshot(Timestamp now);

  bool HasRecentCongestionSignals(Timestamp now) const;
  bool IsEcnFeedbackFresh(Timestamp now) const;

  // Throughput calculation
  void UpdateThroughputWindow(const TransportPacketsFeedback& feedback);
  
  std::deque<std::pair<Timestamp, DataRate>> historical_capacity_window_;
  DataRate historical_max_capacity_ = DataRate::Zero();

  std::deque<std::pair<Timestamp, DataRate>> recent_probes_window_;

  // Metrics
  void LogPeriodicMetrics(Timestamp at_time);

  
  // Pacer transmission tracking
  std::deque<std::pair<Timestamp, int64_t>> send_rate_window_;
  webrtc::DataRate last_send_rate_ = webrtc::DataRate::Zero();

  // Environment and configuration
  const Environment env_;
  L4SControllerConfig config_;

  // Bandwidth estimation components
  std::unique_ptr<PragueCapacityEstimator> prague_estimator_;
  std::unique_ptr<ProbeController> probe_controller_;
  std::unique_ptr<ProbeBitrateEstimator> probe_bitrate_estimator_;
  std::unique_ptr<AcknowledgedBitrateEstimator> acked_estimator_;
  std::unique_ptr<AlrDetector> alr_detector_;

  // State tracking
  Timestamp last_state_snapshot_log_ = Timestamp::MinusInfinity();
  static constexpr TimeDelta kStateSnapshotLogInterval = TimeDelta::Seconds(1);
  static constexpr TimeDelta kMinimumStateDwellFloor = TimeDelta::Millis(250);

  std::optional<DataRate> target_rate_;
  std::optional<DataRate> starting_rate_;
  std::optional<DataRate> min_target_rate_;
  std::optional<DataRate> max_target_rate_;

  // ECN state
  bool ecn_supported_ = false;
  Timestamp last_congestion_signal_ = Timestamp::MinusInfinity();
  int window_ce_count_ = 0;
  int window_ect_count_ = 0;
  Timestamp window_start_time_ = Timestamp::MinusInfinity();

  // ECN policy calculation
  double CalculateEcnYieldRatio(double raw_ce_ratio, double starvation_ratio, TimeDelta rtt_bloat) const;
  void EnforceHistoricalSafetyFloor();

  // RTT tracking
  TimeDelta last_rtt_ = TimeDelta::PlusInfinity();
  TimeDelta last_smoothed_rtt_ = TimeDelta::PlusInfinity();
  TimeDelta last_estimated_round_trip_time_ = TimeDelta::Millis(50);
  // Add this to the private members of L4SNetworkController
  TimeDelta base_rtt_ = TimeDelta::PlusInfinity();

  // Loss tracking
  double last_loss_fraction_ = 0.0;
  int last_packets_lost_ = 0;

  // Probing state
  Timestamp last_probe_time_ = Timestamp::MinusInfinity();
  Timestamp probe_hold_until_ = Timestamp::MinusInfinity();
  Timestamp next_probe_allowed_at_ = Timestamp::MinusInfinity();
  Timestamp demand_high_since_ = Timestamp::MinusInfinity();
  int probe_reject_streak_ = 0;
  DataRate probe_rate_ceiling_ = DataRate::Zero();
  bool initial_probes_sent_ = false;  // SetBitrates deferred to first OnProcessInterval
  // Last bitrate reported to ProbeController via SetEstimatedBitrate.  Used to
  // suppress the call when the estimate hasn't changed meaningfully (>5%) so we
  // don't flood probe_controller.cc's "Measured bitrate" log.
  DataRate last_reported_bitrate_to_probe_controller_ = DataRate::Zero();

  // ALR state tracking for probe controller
  bool previously_in_alr_ = false;

  // Recovery state tracking
  bool recovery_mode_active_ = false;
  bool recovery_probe_bootstrapped_ = false;
  Timestamp recovery_start_time_ = Timestamp::MinusInfinity();
  // After a successful convergence exit, block re-entry for this duration to
  // prevent the rapid enter/exit oscillation seen when the network is stable.
  Timestamp recovery_cooldown_until_ = Timestamp::MinusInfinity();
  static constexpr TimeDelta kRecoveryCooldown = TimeDelta::Seconds(10);
  static constexpr double kRecoveryCeQuietRttMultiplier = 5.0;

  // Throughput calculation
  std::deque<std::pair<Timestamp, int64_t>> throughput_window_;
  DataRate last_actual_bitrate_ = DataRate::Zero();
  std::optional<DataRate> last_acked_bitrate_;

  //Pading the pacer
  std::optional<DataRate> max_padding_rate_;

  // Metrics
  bool metrics_enabled_ = true;
  std::unique_ptr<L4SMetricsCollector> metrics_collector_;
  Timestamp metrics_last_logged_ = Timestamp::MinusInfinity();
  static constexpr TimeDelta kMetricsLoggingInterval = TimeDelta::Millis(50);
};

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_
