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
  int recovery_min_clean_packets = 80;
  TimeDelta recovery_min_clean_duration = TimeDelta::Seconds(2);
  TimeDelta recovery_reentry_cooldown = TimeDelta::Seconds(8);
  
  // Confidence Thresholds
  double ecn_confidence_threshold = 0.8;
  double probe_confidence_threshold = 0.7;
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
  void OnTimeUpdate(Timestamp current_time, bool is_app_limited);

  DataRate GetCurrentEstimate() const;
  // Directly seed the internal estimate (used by the actual-rate floor guard).
  void SetCurrentEstimate(DataRate rate);
  double GetAlpha() const { return alpha_; }
  int GetDirectionFlag() const { return direction_flag_; }
  int GetNonCePacketCount() const { return non_ce_packet_count_; }
  bool IsDiscoveryModeActive() const { return discovery_mode_active_; }
  double GetConfidence(Timestamp now) const;
  
  // Probe-aware rate limiting
  void SetProbeConstraint(DataRate probe_estimate, double probe_confidence);
  void ClearProbeConstraint();
  void SetAdditiveHoldUntil(Timestamp hold_until);
  
  // Discovery mode control
  void ExitDiscoveryMode(const std::string& reason);

private:
  // Context-aware AI step calculation
  int64_t CalculateContextAwareAiStep(int64_t time_scaled_ai_step, DataRate current_rate, Timestamp current_time, double elapsed_s);

  DataRate congestion_based_estimate_;
  DataRate min_target_rate_;
  DataRate max_target_rate_;
  TimeDelta current_rtt_;
  double alpha_ = 0.0;  // DCTCP alpha parameter
  Timestamp last_update_time_;
  Timestamp last_congestion_signal_;
  Timestamp last_ecn_feedback_;  // Track any ECN activity (ECT or CE)
  

  //time-driven AI
  double ai_bits_accumulator_ = 0.0;
  Timestamp last_ai_update_time_ = Timestamp::MinusInfinity();
  Timestamp last_feedback_time_ = Timestamp::MinusInfinity();
  TimeDelta baseline_rtt_ = TimeDelta::PlusInfinity();


  // State machine for direction control
  int direction_flag_ = 1;  // 1 = increasing, -1 = reducing
  int non_ce_packet_count_ = 0;  // Count of consecutive non-CE packets
  static constexpr int kNonCeThreshold = 7;  // Threshold to switch to additive mode

  // RFC 9330 §4.3: MD must be applied at most once per RTT.
  // last_md_time_ tracks when the most recent multiplicative decrease was
  // applied so that subsequent CE-containing batches within the same RTT
  // only update alpha without re-applying the rate reduction.
  Timestamp last_md_time_ = Timestamp::MinusInfinity();

  // Discovery mode for fast startup
  bool discovery_mode_active_ = true;  // Enable aggressive discovery at startup
  bool first_ce_mark_detected_ = false;  // Track if any CE mark has been seen
  
  // Probe constraint for discovery mode
  DataRate probe_constraint_ = DataRate::Zero();
  double probe_constraint_confidence_ = 0.0;
  Timestamp additive_hold_until_ = Timestamp::MinusInfinity();



};

// Bandwidth source fusion engine
class L4SBandwidthFusion {
public:
  struct BandwidthSources {
    DataRate ecn_estimate = DataRate::Zero();
    DataRate probe_estimate = DataRate::Zero();
    DataRate acked_estimate = DataRate::Zero();
    
    double ecn_confidence = 0.0;
    double probe_confidence = 0.0;
    double acked_confidence = 0.0;
    
    Timestamp last_ecn_update = Timestamp::MinusInfinity();
    Timestamp last_probe_update = Timestamp::MinusInfinity();
    Timestamp last_acked_update = Timestamp::MinusInfinity();
  };

  explicit L4SBandwidthFusion(const L4SControllerConfig& config);
  ~L4SBandwidthFusion();

  void UpdateEcnEstimate(DataRate estimate, double confidence, Timestamp now);
  void UpdateProbeEstimate(DataRate estimate, double confidence, Timestamp now);
  void UpdateAckedEstimate(DataRate estimate, double confidence, Timestamp now);

  DataRate GetFusedEstimateWithMode(Timestamp now, bool discovery_mode, bool recovery_mode, DataRate last_actual_bitrate) const;
  BandwidthSources GetCurrentSources() const { return sources_; }

private:
  DataRate GetMostConfidentEstimate(Timestamp now) const;
  DataRate ValidateWithOtherSources(DataRate primary_estimate, const BandwidthSources& sources) const;
  DataRate GetDiscoveryModeFusedEstimate(Timestamp now, bool recovery_mode) const;
  bool IsRecentlyUpdated(Timestamp last_update, Timestamp now) const;

  BandwidthSources sources_;
  L4SControllerConfig config_;
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

  enum class ControllerState {
    kRouteReset,
    kSlowStart,
    kCongestionAvoidance,
    kCongestionExperienced,
    kCongestionRecovery,
  };

  enum class TransitionReason {
    kRouteChange,
    kInitComplete,
    kDiscoveryActive,
    kDiscoveryExit,
    kPragueReduction,
    kPragueAdditive,
    kRecoveryEntered,
    kRecoveryExited,
  };

  struct TransitionDecision {
    ControllerState next_state;
    TransitionReason reason;
  };

  // Initialization
  void InitializeBandwidthEstimators();

  // Core processing methods
  void UpdateAllBandwidthEstimators(const TransportPacketsFeedback& feedback);
  void ProcessEcnFeedback(const TransportPacketsFeedback& feedback, DataRate current_fused_rate);
  DataRate DetermineBottleneckAwareTarget(DataRate fused_rate);
  void UpdateAckedBitrateEstimator(const TransportPacketsFeedback& feedback);
  void ProcessRealProbeResults(const TransportPacketsFeedback& feedback);
  std::optional<DataRate> GetLastProbeResult();
  std::optional<webrtc::ProbeClusterConfig>CreateCustomProbe(Timestamp now, DataRate target_rate);

  // State-owned policy handlers (Phase 3)
  void ApplyStateEcnPolicy(const TransportPacketsFeedback& feedback,
                           DataRate base_fused_rate);
  void ApplyStateProbingPolicy(Timestamp now, NetworkControlUpdate* update);
  DataRate ApplyStateFusionPolicy(Timestamp now);
  
  // Bandwidth fusion methods
  DataRate GetBaseFusedEstimate(Timestamp now);

  // Probing logic
  void HandlePeriodicProbing(Timestamp now, NetworkControlUpdate* update);
  bool ShouldProbeNow(Timestamp now) const;
  void InitiateProbing(Timestamp now, NetworkControlUpdate* update);
  void InitiateRecoveryProbing(Timestamp now, NetworkControlUpdate* update);
  // Returns true if encoder needs more headroom for probing (periodic/recovery)
  bool EncoderNeedsMoreHeadroom(double multiplier) const;
  TimeDelta GetRttScaledInterval() const;
  void StartProbeHold(Timestamp now);
  
  // Convergence detection
  bool CheckProbeAndPragueConvergence(Timestamp now) const;
  bool ShouldExitDiscoveryMode(Timestamp now) const;
  bool IsRecentlyUpdated(Timestamp last_update, Timestamp now) const;
  
  // Recovery detection
  void HandleRecoveryDetection(int ect_count, int ce_count, Timestamp now);

  // ALR detection
  bool IsApplicationLimited() const;
  void UpdateAlrDetector(const TransportPacketsFeedback& feedback);

  // Confidence calculation
  double CalculateProbeConfidence(Timestamp now) const;
  double CalculateAckedConfidence(Timestamp now) const;

  // Rate control
  DataRate FuseBandwidthEstimates(Timestamp now);
  NetworkControlUpdate CreateRateUpdate(Timestamp at_time) const;
  void MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update, Timestamp at_time);

  // State management
  void AdvanceStateMachine(Timestamp now);
  std::optional<TransitionDecision> EvaluateStateTransition(Timestamp now) const;
  TimeDelta GetMinimumStateDwell() const;
  bool CanEnterRecoveryState(Timestamp now) const;
  void LogStateSnapshot(Timestamp now);
  static const char* StateToString(ControllerState state);
  static const char* TransitionReasonToString(TransitionReason reason);
  void TransitionToState(ControllerState new_state,
                         TransitionReason reason,
                         Timestamp at_time);
  bool HasRecentCongestionSignals(Timestamp now) const;
  bool IsEcnFeedbackFresh(Timestamp now) const;

  // Throughput calculation
  void UpdateThroughputWindow(const TransportPacketsFeedback& feedback);

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
  std::unique_ptr<L4SBandwidthFusion> bandwidth_fusion_;

  // State tracking
  ControllerState controller_state_ = ControllerState::kRouteReset;
  Timestamp state_entered_at_ = Timestamp::MinusInfinity();
  uint64_t state_transition_count_ = 0;
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

  // RTT tracking
  TimeDelta last_rtt_ = TimeDelta::PlusInfinity();
  TimeDelta last_estimated_round_trip_time_ = TimeDelta::Millis(50);
  // Add this to the private members of L4SNetworkController
  TimeDelta base_rtt_ = TimeDelta::PlusInfinity();

  // Loss tracking
  double last_loss_fraction_ = 0.0;
  int last_packets_lost_ = 0;

  // Probing state
  Timestamp last_probe_time_ = Timestamp::MinusInfinity();
  Timestamp probe_hold_until_ = Timestamp::MinusInfinity();
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
  int consecutive_clean_packets_ = 0;  // ECT1 without CE
  Timestamp clean_ect_run_start_ = Timestamp::MinusInfinity();
  Timestamp recovery_start_time_ = Timestamp::MinusInfinity();
  // After a successful convergence exit, block re-entry for this duration to
  // prevent the rapid enter/exit oscillation seen when the network is stable.
  Timestamp recovery_cooldown_until_ = Timestamp::MinusInfinity();
  static constexpr TimeDelta kRecoveryCooldown = TimeDelta::Seconds(10);
  static constexpr int kRecoveryPacketThreshold = 20; // floor for dynamic threshold (see HandleRecoveryDetection)

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
