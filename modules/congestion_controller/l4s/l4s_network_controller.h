#ifndef MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_
#define MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_

#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include "api/transport/network_control.h"
#include "api/units/data_rate.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "api/numerics/samples_stats_counter.h"
#include "modules/congestion_controller/l4s/l4s_prague_controller.h"

namespace webrtc {

// Forward declarations
// class AcknowledgedBitrateEstimator;
// class DelayBasedBwe;
// class SendSideBandwidthEstimation;
// class ProbeController;
namespace test {
class MetricsLogger;
}
class Environment;
struct NetworkStateEstimate;
struct TransportPacketsFeedback;
struct NetworkControllerConfig;

struct L4SControllerConfig {
  // Whether to fallback to GCC if L4S isn't supported
  bool fallback_to_gcc = true;
  // Whether to use ECT(1) marking for packets
  bool use_ect1_marking = true;
  // Metrics collection configuration
  bool enable_metrics_collection = true;
  std::string test_case_name = "l4s_vs_gcc_comparison";
};

// Adaptive capacity estimator for realistic bandwidth estimation
class AdaptiveCapacityEstimator {
 public:
  explicit AdaptiveCapacityEstimator(DataRate starting_rate, DataRate min_target_rate, DataRate max_target_rate);
  ~AdaptiveCapacityEstimator();
  
  // Update estimates based on different signals
  void UpdateFromCongestionSignal(DataRate current_rate, double ce_ratio, Timestamp current_time);
  void UpdateFromSustainedRate(DataRate sustained_rate, Timestamp current_time);
  void UpdateFromRtt(TimeDelta rtt);
  void OnPacketLoss(DataRate current_rate, Timestamp current_time);


  DataRate GetHistoricMin() const { return historic_min_; }
  DataRate GetHistoricMax() const { return historic_max_; }
  void SetHistoricMin(DataRate rate) { historic_min_ = rate; }
  void SetHistoricMax(DataRate rate) { historic_max_ = rate; }
  // Get the current adaptive estimate
  DataRate GetMaxRealisticBandwidth() const;
  
  // Decay estimates over time if not reinforced
  void OnTimeUpdate(Timestamp current_time);
  
 private:
  enum class ConnectionType {
    MOBILE_SLOW,    // < 5 Mbps
    MOBILE_FAST,    // 5-50 Mbps  
    WIFI_TYPICAL,   // 10-100 Mbps
    WIRED_FAST,     // 100+ Mbps
    UNKNOWN
  };
  double alpha_ = 0.0;
  
  DataRate historic_min_;
  DataRate historic_max_;

  // Different estimate sources
  DataRate congestion_based_estimate_;
  DataRate min_target_rate_;
  DataRate max_target_rate_;
  
  // Tracking data
  std::deque<DataRate> sustained_rates_history_;
  static constexpr size_t kHistoryWindowSize = 10;
  static constexpr TimeDelta kDecayInterval = TimeDelta::Seconds(30);
  
  Timestamp last_update_time_;
  TimeDelta min_rtt_ = TimeDelta::PlusInfinity();
  
  // Absolute limits
  static constexpr DataRate kAbsoluteMaxLimit = DataRate::KilobitsPerSec(1000000); // 1 Gbps
  static constexpr DataRate kAbsoluteMinLimit = DataRate::KilobitsPerSec(300);    // 300 Kbps
};

// L4S Metrics Collector for comprehensive performance analysis
class L4SMetricsCollector {
 public:
  L4SMetricsCollector(test::MetricsLogger* logger, 
                      const std::string& test_case_name,
                      Clock* clock);
  
  // Time-series metrics logging
  void LogBandwidthMetrics(Timestamp at_time, DataRate target_bitrate, 
                          DataRate actual_bitrate);
  void LogDelayMetrics(Timestamp at_time, TimeDelta rtt, TimeDelta one_way_delay, 
                      TimeDelta jitter = TimeDelta::Zero());
  void LogLossMetrics(Timestamp at_time, double loss_fraction, int packets_lost);
  void LogCongestionMetrics(Timestamp at_time, int ce_count, int ect_count, 
                           double congestion_ratio);

  // Periodic summary metrics
  void LogPeriodicSummary(Timestamp at_time);
  
  // Utility methods for stats tracking
  void UpdateThroughputStats(DataRate actual_bitrate);
  void UpdateDelayStats(TimeDelta rtt);
  void UpdateLossStats(double loss_fraction);
  void ExportToJsonFile(const std::string& filename);

  
 private:
  test::MetricsLogger* logger_;
  std::string test_case_name_;
  Clock* clock_;
  double rfc3550_jitter_ = 0.0;
  
  // Statistics tracking
  SamplesStatsCounter throughput_stats_;
  SamplesStatsCounter delay_stats_;
  SamplesStatsCounter loss_stats_;
  
  // Last logged values to prevent spam
  Timestamp last_bandwidth_log_ = Timestamp::MinusInfinity();
  Timestamp last_delay_log_ = Timestamp::MinusInfinity();
  Timestamp last_loss_log_ = Timestamp::MinusInfinity();
  Timestamp last_summary_log_ = Timestamp::MinusInfinity();
  
  // Minimum intervals between logs
  static constexpr TimeDelta kBandwidthLogInterval = TimeDelta::Millis(100);
  static constexpr TimeDelta kDelayLogInterval = TimeDelta::Millis(100);
  static constexpr TimeDelta kLossLogInterval = TimeDelta::Millis(500);
  static constexpr TimeDelta kSummaryLogInterval = TimeDelta::Seconds(10);
};

// Implementation of Network Controller Interface that uses L4S-based
// congestion control. It can reuse some components from GCC when needed
// and falls back to GCC if L4S (ECN) isn't supported.
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
  NetworkControlUpdate OnTargetRateConstraints(
      TargetRateConstraints msg) override;
  NetworkControlUpdate OnTransportLossReport(TransportLossReport msg) override;
  NetworkControlUpdate OnTransportPacketsFeedback(
      TransportPacketsFeedback msg) override;
  NetworkControlUpdate OnNetworkStateEstimate(
      NetworkStateEstimate msg) override;


 private:
  NetworkControlUpdate CreateRateUpdate(Timestamp at_time) const;
  void MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update,
                                    Timestamp at_time);
  bool IsL4SActive() const;
  
  void ProcessEcnFeedback(const TransportPacketsFeedback& feedback);
  void UpdateNetworkCapacityEstimate(const TransportPacketsFeedback& feedback);
  

  std::optional<Timestamp> last_update_time_;
  TimeDelta update_interval_ = TimeDelta::Millis(25);

  // Environment
  const Environment env_;
  const bool fallback_to_gcc_;
  const bool use_ect1_marking_;

  // // L4S-specific controllers
  // std::unique_ptr<L4SPragueController> prague_controller_;
  
  // Current state
  bool ecn_supported_ = false;
  bool ecn_capable_network_ = false;
  std::optional<DataRate> target_rate_;
  std::optional<DataRate> min_target_rate_;
  std::optional<DataRate> max_target_rate_;
  std::optional<DataRate> starting_rate_;
  
  // Tracking congestion signals
  int ce_count_ = 0;
  int ect_count_ = 0;
  Timestamp last_congestion_signal_ = Timestamp::MinusInfinity();
  
  // For fallback to GCC if needed
  std::unique_ptr<NetworkControllerInterface> gcc_controller_;

  // Add bandwidth estimation integration to the private section
  std::optional<DataRate> estimated_bandwidth_;
  
  // Adaptive capacity estimation
  std::unique_ptr<AdaptiveCapacityEstimator> capacity_estimator_;
  DataRate max_realistic_bandwidth_ = DataRate::KilobitsPerSec(100000); // Will be replaced by adaptive estimator

  // Bandwidth estimation tracking
  DataRate last_acknowledged_rate_ = DataRate::Zero();
  DataRate last_delay_based_estimate_ = DataRate::Zero();
  
  // Enhanced RTT tracking (similar to GCC)
  std::deque<int64_t> feedback_max_rtts_;
  TimeDelta last_estimated_round_trip_time_ = TimeDelta::PlusInfinity();
  
  
  // Metrics collection
  std::unique_ptr<L4SMetricsCollector> metrics_collector_;
  bool metrics_enabled_ = true;
  Timestamp metrics_last_logged_ = Timestamp::MinusInfinity();
  static constexpr TimeDelta kMetricsLoggingInterval = TimeDelta::Millis(100);
  
  // Performance tracking for metrics
  DataRate last_actual_bitrate_ = DataRate::Zero();
  DataRate last_target_bitrate_ = DataRate::Zero();
  TimeDelta last_rtt_ = TimeDelta::PlusInfinity();
  TimeDelta jitter_ = TimeDelta::Zero();
  double last_loss_fraction_ = 0.0;
  int last_packets_lost_ = 0;
  std::string current_active_controller_ = "initializing";
  
  // Helper methods for metrics
  void LogPeriodicMetrics(Timestamp at_time);
  void LogControllerState(Timestamp at_time);


};

/*
 * L4S Network Controller with GCC-Inspired Bandwidth Estimation
 * 
 * Comprehensive Improvements:
 * 1. Integrated GCC's acknowledged bitrate estimator for actual throughput measurement
 * 2. Added delay-based BWE for detecting network congestion through packet delays  
 * 3. Enhanced RTT tracking with moving window analysis (like GCC)
 * 4. Network capacity estimation that learns from actual network behavior
 * 5. BWE-informed rate capping that respects real network conditions
 * 6. Prague controller integration with proper bandwidth constraints
 * 
 * This prevents the 100 Mbps VM network from being overdriven to 2+ Gbps
 * by providing realistic network capacity estimates and proper feedback loops.
 */

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_
