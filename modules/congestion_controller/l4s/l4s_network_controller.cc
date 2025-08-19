#include "modules/congestion_controller/l4s/l4s_network_controller.h"

#include <algorithm>
#include <memory>
#include <numeric>
#include <utility>

#include "absl/strings/match.h"
#include "api/field_trials_view.h"
#include "api/rtc_event_log/rtc_event_log.h"
#include "api/transport/bandwidth_usage.h"
#include "api/transport/network_types.h"
#include "api/units/data_rate.h"
#include "api/units/data_size.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "logging/rtc_event_log/events/rtc_event_probe_cluster_created.h"
#include "rtc_base/checks.h"
#include "rtc_base/experiments/field_trial_parser.h"
#include "rtc_base/logging.h"
#include "system_wrappers/include/metrics.h"

// Metrics collection
#include "api/test/metrics/global_metrics_logger_and_exporter.h"
#include "api/test/metrics/metrics_logger.h"

namespace webrtc {

// =============================================================================
// PragueCapacityEstimator Implementation
// =============================================================================

PragueCapacityEstimator::PragueCapacityEstimator(DataRate starting_rate, DataRate min_rate, DataRate max_rate)
    : congestion_based_estimate_(starting_rate),
      min_target_rate_(min_rate),
      max_target_rate_(max_rate),
      current_rtt_(TimeDelta::Millis(50)),
      last_update_time_(Timestamp::MinusInfinity()),
      last_congestion_signal_(Timestamp::MinusInfinity()),
      direction_flag_(1),
      non_ce_packet_count_(0) {
  
  // Clamp initial estimate to bounds
  if (congestion_based_estimate_ < min_target_rate_) {
    congestion_based_estimate_ = min_target_rate_;
  }
  if (congestion_based_estimate_ > max_target_rate_) {
    congestion_based_estimate_ = max_target_rate_;
  }
}

PragueCapacityEstimator::~PragueCapacityEstimator() = default;

void PragueCapacityEstimator::UpdateFromCongestionSignal(DataRate current_rate, double ce_ratio, Timestamp current_time) {
  constexpr int kDefaultMssBytes = 1440;  // Typical Ethernet MSS
  
  TimeDelta rtt = current_rtt_.IsFinite() && !current_rtt_.IsZero() ? current_rtt_ : TimeDelta::Millis(1);
  double rtt_seconds = rtt.seconds<double>();
  if (rtt_seconds <= 0.0) {
    rtt_seconds = 0.001;  // Fallback RTT (1ms for VM testbed)
  }

  // Prague DCTCP-style rate adaptation with state machine (RFC 9330)
  if (ce_ratio > 0.0) {  // CE-marked packets detected
    // Reset non-CE packet count when we see CE marks
    non_ce_packet_count_ = 0;
    
    // Only perform reduction if we're in increasing mode (flag = 1)
    if (direction_flag_ == 1) {
      // DCTCP-style alpha update with standard EWMA gain
      constexpr double g = 1.0 / 16.0;  // RFC 9330 standard gain
      alpha_ = (1.0 - g) * alpha_ + g * ce_ratio;
      
      // Proportional decrease (much gentler than 50% reduction)
      double reduction_factor = 1.0 - alpha_ / 2.0;
      DataRate reduced = std::max(current_rate * reduction_factor, min_target_rate_);
      congestion_based_estimate_ = reduced;
      
      // Switch to reduction mode
      direction_flag_ = -1;
      
      // Update congestion signal timestamp
      last_congestion_signal_ = current_time;
      
      RTC_LOG(LS_INFO) << "Prague: Switched to reduction mode (alpha=" << alpha_
                       << ", ce_ratio=" << ce_ratio 
                       << ", reduction_factor=" << reduction_factor
                       << "), new rate=" << congestion_based_estimate_.bps() << " bps";
    } else {
      RTC_LOG(LS_VERBOSE) << "Prague: CE marks detected but already in reduction mode, ignoring";
    }
                     
  } else {  // No CE marks in this batch
    // Increment non-CE packet count
    non_ce_packet_count_++;
    
    // Check if we should switch from reduction mode to additive mode
    if (direction_flag_ == -1 && non_ce_packet_count_ >= kNonCeThreshold) {
      direction_flag_ = 1;
      non_ce_packet_count_ = 0;  // Reset counter
      RTC_LOG(LS_INFO) << "Prague: Switched to additive mode after " << kNonCeThreshold 
                       << " consecutive non-CE packets";
    }
    
    // Only perform additive increase if we're in increasing mode (flag = 1)
    if (direction_flag_ == 1 && current_rate >= congestion_based_estimate_ * 0.9) {
    
    // Prague DCTCP additive increase: +1 MSS per RTT period
    // This is the fundamental L4S congestion control behavior
    int64_t bits_per_rtt = kDefaultMssBytes * 8;  // 1440 * 8 = 11520 bits
    
    // Calculate the theoretical AI rate: 1 MSS worth of extra bits per RTT period
    int64_t theoretical_ai_bps = static_cast<int64_t>(bits_per_rtt / rtt_seconds);
    
    // Apply conservative limits to prevent explosive growth
    int64_t ai_step_bps = std::min(
        theoretical_ai_bps,  // DCTCP standard AI rate
        static_cast<int64_t>(congestion_based_estimate_.bps() * 0.05)  // Limit to 5% increase per step
    );
    
    DataRate increased = congestion_based_estimate_ + DataRate::BitsPerSec(ai_step_bps);
    
    // Don't exceed maximum rate
    if (max_target_rate_ > DataRate::Zero()) {
        increased = std::min(increased, max_target_rate_);
    }
    
    congestion_based_estimate_ = increased;
    
    RTC_LOG(LS_INFO) << "Prague: DCTCP additive increase (+1.0 MSS/RTT, rtt=" << rtt.ms() << " ms, "
                     << "step=" << ai_step_bps << " bps), new rate=" 
                     << congestion_based_estimate_.bps() << " bps, non_ce_count=" << non_ce_packet_count_;
    } else if (direction_flag_ == -1) {
      RTC_LOG(LS_VERBOSE) << "Prague: Skipping AI, in reduction mode (need " 
                          << (kNonCeThreshold - non_ce_packet_count_) 
                          << " more non-CE packets to switch)";
    }
  }
  
  last_update_time_ = current_time;
}

void PragueCapacityEstimator::UpdateFromRtt(TimeDelta rtt) {
  if (rtt.IsFinite() && !rtt.IsZero()) {
    current_rtt_ = rtt;
  }
}

void PragueCapacityEstimator::OnPacketLoss(DataRate current_rate, Timestamp current_time) {
  // Multiplicative decrease for packet loss (fallback mechanism)
  DataRate reduced = std::max(current_rate * 0.5, min_target_rate_);
  congestion_based_estimate_ = reduced;
  
  // Switch to reduction mode and reset non-CE counter
  direction_flag_ = -1;
  non_ce_packet_count_ = 0;
  
  RTC_LOG(LS_WARNING) << "Prague: Packet loss detected, halving estimate to "
                      << congestion_based_estimate_.bps() << " bps, switched to reduction mode";
  
  last_update_time_ = current_time;
}

void PragueCapacityEstimator::OnTimeUpdate(Timestamp current_time) {
  if (last_update_time_.IsInfinite()) {
    last_update_time_ = current_time;
    return;
  }
  
  TimeDelta elapsed = current_time - last_update_time_;
  if (elapsed >= kDecayInterval) {
    // Gradually decay estimates if not reinforced
    congestion_based_estimate_ = std::max(congestion_based_estimate_ * 0.95, min_target_rate_);
    last_update_time_ = current_time;
  }
}

DataRate PragueCapacityEstimator::GetCurrentEstimate() const {
  return congestion_based_estimate_;
}

double PragueCapacityEstimator::GetConfidence(Timestamp now) const {
  if (last_congestion_signal_.IsInfinite()) {
    return 0.3;  // Low confidence without any ECN feedback
  }
  
  TimeDelta since_signal = now - last_congestion_signal_;
  if (since_signal < TimeDelta::Seconds(2)) {
    return 0.9;  // Very confident with recent ECN feedback
  } else if (since_signal < TimeDelta::Seconds(5)) {
    return 0.7;  // Moderately confident
  }
  return 0.4;  // Lower confidence with stale ECN feedback
}

// =============================================================================
// L4SBandwidthFusion Implementation
// =============================================================================

L4SBandwidthFusion::L4SBandwidthFusion(const L4SControllerConfig& config) : config_(config) {}

L4SBandwidthFusion::~L4SBandwidthFusion() = default;

void L4SBandwidthFusion::UpdateEcnEstimate(DataRate estimate, double confidence, Timestamp now) {
  RTC_LOG(LS_INFO) << "L4S: Updating ECN estimate to " << estimate.bps() << " bps with confidence " << confidence;
  sources_.ecn_estimate = estimate;
  sources_.ecn_confidence = confidence;
  sources_.last_ecn_update = now;
}

void L4SBandwidthFusion::UpdateDelayEstimate(DataRate estimate, double confidence, Timestamp now) {
  RTC_LOG(LS_INFO) << "L4S: Updating delay estimate to " << estimate.bps() << " bps with confidence " << confidence;
  sources_.delay_estimate = estimate;
  sources_.delay_confidence = confidence;
  sources_.last_delay_update = now;
}

void L4SBandwidthFusion::UpdateProbeEstimate(DataRate estimate, double confidence, Timestamp now) {
  RTC_LOG(LS_INFO) << "L4S: Updating probe estimate to " << estimate.bps() << " bps with confidence " << confidence;
  sources_.probe_estimate = estimate;
  sources_.probe_confidence = confidence;
  sources_.last_probe_update = now;
}

void L4SBandwidthFusion::UpdateAckedEstimate(DataRate estimate, double confidence, Timestamp now) {
  RTC_LOG(LS_INFO) << "L4S: Updating acked estimate to " << estimate.bps() << " bps with confidence " << confidence;
  sources_.acked_estimate = estimate;
  sources_.acked_confidence = confidence;
  sources_.last_acked_update = now;
}

DataRate L4SBandwidthFusion::GetFusedEstimate(Timestamp now) const {
  // L4S Fusion: Prague ECN provides congestion control authority,
  // other estimators provide capacity discovery insights
  
  // 1. Prague ECN estimate has highest priority for congestion control
  // When Prague detects congestion, it overrides other estimates
  if (sources_.ecn_confidence > config_.ecn_confidence_threshold && 
      IsRecentlyUpdated(sources_.last_ecn_update, now)) {
    RTC_LOG(LS_VERBOSE) << "L4S: Using Prague ECN estimate: " << sources_.ecn_estimate.bps() << " bps";
    return sources_.ecn_estimate;
  }
  
  // 2. When no recent congestion, use probe results as capacity upper bound
  if (sources_.probe_confidence > config_.probe_confidence_threshold && 
      IsRecentlyUpdated(sources_.last_probe_update, now)) {
    DataRate capacity_estimate = ValidateWithOtherSources(sources_.probe_estimate, sources_);
    
    // But don't exceed Prague's current estimate if it's lower (recent congestion)
    if (sources_.ecn_confidence > 0.3) {
      capacity_estimate = std::min(capacity_estimate, sources_.ecn_estimate * 1.1);
    }
    
    RTC_LOG(LS_VERBOSE) << "L4S: Using probe-based capacity: " << capacity_estimate.bps() << " bps";
    return capacity_estimate;
  }
  
  // 3. Fallback to conservative combination of delay and acked estimates
  double total_weight = sources_.delay_confidence + sources_.acked_confidence;
  if (total_weight > 0.1) {
    DataRate weighted_estimate = 
        (sources_.delay_estimate * sources_.delay_confidence + 
         sources_.acked_estimate * sources_.acked_confidence) / total_weight;
    
    // Always respect Prague's congestion authority
    if (sources_.ecn_confidence > 0.3) {
      weighted_estimate = std::min(weighted_estimate, sources_.ecn_estimate);
    }
    
    RTC_LOG(LS_VERBOSE) << "L4S: Using weighted delay+acked estimate: " << weighted_estimate.bps() << " bps";
    return weighted_estimate;
  }
  
  // 4. Final fallback to most confident single estimate
  RTC_LOG(LS_VERBOSE) << "L4S: Using fallback estimate";
  return GetMostConfidentEstimate(now);
}

DataRate L4SBandwidthFusion::GetMostConfidentEstimate(Timestamp now) const {
  DataRate best_estimate = DataRate::KilobitsPerSec(300);  // Fallback
  double best_confidence = 0.0;
  
  if (sources_.ecn_confidence > best_confidence && IsRecentlyUpdated(sources_.last_ecn_update, now)) {
    best_estimate = sources_.ecn_estimate;
    best_confidence = sources_.ecn_confidence;
  }
  
  if (sources_.delay_confidence > best_confidence && IsRecentlyUpdated(sources_.last_delay_update, now)) {
    best_estimate = sources_.delay_estimate;
    best_confidence = sources_.delay_confidence;
  }
  
  if (sources_.probe_confidence > best_confidence && IsRecentlyUpdated(sources_.last_probe_update, now)) {
    best_estimate = sources_.probe_estimate;
    best_confidence = sources_.probe_confidence;
  }
  
  if (sources_.acked_confidence > best_confidence && IsRecentlyUpdated(sources_.last_acked_update, now)) {
    best_estimate = sources_.acked_estimate;
    best_confidence = sources_.acked_confidence;
  }
  
  return best_estimate;
}

DataRate L4SBandwidthFusion::ValidateWithOtherSources(DataRate primary_estimate, const BandwidthSources& sources) const {
  // Don't allow probe results that are dramatically higher than other estimates
  DataRate max_alternative = DataRate::Zero();
  
  if (sources.ecn_confidence > 0.3) {
    max_alternative = std::max(max_alternative, sources.ecn_estimate);
  }
  if (sources.delay_confidence > 0.3) {
    max_alternative = std::max(max_alternative, sources.delay_estimate);
  }
  if (sources.acked_confidence > 0.3) {
    max_alternative = std::max(max_alternative, sources.acked_estimate);
  }
  
  if (max_alternative > DataRate::Zero() && primary_estimate > max_alternative * 2.0) {
    // Probe result seems too optimistic, cap it
    return max_alternative * 1.5;
  }
  
  return primary_estimate;
}

bool L4SBandwidthFusion::IsRecentlyUpdated(Timestamp last_update, Timestamp now) const {
  return !last_update.IsInfinite() && (now - last_update) < TimeDelta::Seconds(10);
}

// =============================================================================
// L4SMetricsCollector Implementation
// =============================================================================

L4SMetricsCollector::L4SMetricsCollector(test::MetricsLogger* logger, 
                                                    const std::string& test_case_name,
                                                    Clock* clock)
    : logger_(logger), test_case_name_(test_case_name), clock_(clock) {
  RTC_CHECK(logger_);
  RTC_CHECK(clock_);
  RTC_LOG(LS_INFO) << "L4SMetricsCollector initialized for test case: " << test_case_name_;
}

L4SMetricsCollector::~L4SMetricsCollector() = default;

void L4SMetricsCollector::LogBandwidthMetrics(Timestamp at_time, DataRate target_bitrate, DataRate actual_bitrate) {
  if (at_time - last_bandwidth_log_ < kBandwidthLogInterval) {
    return;
  }
  
  last_bandwidth_log_ = at_time;
  UpdateThroughputStats(actual_bitrate);
  
  logger_->LogSingleValueMetric("bandwidth_target_mbps", test_case_name_, target_bitrate.bps() / 1e6, 
                                webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  
  logger_->LogSingleValueMetric("bandwidth_actual_mbps", test_case_name_, actual_bitrate.bps() / 1e6, 
                                webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogDelayMetrics(Timestamp at_time, TimeDelta rtt, TimeDelta one_way_delay, TimeDelta jitter) {
  if (at_time - last_delay_log_ < kDelayLogInterval) {
    return;
  }
  
  last_delay_log_ = at_time;
  UpdateDelayStats(rtt);
  
  logger_->LogSingleValueMetric("rtt_ms", test_case_name_, rtt.ms(), 
                                webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  
  if (one_way_delay.IsFinite()) {
    logger_->LogSingleValueMetric("one_way_delay_ms", test_case_name_, one_way_delay.ms(), 
                                  webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"timestamp_ms", std::to_string(at_time.ms())}});
  }
}

void L4SMetricsCollector::LogLossMetrics(Timestamp at_time, double loss_fraction, int packets_lost) {
  if (at_time - last_loss_log_ < kLossLogInterval) {
    return;
  }
  
  last_loss_log_ = at_time;
  UpdateLossStats(loss_fraction);
  
  logger_->LogSingleValueMetric("packets_lost_count", test_case_name_, packets_lost, 
                                webrtc::test::Unit::kCount, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogCongestionMetrics(Timestamp at_time, int ce_count, int ect_count, double congestion_ratio) {
  logger_->LogSingleValueMetric("congestion_ce_count", test_case_name_, ce_count, 
                                webrtc::test::Unit::kCount, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  
  logger_->LogSingleValueMetric("congestion_ratio", test_case_name_, congestion_ratio, 
                                webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogFusionMetrics(Timestamp at_time, const L4SBandwidthFusion::BandwidthSources& sources, DataRate fused_rate) {
  logger_->LogSingleValueMetric("fusion_ecn_estimate_mbps", test_case_name_, sources.ecn_estimate.bps() / 1e6, 
                                webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  
  logger_->LogSingleValueMetric("fusion_delay_estimate_mbps", test_case_name_, sources.delay_estimate.bps() / 1e6, 
                                webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  
  logger_->LogSingleValueMetric("fusion_probe_estimate_mbps", test_case_name_, sources.probe_estimate.bps() / 1e6, 
                                webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  
  logger_->LogSingleValueMetric("fusion_fused_rate_mbps", test_case_name_, fused_rate.bps() / 1e6, 
                                webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogPeriodicSummary(Timestamp at_time) {
  if (at_time - last_summary_log_ < kSummaryLogInterval) {
    return;
  }
  
  last_summary_log_ = at_time;
  
  if (throughput_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric("throughput_avg_mbps", test_case_name_, throughput_stats_.GetAverage() / 1e6, 
                                  webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "throughput"}});
  }
  
  if (delay_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric("delay_avg_ms", test_case_name_, delay_stats_.GetAverage(), 
                                  webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "delay"}});
  }
  
  ExportToJsonFile("l4s_network_metrics.json");
}

void L4SMetricsCollector::ExportToJsonFile(const std::string& filename) {
  if (!logger_) return;
  auto metrics = logger_->GetCollectedMetrics();
  FILE* f = fopen(filename.c_str(), "w");
  if (!f) return;
  
  fprintf(f, "[\n");
  for (size_t i = 0; i < metrics.size(); ++i) {
    const auto& m = metrics[i];
    fprintf(f, "  {\n");
    fprintf(f, "    \"name\": \"%s\",\n", m.name.c_str());
    fprintf(f, "    \"samples\": [");
    for (size_t j = 0; j < m.time_series.samples.size(); ++j) {
      const auto& s = m.time_series.samples[j];
      fprintf(f, "%s{\"timestamp\": %lld, \"value\": %f}",
        (j > 0 ? ", " : ""), static_cast<long long>(s.timestamp.us()), s.value);
    }
    fprintf(f, "]\n  }%s\n", (i + 1 < metrics.size()) ? "," : "");
  }
  fprintf(f, "]\n");
  fclose(f);
}

void L4SMetricsCollector::UpdateThroughputStats(DataRate actual_bitrate) {
  throughput_stats_.AddSample(actual_bitrate.bps());
}

void L4SMetricsCollector::UpdateDelayStats(TimeDelta rtt) {
  if (rtt.IsFinite()) {
    delay_stats_.AddSample(rtt.ms());
  }
}

void L4SMetricsCollector::UpdateLossStats(double loss_fraction) {
  loss_stats_.AddSample(loss_fraction);
}

// =============================================================================
// L4SNetworkController Implementation
// =============================================================================

L4SNetworkController::L4SNetworkController(NetworkControllerConfig config,
                                          L4SControllerConfig l4s_config,
                                          test::MetricsLogger* metrics_logger)
    : env_(config.env), config_(l4s_config) {
  
  // Initialize Prague capacity estimator
  DataRate starting_rate = config.constraints.starting_rate.value_or(DataRate::KilobitsPerSec(300));
  DataRate min_rate = config.constraints.min_data_rate.value_or(DataRate::KilobitsPerSec(30));
  DataRate max_rate = config.constraints.max_data_rate.value_or(DataRate::KilobitsPerSec(100000));
  
  prague_estimator_ = std::make_unique<PragueCapacityEstimator>(starting_rate, min_rate, max_rate);
  
  // Initialize bandwidth fusion engine
  bandwidth_fusion_ = std::make_unique<L4SBandwidthFusion>(config_);
  
  // Initialize bandwidth estimation components
  InitializeBandwidthEstimators();
  
  // Initialize metrics collector
  if (config_.enable_metrics_collection) {
    using webrtc::test::GetGlobalMetricsLogger;
    test::MetricsLogger* logger_to_use = metrics_logger;
    if (!logger_to_use) {
      logger_to_use = GetGlobalMetricsLogger();
    }
    metrics_collector_ = std::make_unique<L4SMetricsCollector>(
        logger_to_use, config_.test_case_name, &env_.clock());
  }
  
  // Set initial rate constraints
  starting_rate_ = config.constraints.starting_rate;
  min_target_rate_ = config.constraints.min_data_rate;
  max_target_rate_ = config.constraints.max_data_rate;
  target_rate_ = starting_rate;
  
  RTC_LOG(LS_INFO) << "L4SNetworkController created with starting rate: " 
                   << starting_rate.bps() << " bps";
}

L4SNetworkController::~L4SNetworkController() {
  if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->ExportToJsonFile("l4s_network_metrics.json");
    RTC_LOG(LS_INFO) << "L4S: Exported metrics to l4s_network_metrics.json";
  }
}

void L4SNetworkController::InitializeBandwidthEstimators() {
  if (config_.enable_delay_estimation) {
    delay_estimator_ = std::make_unique<DelayBasedBwe>(&env_.field_trials(), nullptr, nullptr);
  }
  
  if (config_.enable_probing) {
    probe_controller_ = std::make_unique<ProbeController>(&env_.field_trials(), nullptr);
  }
  
  if (config_.enable_acked_estimation) {
    acked_estimator_ = std::make_unique<AcknowledgedBitrateEstimator>(&env_.field_trials(), nullptr);
  }
  
  if (config_.enable_alr_detection) {
    alr_detector_ = std::make_unique<AlrDetector>(&env_.field_trials());
  }
  
  RTC_LOG(LS_INFO) << "L4S: Initialized bandwidth estimators - "
                   << "Delay: " << (delay_estimator_ ? "enabled" : "disabled")
                   << ", Probe: " << (probe_controller_ ? "enabled" : "disabled")
                   << ", Acked: " << (acked_estimator_ ? "enabled" : "disabled")
                   << ", ALR: " << (alr_detector_ ? "enabled" : "disabled");
}

NetworkControlUpdate L4SNetworkController::OnNetworkAvailability(NetworkAvailability msg) {
  NetworkControlUpdate update;
  return update;
}

NetworkControlUpdate L4SNetworkController::OnNetworkRouteChange(NetworkRouteChange msg) {
  NetworkControlUpdate update;
  
  RTC_LOG(LS_INFO) << "L4S: OnNetworkRouteChange called";
  
  // Reset ECN support detection on network change
  ecn_supported_ = false;
  ecn_capable_network_ = false;
  ect_count_ = 0;
  ce_count_ = 0;
  last_congestion_signal_ = Timestamp::MinusInfinity();
  
  // Update rate constraints
  if (msg.constraints.starting_rate) {
    starting_rate_ = msg.constraints.starting_rate;
    target_rate_ = starting_rate_;
  }
  min_target_rate_ = msg.constraints.min_data_rate;
  max_target_rate_ = msg.constraints.max_data_rate;
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnProcessInterval(ProcessInterval msg) {
  NetworkControlUpdate update;
  
  // Log periodic metrics
  LogPeriodicMetrics(msg.at_time);
  
  // Handle periodic probing
  HandlePeriodicProbing(msg.at_time, &update);
  
  // Update time-based decay in Prague estimator
  prague_estimator_->OnTimeUpdate(msg.at_time);
  
  // Fuse all bandwidth estimates and update target rate
  DataRate fused_rate = FuseBandwidthEstimates(msg.at_time);
  target_rate_ = fused_rate;
  
  // Create rate update
  MaybeTriggerOnNetworkChanged(&update, msg.at_time);
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnRemoteBitrateReport(RemoteBitrateReport msg) {
  NetworkControlUpdate update;
  return update;
}

NetworkControlUpdate L4SNetworkController::OnRoundTripTimeUpdate(RoundTripTimeUpdate msg) {
  NetworkControlUpdate update;
  
  // Update Prague estimator with RTT
  prague_estimator_->UpdateFromRtt(msg.round_trip_time);
  
  // Update local RTT tracking
  if (msg.round_trip_time.IsFinite() && !msg.round_trip_time.IsZero()) {
    last_rtt_ = msg.round_trip_time;
    last_estimated_round_trip_time_ = msg.round_trip_time;
  }
  
  // Log RTT metrics
  if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->LogDelayMetrics(
        Timestamp::Millis(env_.clock().TimeInMilliseconds()),
        msg.round_trip_time, TimeDelta::PlusInfinity(), TimeDelta::Zero());
  }
  
  RTC_LOG(LS_INFO) << "L4S: RTT updated to " << msg.round_trip_time.ms() << " ms";
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnSentPacket(SentPacket msg) {
  NetworkControlUpdate update;
  return update;
}

NetworkControlUpdate L4SNetworkController::OnReceivedPacket(ReceivedPacket msg) {
  NetworkControlUpdate update;
  return update;
}

NetworkControlUpdate L4SNetworkController::OnStreamsConfig(StreamsConfig msg) {
  NetworkControlUpdate update;
  return update;
}

NetworkControlUpdate L4SNetworkController::OnTargetRateConstraints(TargetRateConstraints msg) {
  NetworkControlUpdate update;
  
  // Update constraints
  min_target_rate_ = msg.min_data_rate;
  max_target_rate_ = msg.max_data_rate;
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnTransportLossReport(TransportLossReport msg) {
  NetworkControlUpdate update;
  
  if (msg.packets_lost_delta > 0) {
    RTC_LOG(LS_INFO) << "L4S: Transport loss report - "
                     << "Lost: " << msg.packets_lost_delta
                     << ", Received: " << msg.packets_received_delta;
    
    DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
    prague_estimator_->OnPacketLoss(current_rate, msg.receive_time);
  }
  
  // Update loss metrics
  int total_packets = msg.packets_lost_delta + msg.packets_received_delta;
  if (total_packets > 0) {
    last_loss_fraction_ = static_cast<double>(msg.packets_lost_delta) / total_packets;
  } else {
    last_loss_fraction_ = 0.0;
  }
  last_packets_lost_ = static_cast<int>(msg.packets_lost_delta);
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnTransportPacketsFeedback(TransportPacketsFeedback msg) {
  NetworkControlUpdate update;
  
  // Update all bandwidth estimators
  UpdateAllBandwidthEstimators(msg);
  
  // Handle periodic probing
  HandlePeriodicProbing(msg.feedback_time, &update);
  
  // Fuse bandwidth estimates and update target rate
  DataRate fused_rate = FuseBandwidthEstimates(msg.feedback_time);
  target_rate_ = fused_rate;
  
  // Update throughput calculation
  UpdateThroughputWindow(msg);
  
  // Create rate update
  MaybeTriggerOnNetworkChanged(&update, msg.feedback_time);
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnNetworkStateEstimate(NetworkStateEstimate msg) {
  NetworkControlUpdate update;
  return update;
}

void L4SNetworkController::UpdateAllBandwidthEstimators(const TransportPacketsFeedback& feedback) {
  // Update ALR detector first
  UpdateAlrDetector(feedback);
  
  // 1. Update Prague ECN controller (skip if application limited)
  if (!IsApplicationLimited()) {
    ProcessEcnFeedback(feedback);
  } else {
    RTC_LOG(LS_VERBOSE) << "L4S: Skipping ECN processing during ALR period";
  }
  
  // 2. Update DelayBasedBwe
  if (delay_estimator_) {
    UpdateDelayBasedEstimator(feedback);
  }
  
  // 3. Update AcknowledgedBitrateEstimator
  if (acked_estimator_) {
    UpdateAckedBitrateEstimator(feedback);
  }
  
  // 4. Process probe results (more aggressive during ALR)
  if (probe_controller_) {
    ProcessProbeResults(feedback);
  }
}

void L4SNetworkController::ProcessEcnFeedback(const TransportPacketsFeedback& feedback) {
  if (feedback.packet_feedbacks.empty()) {
    return;
  }
  
  int new_ect_count = 0;
  int new_ce_count = 0;
  
  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1 || packet.ecn == EcnMarking::kCe) {
      new_ect_count++;
    }
    if (packet.ecn == EcnMarking::kCe) {
      new_ce_count++;
      last_congestion_signal_ = feedback.feedback_time;
      
      RTC_LOG(LS_WARNING) << "L4S: CE mark detected! Count=" << new_ce_count
                         << ", ECT count=" << new_ect_count;
    }
  }
  
  // Update ECN support detection
  if (new_ect_count > 0 || new_ce_count > 0) {
    ecn_supported_ = true;
    ecn_capable_network_ = true;
  }
  
  // Update Prague estimator with CE ratio
  if (new_ect_count + new_ce_count > 0) {
    double ce_ratio = static_cast<double>(new_ce_count) / (new_ect_count + new_ce_count);
    DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
    prague_estimator_->UpdateFromCongestionSignal(current_rate, ce_ratio, feedback.feedback_time);
    
    // Update fusion engine with ECN estimate
    double ecn_confidence = prague_estimator_->GetConfidence(feedback.feedback_time);
    
    // Boost ECN confidence when Prague is in reduction mode to prevent other estimators from overriding
    if (prague_estimator_->GetDirectionFlag() == -1) {
      ecn_confidence = std::max(ecn_confidence, 0.95);  // Very high confidence during reduction
      RTC_LOG(LS_VERBOSE) << "L4S: Prague in reduction mode, boosting ECN confidence to " << ecn_confidence;
    }
    
    bandwidth_fusion_->UpdateEcnEstimate(prague_estimator_->GetCurrentEstimate(), ecn_confidence, feedback.feedback_time);
    
    // Log congestion metrics
    if (metrics_enabled_ && metrics_collector_) {
      metrics_collector_->LogCongestionMetrics(feedback.feedback_time, new_ce_count, new_ect_count, ce_ratio);
    }
  }
  
  // Update counters
  ect_count_ = new_ect_count;
  ce_count_ = new_ce_count;
}

void L4SNetworkController::UpdateDelayBasedEstimator(const TransportPacketsFeedback& feedback) {
  // Extract delay information and update delay-based estimator
  // This is a simplified implementation - in practice, you'd need to properly
  // convert the feedback format for DelayBasedBwe
  
  if (last_rtt_.IsFinite()) {
    // Update delay estimator with RTT information
    // Note: This is a placeholder - actual implementation would need proper adaptation
    DataRate delay_estimate = DataRate::KilobitsPerSec(1000);  // Placeholder
    double delay_confidence = CalculateDelayConfidence(feedback.feedback_time);
    bandwidth_fusion_->UpdateDelayEstimate(delay_estimate, delay_confidence, feedback.feedback_time);
  }
}

void L4SNetworkController::UpdateAckedBitrateEstimator(const TransportPacketsFeedback& feedback) {
  // Update acknowledged bitrate estimator
  // This is a simplified implementation - actual implementation would need
  // proper packet acknowledgment processing
  
  if (!feedback.packet_feedbacks.empty()) {
    // Calculate acknowledged bitrate from feedback
    DataRate acked_estimate = last_actual_bitrate_;  // Use calculated throughput as proxy
    double acked_confidence = CalculateAckedConfidence(feedback.feedback_time);
    bandwidth_fusion_->UpdateAckedEstimate(acked_estimate, acked_confidence, feedback.feedback_time);
  }
}

void L4SNetworkController::ProcessProbeResults(const TransportPacketsFeedback& feedback) {
  // Process probe results and update fusion engine
  // This is a placeholder - actual implementation would need to detect and process probe clusters
  
  TimeDelta since_probe = feedback.feedback_time - last_probe_time_;
  if (since_probe < TimeDelta::Seconds(2) && last_probe_estimate_ > DataRate::Zero()) {
    double probe_confidence = CalculateProbeConfidence(feedback.feedback_time);
    bandwidth_fusion_->UpdateProbeEstimate(last_probe_estimate_, probe_confidence, feedback.feedback_time);
  }
}

void L4SNetworkController::HandlePeriodicProbing(Timestamp now, NetworkControlUpdate* update) {
  if (!config_.enable_probing || !probe_controller_) {
    return;
  }
  
  // Check if it's time for periodic probing
  bool should_probe = (now - last_probe_time_) >= config_.probe_interval;
  should_probe = should_probe && ShouldProbeNow(now);
  
  if (should_probe) {
    InitiateProbing(now, update);
    last_probe_time_ = now;
  }
}

bool L4SNetworkController::ShouldProbeNow(Timestamp now) const {
  // Don't probe if we're experiencing heavy congestion
  if (HasRecentCongestionSignals(now)) {
    return false;
  }
  
  // Don't probe if ECN feedback is very fresh and confident
  if (IsEcnFeedbackFresh(now) && prague_estimator_->GetConfidence(now) > 0.9) {
    return false;
  }
  
  // Don't probe during high loss periods
  if (last_loss_fraction_ > 0.02) {  // 2% loss threshold
    return false;
  }
  
  return true;
}

void L4SNetworkController::InitiateProbing(Timestamp now, NetworkControlUpdate* update) {
  // Get current best estimate for probe rate calculation
  DataRate current_estimate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  
  // Be more aggressive during ALR periods
  double probe_multiplier = 1.5;  // Default
  if (IsApplicationLimited()) {
    probe_multiplier = 2.0;  // More aggressive when application limited
    RTC_LOG(LS_VERBOSE) << "L4S: Using aggressive probing during ALR period";
  }
  
  // Request probe at multiplier x current estimate
  DataRate probe_rate = current_estimate * probe_multiplier;
  if (max_target_rate_) {
    probe_rate = std::min(probe_rate, *max_target_rate_);
  }
  
  // Store probe estimate for later processing
  last_probe_estimate_ = probe_rate;
  
  RTC_LOG(LS_INFO) << "L4S: Initiating probe at " << probe_rate.bps() 
                   << " bps (multiplier: " << probe_multiplier << ")";
  
  // Note: Actual probe cluster creation would be done here with probe_controller_
  // This is a simplified implementation
}

double L4SNetworkController::CalculateEcnConfidence(Timestamp now) const {
  return prague_estimator_->GetConfidence(now);
}

double L4SNetworkController::CalculateDelayConfidence(Timestamp now) const {
  if (!delay_estimator_ || !last_rtt_.IsFinite()) {
    return 0.0;
  }
  
  // High confidence if RTT is stable
  if (IsRttStable()) {
    return 0.8;
  }
  return 0.5;
}

double L4SNetworkController::CalculateProbeConfidence(Timestamp now) const {
  TimeDelta since_probe = now - last_probe_time_;
  if (since_probe < TimeDelta::Seconds(1)) {
    return 0.95;  // Very high confidence in fresh probe results
  } else if (since_probe < TimeDelta::Seconds(10)) {
    return 0.8;   // Good confidence in recent probes
  }
  return 0.2;   // Low confidence in old probe results
}

double L4SNetworkController::CalculateAckedConfidence(Timestamp now) const {
  if (!acked_estimator_) {
    return 0.0;
  }
  
  // Moderate confidence in acknowledged bitrate
  return 0.6;
}

DataRate L4SNetworkController::FuseBandwidthEstimates(Timestamp now) {
  DataRate fused_rate = bandwidth_fusion_->GetFusedEstimate(now);
  
  // Apply rate constraints
  if (min_target_rate_ && fused_rate < *min_target_rate_) {
    fused_rate = *min_target_rate_;
  }
  if (max_target_rate_ && fused_rate > *max_target_rate_) {
    fused_rate = *max_target_rate_;
  }
  
  // Log fusion metrics
  if (metrics_enabled_ && metrics_collector_) {
    auto sources = bandwidth_fusion_->GetCurrentSources();
    metrics_collector_->LogFusionMetrics(now, sources, fused_rate);
  }
  
  return fused_rate;
}

NetworkControlUpdate L4SNetworkController::CreateRateUpdate(Timestamp at_time) const {
  NetworkControlUpdate update;
  
  if (!at_time.IsFinite()) {
    at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }
  
  DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  
  // Validate rate
  if (!current_rate.IsFinite() || current_rate.bps() <= 0) {
    current_rate = DataRate::KilobitsPerSec(300);
  }
  
  // Set target rate
  update.target_rate = TargetTransferRate();
  update.target_rate->at_time = at_time;
  update.target_rate->network_estimate.at_time = at_time;
  update.target_rate->network_estimate.bandwidth = current_rate;
  update.target_rate->network_estimate.loss_rate_ratio = static_cast<float>(last_loss_fraction_);
  update.target_rate->network_estimate.round_trip_time = last_estimated_round_trip_time_;
  update.target_rate->network_estimate.bwe_period = TimeDelta::Millis(500);
  update.target_rate->target_rate = current_rate;
  
  // Set pacer config
  update.pacer_config = PacerConfig();
  update.pacer_config->at_time = at_time;
  update.pacer_config->time_window = TimeDelta::Millis(10);
  update.pacer_config->data_window = current_rate * update.pacer_config->time_window;
  update.pacer_config->pad_window = DataSize::Zero();
  
  return update;
}

void L4SNetworkController::MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update, Timestamp at_time) {
  if (!at_time.IsFinite()) {
    at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }
  
  NetworkControlUpdate rate_update = CreateRateUpdate(at_time);
  
  if (rate_update.pacer_config) {
    update->pacer_config = rate_update.pacer_config;
  }
  if (rate_update.target_rate) {
    update->target_rate = rate_update.target_rate;
  }
}

bool L4SNetworkController::IsL4SActive() const {
  return ecn_supported_ && ecn_capable_network_;
}

bool L4SNetworkController::HasRecentCongestionSignals(Timestamp now) const {
  return !last_congestion_signal_.IsInfinite() && 
         (now - last_congestion_signal_) < TimeDelta::Seconds(2);
}

bool L4SNetworkController::IsEcnFeedbackFresh(Timestamp now) const {
  return HasRecentCongestionSignals(now) || 
         (ecn_supported_ && (now - last_congestion_signal_) < TimeDelta::Seconds(5));
}

bool L4SNetworkController::EstimatesAreDiverging() const {
  // Simple check for estimate divergence
  auto sources = bandwidth_fusion_->GetCurrentSources();
  if (sources.ecn_estimate > DataRate::Zero() && sources.delay_estimate > DataRate::Zero()) {
    double ratio = sources.ecn_estimate.bps() / static_cast<double>(sources.delay_estimate.bps());
    return ratio > 2.0 || ratio < 0.5;  // 2x divergence threshold
  }
  return false;
}

bool L4SNetworkController::IsRttStable() const {
  // Simplified RTT stability check
  return last_rtt_.IsFinite() && last_rtt_ < TimeDelta::Millis(100);
}

void L4SNetworkController::UpdateThroughputWindow(const TransportPacketsFeedback& feedback) {
  constexpr TimeDelta kThroughputWindow = TimeDelta::Millis(500);
  Timestamp now = feedback.feedback_time;
  
  // Add new packets to the window
  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.receive_time.IsFinite() && packet.sent_packet.send_time.IsFinite()) {
      throughput_window_.emplace_back(packet.receive_time, packet.sent_packet.size.bytes());
    }
  }
  
  // Remove old packets outside the window
  while (!throughput_window_.empty() && now - throughput_window_.front().first > kThroughputWindow) {
    throughput_window_.pop_front();
  }
  
  // Calculate throughput over the window
  int64_t window_bytes = 0;
  if (!throughput_window_.empty()) {
    Timestamp window_start = throughput_window_.front().first;
    Timestamp window_end = throughput_window_.back().first;
    for (const auto& entry : throughput_window_) {
      window_bytes += entry.second;
    }
    
    TimeDelta window_interval = window_end - window_start;
    if (window_interval > TimeDelta::Millis(1)) {
      last_actual_bitrate_ = DataRate::BitsPerSec(
          static_cast<int64_t>((window_bytes * 8) / window_interval.seconds<double>()));
    } else {
      last_actual_bitrate_ = DataRate::Zero();
    }
  }
}

void L4SNetworkController::LogPeriodicMetrics(Timestamp at_time) {
  if (!metrics_enabled_ || !metrics_collector_) {
    return;
  }
  
  if (at_time - metrics_last_logged_ < kMetricsLoggingInterval) {
    return;
  }
  
  metrics_last_logged_ = at_time;
  
  // Log bandwidth metrics
  DataRate target_rate = target_rate_.value_or(DataRate::Zero());
  metrics_collector_->LogBandwidthMetrics(at_time, target_rate, last_actual_bitrate_);
  
  // Log delay metrics
  if (last_rtt_.IsFinite()) {
    metrics_collector_->LogDelayMetrics(at_time, last_rtt_, last_rtt_ / 2, TimeDelta::Zero());
  }
  
  // Log loss metrics
  metrics_collector_->LogLossMetrics(at_time, last_loss_fraction_, last_packets_lost_);
  
  // Log periodic summary
  metrics_collector_->LogPeriodicSummary(at_time);
}

bool L4SNetworkController::IsApplicationLimited() const {
  if (!alr_detector_) {
    return false;
  }
  return alr_detector_->GetApplicationLimitedRegionStartTime().has_value();
}

void L4SNetworkController::UpdateAlrDetector(const TransportPacketsFeedback& feedback) {
  if (!alr_detector_) {
    return;
  }
  
  // Calculate approximate bytes sent from feedback
  size_t bytes_sent = 0;
  for (const auto& packet : feedback.PacketsWithFeedback()) {
    (void)packet;  // Mark as used to avoid warning
    // Use a reasonable estimate if packet size isn't available
    bytes_sent += 1200;  // Typical packet size
  }
  
  alr_detector_->OnBytesSent(bytes_sent, feedback.feedback_time.ms());
}

}  // namespace webrtc
