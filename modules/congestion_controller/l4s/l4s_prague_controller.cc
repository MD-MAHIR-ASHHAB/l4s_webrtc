// #include "modules/congestion_controller/l4s/l4s_prague_controller.h"

// #include <algorithm>
// #include <memory>
// #include <numeric>
// #include <utility>

// #include "absl/strings/match.h"
// #include "api/field_trials_view.h"
// #include "api/rtc_event_log/rtc_event_log.h"
// #include "api/transport/bandwidth_usage.h"
// #include "api/transport/network_types.h"
// #include "api/units/data_rate.h"
// #include "api/units/data_size.h"
// #include "api/units/time_delta.h"
// #include "api/units/timestamp.h"
// #include "logging/rtc_event_log/events/rtc_event_probe_cluster_created.h"
// #include "rtc_base/checks.h"
// #include "rtc_base/experiments/field_trial_parser.h"
// #include "rtc_base/logging.h"
// #include "system_wrappers/include/metrics.h"

// // Metrics collection
// #include "api/test/metrics/global_metrics_logger_and_exporter.h"
// #include "api/test/metrics/metrics_logger.h"

// namespace webrtc {

// // =============================================================================
// // PragueCapacityEstimator Implementation
// // =============================================================================

// PragueCapacityEstimator::PragueCapacityEstimator(DataRate starting_rate, DataRate min_rate, DataRate max_rate)
//     : congestion_based_estimate_(starting_rate),
//       min_target_rate_(min_rate),
//       max_target_rate_(max_rate),
//       current_rtt_(TimeDelta::Millis(50)),
//       last_update_time_(Timestamp::MinusInfinity()),
//       last_congestion_signal_(Timestamp::MinusInfinity()) {
  
//   // Clamp initial estimate to bounds
//   if (congestion_based_estimate_ < min_target_rate_) {
//     congestion_based_estimate_ = min_target_rate_;
//   }
//   if (congestion_based_estimate_ > max_target_rate_) {
//     congestion_based_estimate_ = max_target_rate_;
//   }
// }

// PragueCapacityEstimator::~PragueCapacityEstimator() = default;

// void PragueCapacityEstimator::UpdateFromCongestionSignal(DataRate current_rate, double ce_ratio, Timestamp current_time) {
//   constexpr int kDefaultMssBytes = 1440;  // Typical Ethernet MSS
  
//   TimeDelta rtt = current_rtt_.IsFinite() && !current_rtt_.IsZero() ? current_rtt_ : TimeDelta::Millis(50);
//   double rtt_seconds = rtt.seconds<double>();
//   if (rtt_seconds <= 0.0) {
//     rtt_seconds = 0.001;  // Fallback RTT (1ms for VM testbed)
//   }

//   // Pure Prague DCTCP-style rate adaptation (RFC 9330)
//   if (ce_ratio > 0.0) {  // Prague: Respond to ANY CE marking (no threshold)
//     // DCTCP-style alpha update with standard EWMA gain
//     constexpr double g = 1.0 / 16.0;  // RFC 9330 standard gain
//     alpha_ = (1.0 - g) * alpha_ + g * ce_ratio;
    
//     // Proportional decrease (much gentler than 50% reduction)
//     double reduction_factor = 1.0 - alpha_ / 2.0;
//     DataRate reduced = std::max(current_rate * reduction_factor, min_target_rate_);
//     congestion_based_estimate_ = reduced;
    
//     // Update congestion signal timestamp
//     last_congestion_signal_ = current_time;
    
//     RTC_LOG(LS_INFO) << "Prague: Proportional decrease (alpha=" << alpha_
//                      << ", ce_ratio=" << ce_ratio 
//                      << ", reduction_factor=" << reduction_factor
//                      << "), new rate=" << congestion_based_estimate_.bps() << " bps";
                     
//   } else if (current_rate >= congestion_based_estimate_ * 0.9) {
//     // More aggressive additive increase since decreases are gentler
//     int64_t bits_per_rtt = kDefaultMssBytes * 8;
//     double ai_factor = 1.0;  // 1.0 MSS per RTT
//     int64_t increase_bps = rtt_seconds > 0 ? static_cast<int64_t>((bits_per_rtt * ai_factor) / rtt_seconds) : 0;
//     DataRate increased = std::min(congestion_based_estimate_ + DataRate::BitsPerSec(increase_bps), max_target_rate_);
    
//     congestion_based_estimate_ = increased;
    
//     RTC_LOG(LS_INFO) << "Prague: Additive increase (+1.0 MSS/RTT, rtt=" << rtt.ms() << " ms), "
//                      << "new rate=" << congestion_based_estimate_.bps() << " bps";
//   }
  
//   last_update_time_ = current_time;
// }

// void PragueCapacityEstimator::UpdateFromRtt(TimeDelta rtt) {
//   if (rtt.IsFinite() && !rtt.IsZero()) {
//     current_rtt_ = rtt;
//   }
// }

// void PragueCapacityEstimator::OnPacketLoss(DataRate current_rate, Timestamp current_time) {
//   // Multiplicative decrease for packet loss (fallback mechanism)
//   DataRate reduced = std::max(current_rate * 0.5, min_target_rate_);
//   congestion_based_estimate_ = reduced;
  
//   RTC_LOG(LS_WARNING) << "Prague: Packet loss detected, halving estimate to "
//                       << congestion_based_estimate_.bps() << " bps";
  
//   last_update_time_ = current_time;
// }

// void PragueCapacityEstimator::OnTimeUpdate(Timestamp current_time) {
//   if (last_update_time_.IsInfinite()) {
//     last_update_time_ = current_time;
//     return;
//   }
  
//   TimeDelta elapsed = current_time - last_update_time_;
//   if (elapsed >= kDecayInterval) {
//     // Gradually decay estimates if not reinforced
//     congestion_based_estimate_ = std::max(congestion_based_estimate_ * 0.95, min_target_rate_);
//     last_update_time_ = current_time;
//   }
// }

// DataRate PragueCapacityEstimator::GetCurrentEstimate() const {
//   return congestion_based_estimate_;
// }

// double PragueCapacityEstimator::GetConfidence(Timestamp now) const {
//   if (last_congestion_signal_.IsInfinite()) {
//     return 0.3;  // Low confidence without any ECN feedback
//   }
  
//   TimeDelta since_signal = now - last_congestion_signal_;
//   if (since_signal < TimeDelta::Seconds(2)) {
//     return 0.9;  // Very confident with recent ECN feedback
//   } else if (since_signal < TimeDelta::Seconds(5)) {
//     return 0.7;  // Moderately confident
//   }
//   return 0.4;  // Lower confidence with stale ECN feedback
// }

// // =============================================================================
// // L4SBandwidthFusion Implementation
// // =============================================================================

// L4SBandwidthFusion::L4SBandwidthFusion(const L4SPragueConfig& config) : config_(config) {}

// L4SBandwidthFusion::~L4SBandwidthFusion() = default;

// void L4SBandwidthFusion::UpdateEcnEstimate(DataRate estimate, double confidence, Timestamp now) {
//   sources_.ecn_estimate = estimate;
//   sources_.ecn_confidence = confidence;
//   sources_.last_ecn_update = now;
// }

// void L4SBandwidthFusion::UpdateDelayEstimate(DataRate estimate, double confidence, Timestamp now) {
//   sources_.delay_estimate = estimate;
//   sources_.delay_confidence = confidence;
//   sources_.last_delay_update = now;
// }

// void L4SBandwidthFusion::UpdateProbeEstimate(DataRate estimate, double confidence, Timestamp now) {
//   sources_.probe_estimate = estimate;
//   sources_.probe_confidence = confidence;
//   sources_.last_probe_update = now;
// }

// void L4SBandwidthFusion::UpdateAckedEstimate(DataRate estimate, double confidence, Timestamp now) {
//   sources_.acked_estimate = estimate;
//   sources_.acked_confidence = confidence;
//   sources_.last_acked_update = now;
// }

// DataRate L4SBandwidthFusion::GetFusedEstimate(Timestamp now) const {
//   // Priority-based fusion with confidence weighting
  
//   // 1. If ECN feedback is fresh and confident, prioritize it
//   if (sources_.ecn_confidence > config_.ecn_confidence_threshold && 
//       IsRecentlyUpdated(sources_.last_ecn_update, now)) {
//     return sources_.ecn_estimate;
//   }
  
//   // 2. If we have fresh probe results, they're usually most accurate
//   if (sources_.probe_confidence > config_.probe_confidence_threshold && 
//       IsRecentlyUpdated(sources_.last_probe_update, now)) {
//     // Cross-validate with other estimates
//     return ValidateWithOtherSources(sources_.probe_estimate, sources_);
//   }
  
//   // 3. Weighted combination of delay and acked estimates
//   double total_weight = sources_.delay_confidence + sources_.acked_confidence;
//   if (total_weight > 0.1) {
//     DataRate weighted_estimate = 
//         (sources_.delay_estimate * sources_.delay_confidence + 
//          sources_.acked_estimate * sources_.acked_confidence) / total_weight;
    
//     // Apply ECN constraints if available
//     if (sources_.ecn_confidence > 0.3) {
//       weighted_estimate = std::min(weighted_estimate, sources_.ecn_estimate * 1.2);
//     }
    
//     return weighted_estimate;
//   }
  
//   // 4. Fallback to most confident single estimate
//   return GetMostConfidentEstimate(now);
// }

// DataRate L4SBandwidthFusion::GetMostConfidentEstimate(Timestamp now) const {
//   DataRate best_estimate = DataRate::KilobitsPerSec(300);  // Fallback
//   double best_confidence = 0.0;
  
//   if (sources_.ecn_confidence > best_confidence && IsRecentlyUpdated(sources_.last_ecn_update, now)) {
//     best_estimate = sources_.ecn_estimate;
//     best_confidence = sources_.ecn_confidence;
//   }
  
//   if (sources_.delay_confidence > best_confidence && IsRecentlyUpdated(sources_.last_delay_update, now)) {
//     best_estimate = sources_.delay_estimate;
//     best_confidence = sources_.delay_confidence;
//   }
  
//   if (sources_.probe_confidence > best_confidence && IsRecentlyUpdated(sources_.last_probe_update, now)) {
//     best_estimate = sources_.probe_estimate;
//     best_confidence = sources_.probe_confidence;
//   }
  
//   if (sources_.acked_confidence > best_confidence && IsRecentlyUpdated(sources_.last_acked_update, now)) {
//     best_estimate = sources_.acked_estimate;
//     best_confidence = sources_.acked_confidence;
//   }
  
//   return best_estimate;
// }

// DataRate L4SBandwidthFusion::ValidateWithOtherSources(DataRate primary_estimate, const BandwidthSources& sources) const {
//   // Don't allow probe results that are dramatically higher than other estimates
//   DataRate max_alternative = DataRate::Zero();
  
//   if (sources.ecn_confidence > 0.3) {
//     max_alternative = std::max(max_alternative, sources.ecn_estimate);
//   }
//   if (sources.delay_confidence > 0.3) {
//     max_alternative = std::max(max_alternative, sources.delay_estimate);
//   }
//   if (sources.acked_confidence > 0.3) {
//     max_alternative = std::max(max_alternative, sources.acked_estimate);
//   }
  
//   if (max_alternative > DataRate::Zero() && primary_estimate > max_alternative * 2.0) {
//     // Probe result seems too optimistic, cap it
//     return max_alternative * 1.5;
//   }
  
//   return primary_estimate;
// }

// bool L4SBandwidthFusion::IsRecentlyUpdated(Timestamp last_update, Timestamp now) const {
//   return !last_update.IsInfinite() && (now - last_update) < TimeDelta::Seconds(10);
// }

// // =============================================================================
// // L4SPragueMetricsCollector Implementation
// // =============================================================================

// L4SPragueMetricsCollector::L4SPragueMetricsCollector(test::MetricsLogger* logger, 
//                                                     const std::string& test_case_name,
//                                                     Clock* clock)
//     : logger_(logger), test_case_name_(test_case_name), clock_(clock) {
//   RTC_CHECK(logger_);
//   RTC_CHECK(clock_);
//   RTC_LOG(LS_INFO) << "L4SPragueMetricsCollector initialized for test case: " << test_case_name_;
// }

// L4SPragueMetricsCollector::~L4SPragueMetricsCollector() = default;

// void L4SPragueMetricsCollector::LogBandwidthMetrics(Timestamp at_time, DataRate target_bitrate, DataRate actual_bitrate) {
//   if (at_time - last_bandwidth_log_ < kBandwidthLogInterval) {
//     return;
//   }
  
//   last_bandwidth_log_ = at_time;
//   UpdateThroughputStats(actual_bitrate);
  
//   logger_->LogSingleValueMetric("bandwidth_target_mbps", test_case_name_, target_bitrate.bps() / 1e6, 
//                                 webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
  
//   logger_->LogSingleValueMetric("bandwidth_actual_mbps", test_case_name_, actual_bitrate.bps() / 1e6, 
//                                 webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
// }

// void L4SPragueMetricsCollector::LogDelayMetrics(Timestamp at_time, TimeDelta rtt, TimeDelta one_way_delay, TimeDelta jitter) {
//   if (at_time - last_delay_log_ < kDelayLogInterval) {
//     return;
//   }
  
//   last_delay_log_ = at_time;
//   UpdateDelayStats(rtt);
  
//   logger_->LogSingleValueMetric("rtt_ms", test_case_name_, rtt.ms(), 
//                                 webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
  
//   if (one_way_delay.IsFinite()) {
//     logger_->LogSingleValueMetric("one_way_delay_ms", test_case_name_, one_way_delay.ms(), 
//                                   webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
//                                   {{"timestamp_ms", std::to_string(at_time.ms())}});
//   }
// }

// void L4SPragueMetricsCollector::LogLossMetrics(Timestamp at_time, double loss_fraction, int packets_lost) {
//   if (at_time - last_loss_log_ < kLossLogInterval) {
//     return;
//   }
  
//   last_loss_log_ = at_time;
//   UpdateLossStats(loss_fraction);
  
//   logger_->LogSingleValueMetric("packets_lost_count", test_case_name_, packets_lost, 
//                                 webrtc::test::Unit::kCount, webrtc::test::ImprovementDirection::kSmallerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
// }

// void L4SPragueMetricsCollector::LogCongestionMetrics(Timestamp at_time, int ce_count, int ect_count, double congestion_ratio) {
//   logger_->LogSingleValueMetric("congestion_ce_count", test_case_name_, ce_count, 
//                                 webrtc::test::Unit::kCount, webrtc::test::ImprovementDirection::kSmallerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
  
//   logger_->LogSingleValueMetric("congestion_ratio", test_case_name_, congestion_ratio, 
//                                 webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kSmallerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
// }

// void L4SPragueMetricsCollector::LogFusionMetrics(Timestamp at_time, const L4SBandwidthFusion::BandwidthSources& sources, DataRate fused_rate) {
//   logger_->LogSingleValueMetric("fusion_ecn_estimate_mbps", test_case_name_, sources.ecn_estimate.bps() / 1e6, 
//                                 webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
  
//   logger_->LogSingleValueMetric("fusion_delay_estimate_mbps", test_case_name_, sources.delay_estimate.bps() / 1e6, 
//                                 webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
  
//   logger_->LogSingleValueMetric("fusion_probe_estimate_mbps", test_case_name_, sources.probe_estimate.bps() / 1e6, 
//                                 webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
  
//   logger_->LogSingleValueMetric("fusion_fused_rate_mbps", test_case_name_, fused_rate.bps() / 1e6, 
//                                 webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
// }

// void L4SPragueMetricsCollector::LogPeriodicSummary(Timestamp at_time) {
//   if (at_time - last_summary_log_ < kSummaryLogInterval) {
//     return;
//   }
  
//   last_summary_log_ = at_time;
  
//   if (throughput_stats_.NumSamples() > 0) {
//     logger_->LogSingleValueMetric("throughput_avg_mbps", test_case_name_, throughput_stats_.GetAverage() / 1e6, 
//                                   webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                   {{"stat_type", "average"}, {"metric", "throughput"}});
//   }
  
//   if (delay_stats_.NumSamples() > 0) {
//     logger_->LogSingleValueMetric("delay_avg_ms", test_case_name_, delay_stats_.GetAverage(), 
//                                   webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
//                                   {{"stat_type", "average"}, {"metric", "delay"}});
//   }
  
//   ExportToJsonFile("l4s_prague_metrics.json");
// }

// void L4SPragueMetricsCollector::ExportToJsonFile(const std::string& filename) {
//   if (!logger_) return;
//   auto metrics = logger_->GetCollectedMetrics();
//   FILE* f = fopen(filename.c_str(), "w");
//   if (!f) return;
  
//   fprintf(f, "[\n");
//   for (size_t i = 0; i < metrics.size(); ++i) {
//     const auto& m = metrics[i];
//     fprintf(f, "  {\n");
//     fprintf(f, "    \"name\": \"%s\",\n", m.name.c_str());
//     fprintf(f, "    \"samples\": [");
//     for (size_t j = 0; j < m.time_series.samples.size(); ++j) {
//       const auto& s = m.time_series.samples[j];
//       fprintf(f, "%s{\"timestamp\": %lld, \"value\": %f}",
//         (j > 0 ? ", " : ""), static_cast<long long>(s.timestamp.us()), s.value);
//     }
//     fprintf(f, "]\n  }%s\n", (i + 1 < metrics.size()) ? "," : "");
//   }
//   fprintf(f, "]\n");
//   fclose(f);
// }

// void L4SPragueMetricsCollector::UpdateThroughputStats(DataRate actual_bitrate) {
//   throughput_stats_.AddSample(actual_bitrate.bps());
// }

// void L4SPragueMetricsCollector::UpdateDelayStats(TimeDelta rtt) {
//   if (rtt.IsFinite()) {
//     delay_stats_.AddSample(rtt.ms());
//   }
// }

// void L4SPragueMetricsCollector::UpdateLossStats(double loss_fraction) {
//   loss_stats_.AddSample(loss_fraction);
// }

// // =============================================================================
// // L4SPragueNetworkController Implementation
// // =============================================================================

// L4SPragueNetworkController::L4SPragueNetworkController(NetworkControllerConfig config,
//                                                       L4SPragueConfig l4s_config,
//                                                       test::MetricsLogger* metrics_logger)
//     : env_(config.env), config_(l4s_config) {
  
//   // Initialize Prague capacity estimator
//   DataRate starting_rate = config.constraints.starting_rate.value_or(DataRate::KilobitsPerSec(300));
//   DataRate min_rate = config.constraints.min_data_rate.value_or(DataRate::KilobitsPerSec(30));
//   DataRate max_rate = config.constraints.max_data_rate.value_or(DataRate::KilobitsPerSec(100000));
  
//   prague_estimator_ = std::make_unique<PragueCapacityEstimator>(starting_rate, min_rate, max_rate);
  
//   // Initialize bandwidth fusion engine
//   bandwidth_fusion_ = std::make_unique<L4SBandwidthFusion>(config_);
  
//   // Initialize bandwidth estimation components
//   InitializeBandwidthEstimators();
  
//   // Initialize metrics collector
//   if (config_.enable_metrics_collection) {
//     using webrtc::test::GetGlobalMetricsLogger;
//     test::MetricsLogger* logger_to_use = metrics_logger;
//     if (!logger_to_use) {
//       logger_to_use = GetGlobalMetricsLogger();
//     }
//     metrics_collector_ = std::make_unique<L4SPragueMetricsCollector>(
//         logger_to_use, config_.test_case_name, &env_.clock());
//   }
  
//   // Set initial rate constraints
//   starting_rate_ = config.constraints.starting_rate;
//   min_target_rate_ = config.constraints.min_data_rate;
//   max_target_rate_ = config.constraints.max_data_rate;
//   target_rate_ = starting_rate;
  
//   RTC_LOG(LS_INFO) << "L4SPragueNetworkController created with starting rate: " 
//                    << starting_rate.bps() << " bps";
// }

// L4SPragueNetworkController::~L4SPragueNetworkController() {
//   if (metrics_enabled_ && metrics_collector_) {
//     metrics_collector_->ExportToJsonFile("l4s_prague_metrics.json");
//     RTC_LOG(LS_INFO) << "L4S Prague: Exported metrics to l4s_prague_metrics.json";
//   }
// }

// void L4SPragueNetworkController::InitializeBandwidthEstimators() {
//   if (config_.enable_delay_estimation) {
//     delay_estimator_ = std::make_unique<DelayBasedBwe>(&env_.field_trials(), nullptr, nullptr);
//   }
  
//   if (config_.enable_probing) {
//     probe_controller_ = std::make_unique<ProbeController>(&env_.field_trials(), nullptr);
//   }
  
//   if (config_.enable_acked_estimation) {
//     acked_estimator_ = std::make_unique<AcknowledgedBitrateEstimator>(&env_.field_trials());
//   }
  
//   RTC_LOG(LS_INFO) << "L4S Prague: Initialized bandwidth estimators - "
//                    << "Delay: " << (delay_estimator_ ? "enabled" : "disabled")
//                    << ", Probe: " << (probe_controller_ ? "enabled" : "disabled")
//                    << ", Acked: " << (acked_estimator_ ? "enabled" : "disabled");
// }

// NetworkControlUpdate L4SPragueNetworkController::OnNetworkAvailability(NetworkAvailability msg) {
//   NetworkControlUpdate update;
//   return update;
// }

// NetworkControlUpdate L4SPragueNetworkController::OnNetworkRouteChange(NetworkRouteChange msg) {
//   NetworkControlUpdate update;
  
//   RTC_LOG(LS_INFO) << "L4S Prague: OnNetworkRouteChange called";
  
//   // Reset ECN support detection on network change
//   ecn_supported_ = false;
//   ecn_capable_network_ = false;
//   ect_count_ = 0;
//   ce_count_ = 0;
//   last_congestion_signal_ = Timestamp::MinusInfinity();
  
//   // Update rate constraints
//   if (msg.constraints.starting_rate) {
//     starting_rate_ = msg.constraints.starting_rate;
//     target_rate_ = starting_rate_;
//   }
//   min_target_rate_ = msg.constraints.min_data_rate;
//   max_target_rate_ = msg.constraints.max_data_rate;
  
//   return update;
// }

// NetworkControlUpdate L4SPragueNetworkController::OnProcessInterval(ProcessInterval msg) {
//   NetworkControlUpdate update;
  
//   // Log periodic metrics
//   LogPeriodicMetrics(msg.at_time);
  
//   // Handle periodic probing
//   HandlePeriodicProbing(msg.at_time, &update);
  
//   // Update time-based decay in Prague estimator
//   prague_estimator_->OnTimeUpdate(msg.at_time);
  
//   // Fuse all bandwidth estimates and update target rate
//   DataRate fused_rate = FuseBandwidthEstimates(msg.at_time);
//   target_rate_ = fused_rate;
  
//   // Create rate update
//   MaybeTriggerOnNetworkChanged(&update, msg.at_time);
  
//   return update;
// }

// NetworkControlUpdate L4SPragueNetworkController::OnRemoteBitrateReport(RemoteBitrateReport msg) {
//   NetworkControlUpdate update;
//   return update;
// }

// NetworkControlUpdate L4SPragueNetworkController::OnRoundTripTimeUpdate(RoundTripTimeUpdate msg) {
//   NetworkControlUpdate update;
  
//   // Update Prague estimator with RTT
//   prague_estimator_->UpdateFromRtt(msg.round_trip_time);
  
//   // Update local RTT tracking
//   if (msg.round_trip_time.IsFinite() && !msg.round_trip_time.IsZero()) {
//     last_rtt_ = msg.round_trip_time;
//     last_estimated_round_trip_time_ = msg.round_trip_time;
//   }
  
//   // Log RTT metrics
//   if (metrics_enabled_ && metrics_collector_) {
//     metrics_collector_->LogDelayMetrics(
//         Timestamp::Millis(env_.clock().TimeInMilliseconds()),
//         msg.round_trip_time, TimeDelta::PlusInfinity(), TimeDelta::Zero());
//   }
  
//   RTC_LOG(LS_INFO) << "L4S Prague: RTT updated to " << msg.round_trip_time.ms() << " ms";
  
//   return update;
// }

// NetworkControlUpdate L4SPragueNetworkController::OnSentPacket(SentPacket msg) {
//   NetworkControlUpdate update;
//   return update;
// }

// NetworkControlUpdate L4SPragueNetworkController::OnReceivedPacket(ReceivedPacket msg) {
//   NetworkControlUpdate update;
//   return update;
// }

// NetworkControlUpdate L4SPragueNetworkController::OnStreamsConfig(StreamsConfig msg) {
//   NetworkControlUpdate update;
//   return update;
// }

// NetworkControlUpdate L4SPragueNetworkController::OnTargetRateConstraints(TargetRateConstraints msg) {
//   NetworkControlUpdate update;
  
//   // Update constraints
//   min_target_rate_ = msg.min_data_rate;
//   max_target_rate_ = msg.max_data_rate;
  
//   return update;
// }

// NetworkControlUpdate L4SPragueNetworkController::OnTransportLossReport(TransportLossReport msg) {
//   NetworkControlUpdate update;
  
//   if (msg.packets_lost_delta > 0) {
//     RTC_LOG(LS_INFO) << "L4S Prague: Transport loss report - "
//                      << "Lost: " << msg.packets_lost_delta
//                      << ", Received: " << msg.packets_received_delta;
    
//     DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
//     prague_estimator_->OnPacketLoss(current_rate, msg.receive_time);
//   }
  
//   // Update loss metrics
//   int total_packets = msg.packets_lost_delta + msg.packets_received_delta;
//   if (total_packets > 0) {
//     last_loss_fraction_ = static_cast<double>(msg.packets_lost_delta) / total_packets;
//   } else {
//     last_loss_fraction_ = 0.0;
//   }
//   last_packets_lost_ = static_cast<int>(msg.packets_lost_delta);
  
//   return update;
// }

// NetworkControlUpdate L4SPragueNetworkController::OnTransportPacketsFeedback(TransportPacketsFeedback msg) {
//   NetworkControlUpdate update;
  
//   // Update all bandwidth estimators
//   UpdateAllBandwidthEstimators(msg);
  
//   // Handle periodic probing
//   HandlePeriodicProbing(msg.feedback_time, &update);
  
//   // Fuse bandwidth estimates and update target rate
//   DataRate fused_rate = FuseBandwidthEstimates(msg.feedback_time);
//   target_rate_ = fused_rate;
  
//   // Update throughput calculation
//   UpdateThroughputWindow(msg);
  
//   // Create rate update
//   MaybeTriggerOnNetworkChanged(&update, msg.feedback_time);
  
//   return update;
// }

// NetworkControlUpdate L4SPragueNetworkController::OnNetworkStateEstimate(NetworkStateEstimate msg) {
//   NetworkControlUpdate update;
//   return update;
// }

// void L4SPragueNetworkController::UpdateAllBandwidthEstimators(const TransportPacketsFeedback& feedback) {
//   // 1. Update Prague ECN controller
//   ProcessEcnFeedback(feedback);
  
//   // 2. Update DelayBasedBwe
//   if (delay_estimator_) {
//     UpdateDelayBasedEstimator(feedback);
//   }
  
//   // 3. Update AcknowledgedBitrateEstimator
//   if (acked_estimator_) {
//     UpdateAckedBitrateEstimator(feedback);
//   }
  
//   // 4. Process probe results
//   if (probe_controller_) {
//     ProcessProbeResults(feedback);
//   }
// }

// void L4SPragueNetworkController::ProcessEcnFeedback(const TransportPacketsFeedback& feedback) {
//   if (feedback.packet_feedbacks.empty()) {
//     return;
//   }
  
//   int new_ect_count = 0;
//   int new_ce_count = 0;
  
//   for (const auto& packet : feedback.packet_feedbacks) {
//     if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1 || packet.ecn == EcnMarking::kCe) {
//       new_ect_count++;
//     }
//     if (packet.ecn == EcnMarking::kCe) {
//       new_ce_count++;
//       last_congestion_signal_ = feedback.feedback_time;
      
//       RTC_LOG(LS_WARNING) << "L4S Prague: CE mark detected! Count=" << new_ce_count
//                          << ", ECT count=" << new_ect_count;
//     }
//   }
  
//   // Update ECN support detection
//   if (new_ect_count > 0 || new_ce_count > 0) {
//     ecn_supported_ = true;
//     ecn_capable_network_ = true;
//   }
  
//   // Update Prague estimator with CE ratio
//   if (new_ect_count + new_ce_count > 0) {
//     double ce_ratio = static_cast<double>(new_ce_count) / (new_ect_count + new_ce_count);
//     DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
//     prague_estimator_->UpdateFromCongestionSignal(current_rate, ce_ratio, feedback.feedback_time);
    
//     // Update fusion engine with ECN estimate
//     double ecn_confidence = prague_estimator_->GetConfidence(feedback.feedback_time);
//     bandwidth_fusion_->UpdateEcnEstimate(prague_estimator_->GetCurrentEstimate(), ecn_confidence, feedback.feedback_time);
    
//     // Log congestion metrics
//     if (metrics_enabled_ && metrics_collector_) {
//       metrics_collector_->LogCongestionMetrics(feedback.feedback_time, new_ce_count, new_ect_count, ce_ratio);
//     }
//   }
  
//   // Update counters
//   ect_count_ = new_ect_count;
//   ce_count_ = new_ce_count;
// }

// void L4SPragueNetworkController::UpdateDelayBasedEstimator(const TransportPacketsFeedback& feedback) {
//   // Extract delay information and update delay-based estimator
//   // This is a simplified implementation - in practice, you'd need to properly
//   // convert the feedback format for DelayBasedBwe
  
//   if (last_rtt_.IsFinite()) {
//     // Update delay estimator with RTT information
//     // Note: This is a placeholder - actual implementation would need proper adaptation
//     DataRate delay_estimate = DataRate::KilobitsPerSec(1000);  // Placeholder
//     double delay_confidence = CalculateDelayConfidence(feedback.feedback_time);
//     bandwidth_fusion_->UpdateDelayEstimate(delay_estimate, delay_confidence, feedback.feedback_time);
//   }
// }

// void L4SPragueNetworkController::UpdateAckedBitrateEstimator(const TransportPacketsFeedback& feedback) {
//   // Update acknowledged bitrate estimator
//   // This is a simplified implementation - actual implementation would need
//   // proper packet acknowledgment processing
  
//   if (!feedback.packet_feedbacks.empty()) {
//     // Calculate acknowledged bitrate from feedback
//     DataRate acked_estimate = last_actual_bitrate_;  // Use calculated throughput as proxy
//     double acked_confidence = CalculateAckedConfidence(feedback.feedback_time);
//     bandwidth_fusion_->UpdateAckedEstimate(acked_estimate, acked_confidence, feedback.feedback_time);
//   }
// }

// void L4SPragueNetworkController::ProcessProbeResults(const TransportPacketsFeedback& feedback) {
//   // Process probe results and update fusion engine
//   // This is a placeholder - actual implementation would need to detect and process probe clusters
  
//   TimeDelta since_probe = feedback.feedback_time - last_probe_time_;
//   if (since_probe < TimeDelta::Seconds(2) && last_probe_estimate_ > DataRate::Zero()) {
//     double probe_confidence = CalculateProbeConfidence(feedback.feedback_time);
//     bandwidth_fusion_->UpdateProbeEstimate(last_probe_estimate_, probe_confidence, feedback.feedback_time);
//   }
// }

// void L4SPragueNetworkController::HandlePeriodicProbing(Timestamp now, NetworkControlUpdate* update) {
//   if (!config_.enable_probing || !probe_controller_) {
//     return;
//   }
  
//   // Check if it's time for periodic probing
//   bool should_probe = (now - last_probe_time_) >= config_.probe_interval;
//   should_probe = should_probe && ShouldProbeNow(now);
  
//   if (should_probe) {
//     InitiateProbing(now, update);
//     last_probe_time_ = now;
//   }
// }

// bool L4SPragueNetworkController::ShouldProbeNow(Timestamp now) const {
//   // Don't probe if we're experiencing heavy congestion
//   if (HasRecentCongestionSignals(now)) {
//     return false;
//   }
  
//   // Don't probe if ECN feedback is very fresh and confident
//   if (IsEcnFeedbackFresh(now) && prague_estimator_->GetConfidence(now) > 0.9) {
//     return false;
//   }
  
//   // Don't probe during high loss periods
//   if (last_loss_fraction_ > 0.02) {  // 2% loss threshold
//     return false;
//   }
  
//   return true;
// }

// void L4SPragueNetworkController::InitiateProbing(Timestamp now, NetworkControlUpdate* update) {
//   // Get current best estimate for probe rate calculation
//   DataRate current_estimate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  
//   // Request probe at 1.5x current estimate
//   DataRate probe_rate = current_estimate * 1.5;
//   if (max_target_rate_) {
//     probe_rate = std::min(probe_rate, *max_target_rate_);
//   }
  
//   // Store probe estimate for later processing
//   last_probe_estimate_ = probe_rate;
  
//   RTC_LOG(LS_INFO) << "L4S Prague: Initiating periodic probe at " << probe_rate.bps() << " bps";
  
//   // Note: Actual probe cluster creation would be done here with probe_controller_
//   // This is a simplified implementation
// }

// double L4SPragueNetworkController::CalculateEcnConfidence(Timestamp now) const {
//   return prague_estimator_->GetConfidence(now);
// }

// double L4SPragueNetworkController::CalculateDelayConfidence(Timestamp now) const {
//   if (!delay_estimator_ || !last_rtt_.IsFinite()) {
//     return 0.0;
//   }
  
//   // High confidence if RTT is stable
//   if (IsRttStable()) {
//     return 0.8;
//   }
//   return 0.5;
// }

// double L4SPragueNetworkController::CalculateProbeConfidence(Timestamp now) const {
//   TimeDelta since_probe = now - last_probe_time_;
//   if (since_probe < TimeDelta::Seconds(1)) {
//     return 0.95;  // Very high confidence in fresh probe results
//   } else if (since_probe < TimeDelta::Seconds(10)) {
//     return 0.8;   // Good confidence in recent probes
//   }
//   return 0.2;   // Low confidence in old probe results
// }

// double L4SPragueNetworkController::CalculateAckedConfidence(Timestamp now) const {
//   if (!acked_estimator_) {
//     return 0.0;
//   }
  
//   // Moderate confidence in acknowledged bitrate
//   return 0.6;
// }

// DataRate L4SPragueNetworkController::FuseBandwidthEstimates(Timestamp now) {
//   DataRate fused_rate = bandwidth_fusion_->GetFusedEstimate(now);
  
//   // Apply rate constraints
//   if (min_target_rate_ && fused_rate < *min_target_rate_) {
//     fused_rate = *min_target_rate_;
//   }
//   if (max_target_rate_ && fused_rate > *max_target_rate_) {
//     fused_rate = *max_target_rate_;
//   }
  
//   // Log fusion metrics
//   if (metrics_enabled_ && metrics_collector_) {
//     auto sources = bandwidth_fusion_->GetCurrentSources();
//     metrics_collector_->LogFusionMetrics(now, sources, fused_rate);
//   }
  
//   return fused_rate;
// }

// NetworkControlUpdate L4SPragueNetworkController::CreateRateUpdate(Timestamp at_time) const {
//   NetworkControlUpdate update;
  
//   if (!at_time.IsFinite()) {
//     at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
//   }
  
//   DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  
//   // Validate rate
//   if (!current_rate.IsFinite() || current_rate.bps() <= 0) {
//     current_rate = DataRate::KilobitsPerSec(300);
//   }
  
//   // Set target rate
//   update.target_rate = TargetTransferRate();
//   update.target_rate->at_time = at_time;
//   update.target_rate->network_estimate.at_time = at_time;
//   update.target_rate->network_estimate.bandwidth = current_rate;
//   update.target_rate->network_estimate.loss_rate_ratio = static_cast<float>(last_loss_fraction_);
//   update.target_rate->network_estimate.round_trip_time = last_estimated_round_trip_time_;
//   update.target_rate->network_estimate.bwe_period = TimeDelta::Millis(500);
//   update.target_rate->target_rate = current_rate;
  
//   // Set pacer config
//   update.pacer_config = PacerConfig();
//   update.pacer_config->at_time = at_time;
//   update.pacer_config->time_window = TimeDelta::Millis(10);
//   update.pacer_config->data_window = current_rate * update.pacer_config->time_window;
//   update.pacer_config->pad_window = DataSize::Zero();
  
//   return update;
// }

// void L4SPragueNetworkController::MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update, Timestamp at_time) {
//   if (!at_time.IsFinite()) {
//     at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
//   }
  
//   NetworkControlUpdate rate_update = CreateRateUpdate(at_time);
  
//   if (rate_update.pacer_config) {
//     update->pacer_config = rate_update.pacer_config;
//   }
//   if (rate_update.target_rate) {
//     update->target_rate = rate_update.target_rate;
//   }
// }

// bool L4SPragueNetworkController::IsL4SActive() const {
//   return ecn_supported_ && ecn_capable_network_;
// }

// bool L4SPragueNetworkController::HasRecentCongestionSignals(Timestamp now) const {
//   return !last_congestion_signal_.IsInfinite() && 
//          (now - last_congestion_signal_) < TimeDelta::Seconds(2);
// }

// bool L4SPragueNetworkController::IsEcnFeedbackFresh(Timestamp now) const {
//   return HasRecentCongestionSignals(now) || 
//          (ecn_supported_ && (now - last_congestion_signal_) < TimeDelta::Seconds(5));
// }

// bool L4SPragueNetworkController::EstimatesAreDiverging() const {
//   // Simple check for estimate divergence
//   auto sources = bandwidth_fusion_->GetCurrentSources();
//   if (sources.ecn_estimate > DataRate::Zero() && sources.delay_estimate > DataRate::Zero()) {
//     double ratio = sources.ecn_estimate.bps() / static_cast<double>(sources.delay_estimate.bps());
//     return ratio > 2.0 || ratio < 0.5;  // 2x divergence threshold
//   }
//   return false;
// }

// bool L4SPragueNetworkController::IsRttStable() const {
//   // Simplified RTT stability check
//   return last_rtt_.IsFinite() && last_rtt_ < TimeDelta::Millis(100);
// }

// void L4SPragueNetworkController::UpdateThroughputWindow(const TransportPacketsFeedback& feedback) {
//   constexpr TimeDelta kThroughputWindow = TimeDelta::Millis(500);
//   Timestamp now = feedback.feedback_time;
  
//   // Add new packets to the window
//   for (const auto& packet : feedback.packet_feedbacks) {
//     if (packet.receive_time.IsFinite() && packet.sent_packet.send_time.IsFinite()) {
//       throughput_window_.emplace_back(packet.receive_time, packet.sent_packet.size.bytes());
//     }
//   }
  
//   // Remove old packets outside the window
//   while (!throughput_window_.empty() && now - throughput_window_.front().first > kThroughputWindow) {
//     throughput_window_.pop_front();
//   }
  
//   // Calculate throughput over the window
//   int64_t window_bytes = 0;
//   if (!throughput_window_.empty()) {
//     Timestamp window_start = throughput_window_.front().first;
//     Timestamp window_end = throughput_window_.back().first;
//     for (const auto& entry : throughput_window_) {
//       window_bytes += entry.second;
//     }
    
//     TimeDelta window_interval = window_end - window_start;
//     if (window_interval > TimeDelta::Millis(1)) {
//       last_actual_bitrate_ = DataRate::BitsPerSec(
//           static_cast<int64_t>((window_bytes * 8) / window_interval.seconds<double>()));
//     } else {
//       last_actual_bitrate_ = DataRate::Zero();
//     }
//   }
// }

// void L4SPragueNetworkController::LogPeriodicMetrics(Timestamp at_time) {
//   if (!metrics_enabled_ || !metrics_collector_) {
//     return;
//   }
  
//   if (at_time - metrics_last_logged_ < kMetricsLoggingInterval) {
//     return;
//   }
  
//   metrics_last_logged_ = at_time;
  
//   // Log bandwidth metrics
//   DataRate target_rate = target_rate_.value_or(DataRate::Zero());
//   metrics_collector_->LogBandwidthMetrics(at_time, target_rate, last_actual_bitrate_);
  
//   // Log delay metrics
//   if (last_rtt_.IsFinite()) {
//     metrics_collector_->LogDelayMetrics(at_time, last_rtt_, last_rtt_ / 2, TimeDelta::Zero());
//   }
  
//   // Log loss metrics
//   metrics_collector_->LogLossMetrics(at_time, last_loss_fraction_, last_packets_lost_);
  
//   // Log periodic summary
//   metrics_collector_->LogPeriodicSummary(at_time);
// }

// }  // namespace webrtc






// new methods by Gemini


//header file 

// Add to L4SNetworkController in your .h file:


// Add to PragueCapacityEstimator in your .h file:

// //Timestamp last_ai_update_time_ = Timestamp::MinusInfinity();
// //Timestamp last_feedback_time_ = Timestamp::MinusInfinity();

// Add to L4SNetworkController in your .h file:

// Add to L4SNetworkController in your .cc file:

// void webrtc::PragueCapacityEstimator::UpdateFromCongestionSignal(DataRate current_rate, double ce_ratio, Timestamp current_time) {
//   last_feedback_time_ = current_time; // Track the last time we heard from the network

//   if (ce_ratio > 0.0) {  // CE-marked packets detected
//     non_ce_packet_count_ = 0;
    
//     if (discovery_mode_active_ && !first_ce_mark_detected_) {
//       discovery_mode_active_ = false;
//       first_ce_mark_detected_ = true;
//       RTC_LOG(LS_INFO) << "Prague: Exiting discovery mode - first CE mark detected (ce_ratio=" 
//                        << ce_ratio << ")";
//     }
    
//     constexpr double g = 1.0 / 16.0;  // RFC 9330 standard gain
//     alpha_ = (1.0 - g) * alpha_ + g * ce_ratio;
    
//     if (direction_flag_ == 1) {
//       direction_flag_ = -1;
//       double reduction_factor = 1.0 - alpha_ / 2.0;
//       DataRate reduced = std::max(current_rate * reduction_factor, min_target_rate_);
//       reduced = std::max(reduced, DataRate::KilobitsPerSec(20));
//       congestion_based_estimate_ = reduced;
//       last_md_time_ = current_time; 
      
//       RTC_LOG(LS_INFO) << "Prague: Switched to reduction mode (alpha=" << alpha_
//                        << ", reduction_factor=" << reduction_factor
//                        << "), new rate=" << congestion_based_estimate_.bps() << " bps";
//     } else {
//       TimeDelta gate_rtt = current_rtt_.IsFinite() && !current_rtt_.IsZero() ? current_rtt_ : TimeDelta::Millis(100);
//       bool gate_open = last_md_time_.IsInfinite() || (current_time - last_md_time_ >= gate_rtt);
//       if (gate_open) {
//         double additional_reduction = 1.0 - alpha_ / 4.0; 
//         DataRate further_reduced = std::max(congestion_based_estimate_ * additional_reduction, min_target_rate_);
//         further_reduced = std::max(further_reduced, DataRate::KilobitsPerSec(20));
//         congestion_based_estimate_ = further_reduced;
//         last_md_time_ = current_time;
//       }
//     }
//     last_congestion_signal_ = current_time;
                      
//   } else {  // No CE marks in this batch
//     non_ce_packet_count_++;
//     if (direction_flag_ == -1 && non_ce_packet_count_ >= kNonCeThreshold) {
//       direction_flag_ = 1;
//       non_ce_packet_count_ = 0;
//       RTC_LOG(LS_INFO) << "Prague: Switched to additive mode after " << kNonCeThreshold 
//                        << " consecutive non-CE packets";
//     }
//     // AI logic is entirely removed from here!
//   }
// }





// void webrtc::PragueCapacityEstimator::OnTimeUpdate(Timestamp current_time) {
//   if (last_update_time_.IsInfinite() || last_ai_update_time_.IsInfinite()) {
//     last_update_time_ = current_time;
//     last_ai_update_time_ = current_time;
//     return;
//   }
  
//   TimeDelta elapsed = current_time - last_update_time_;
//   TimeDelta ai_elapsed = current_time - last_ai_update_time_;

//   // 1. Autonomous Additive Increase (Virtual ACKs)
//   // Only increase if we are in additive mode AND we have recent network feedback
//   // (Prevents infinite ramp-up if the network dies completely)
//   bool network_is_alive = !last_feedback_time_.IsInfinite() && 
//                           (current_time - last_feedback_time_) < TimeDelta::Seconds(2);

//   if (direction_flag_ == 1 && network_is_alive) {
//     if (additive_hold_until_.IsInfinite() || current_time >= additive_hold_until_) {
      
//       double rtt_s = current_rtt_.IsFinite() && !current_rtt_.IsZero() ? current_rtt_.seconds<double>() : 0.05;
//       double elapsed_s = ai_elapsed.seconds<double>();
      
//       // Calculate continuous bits per second increase based on elapsed time
//       constexpr double mss_bits = 1440.0 * 8.0;
//       double theoretical_increase = (mss_bits / rtt_s) * (elapsed_s / rtt_s);
      
//       int64_t ai_step_bps = static_cast<int64_t>(theoretical_increase);

//       // Discovery Mode multiplier
//       if (discovery_mode_active_ && !first_ce_mark_detected_ && congestion_based_estimate_.bps() < 5000000) {
//         ai_step_bps *= 5;
//         ai_step_bps = std::min(ai_step_bps, static_cast<int64_t>(2000000 * elapsed_s)); // Cap burst
//       } else {
//         // Apply context-aware logic
//         ai_step_bps = CalculateContextAwareAiStep(ai_step_bps, congestion_based_estimate_, current_time, elapsed_s);
//       }

//       if (ai_step_bps > 0) {
//         DataRate increased = congestion_based_estimate_ + DataRate::BitsPerSec(ai_step_bps);
//         if (max_target_rate_ > DataRate::Zero()) {
//             increased = std::min(increased, max_target_rate_);
//         }
//         congestion_based_estimate_ = increased;
//       }
//     }
//   }

//   last_ai_update_time_ = current_time;

//   // 2. Existing Time Decay Logic (if no feedback for a long time)
//   if (elapsed >= kDecayInterval) {
//     congestion_based_estimate_ = std::max(congestion_based_estimate_ * 0.95, min_target_rate_);
//     congestion_based_estimate_ = std::max(congestion_based_estimate_, DataRate::KilobitsPerSec(20));
//     last_update_time_ = current_time;
//   }
  
//   if (first_ce_mark_detected_ && !discovery_mode_active_) {
//     TimeDelta since_congestion = current_time - last_congestion_signal_;
//     if (since_congestion > TimeDelta::Seconds(30)) {
//       discovery_mode_active_ = true;
//       first_ce_mark_detected_ = false; 
//     }
//   }
// }






// void webrtc::L4SNetworkController::ProcessEcnFeedback(const TransportPacketsFeedback& feedback, DataRate current_fused_rate) {
//   if (feedback.packet_feedbacks.empty()) {
//     return;
//   }

//   static int window_ce_count = 0;
//   static int window_ect_count = 0;
//   static Timestamp window_start_time = Timestamp::MinusInfinity();
//   TimeDelta window_duration = last_rtt_.IsFinite() && !last_rtt_.IsZero() ? last_rtt_ : TimeDelta::Millis(100);

//   int batch_ect_count = 0;
//   int batch_ce_count = 0;
//   bool probe_caused_congestion = false;

//   for (const auto& packet : feedback.packet_feedbacks) {
//     if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1 || packet.ecn == EcnMarking::kCe) {
//       batch_ect_count++;
//     }
//     if (packet.ecn == EcnMarking::kCe) {
//       batch_ce_count++;
//       last_congestion_signal_ = feedback.feedback_time;
      
//       // CRITICAL DISTINCTION: Was this CE mark caused by our own probe burst?
//       if (packet.sent_packet.pacing_info.probe_cluster_id != PacedPacketInfo::kNotAProbe) {
//         probe_caused_congestion = true;
//       }
//     }
//   }

//   if (batch_ect_count > 0 || batch_ce_count > 0) {
//     ecn_supported_ = true;
//     prague_estimator_->UpdateEcnActivity(feedback.feedback_time);
//   }

//   window_ce_count += batch_ce_count;
//   window_ect_count += batch_ect_count;

//   int batch_total = batch_ect_count + batch_ce_count;
//   if (batch_total < 3) {
//     HandleRecoveryDetection(batch_ect_count, batch_ce_count, feedback.feedback_time);
//     return;
//   }

//   if (window_start_time.IsInfinite() || (feedback.feedback_time - window_start_time) >= window_duration) {
//     int window_total = window_ect_count + window_ce_count;
//     double ce_ratio = (window_total > 0) ? static_cast<double>(window_ce_count) / window_total : 0.0;

//     const int min_packets_threshold = 3;
//     if (window_total >= min_packets_threshold) {
      
//       // Logic split: Network CE vs Probe CE
//       if (batch_ce_count > 0 && probe_caused_congestion) {
//         RTC_LOG(LS_INFO) << "L4S: CE marks triggered by Micro-Probe. Freezing AI, but bypassing MD.";
        
//         // Tell Prague to hold current rate, do NOT apply Multiplicative Decrease
//         prague_estimator_->SetAdditiveHoldUntil(feedback.feedback_time + (last_rtt_ * 2));
        
//         // Re-align probe estimate to safe actual bitrate
//         if (!last_actual_bitrate_.IsZero()) {
//           bandwidth_fusion_->UpdateProbeEstimate(last_actual_bitrate_, 0.8, feedback.feedback_time);
//         }

//       } else {
//         // Standard Cross-Traffic Congestion - Apply full Prague MD
//         DataRate prague_input_rate = prague_estimator_->IsDiscoveryModeActive() 
//                                      ? prague_estimator_->GetCurrentEstimate() 
//                                      : DetermineBottleneckAwareTarget(current_fused_rate);

//         prague_estimator_->UpdateFromCongestionSignal(prague_input_rate, ce_ratio, feedback.feedback_time);

//         if (!last_actual_bitrate_.IsZero()) {
//           DataRate floor = last_actual_bitrate_ * 0.5;
//           if (prague_estimator_->GetCurrentEstimate() < floor) {
//             prague_estimator_->SetCurrentEstimate(floor);
//           }
//         }
//       }

//       double ecn_confidence = prague_estimator_->GetConfidence(feedback.feedback_time);
//       if (prague_estimator_->GetDirectionFlag() == -1) {
//         ecn_confidence = std::max(ecn_confidence, 0.95);
//       }
//       bandwidth_fusion_->UpdateEcnEstimate(prague_estimator_->GetCurrentEstimate(), ecn_confidence, feedback.feedback_time);
//     }

//     window_ce_count = 0;
//     window_ect_count = 0;
//     window_start_time = feedback.feedback_time;
//   }

//   HandleRecoveryDetection(batch_ect_count, batch_ce_count, feedback.feedback_time);
// }





// void webrtc::L4SNetworkController::InitiateProbing(Timestamp now, NetworkControlUpdate* update) {
//   if (!probe_controller_) {
//     return;
//   }

//   DataRate current_estimate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  
//   // Shift to Micro-Probing: 5% to 10% elevation maximum
//   // Overwrite the GCC config defaults to prevent burst marks
//   double micro_probe_multiplier = 1.05; 
//   if (IsApplicationLimited()) {
//     micro_probe_multiplier = 1.10; // Slightly higher if ALR limits us
//   }
  
//   DataRate probe_rate = current_estimate * micro_probe_multiplier;
//   if (max_target_rate_) {
//     probe_rate = std::min(probe_rate, *max_target_rate_);
//   }

//   // Force the ProbeController to use our micro-probe rate rather than its default 3x multipliers
//   auto probes = probe_controller_->SetEstimatedBitrate(current_estimate, BandwidthLimitedCause::kDelayBasedLimited, now);
//   if (probes.empty()) {
//      probes = probe_controller_->RequestProbe(now);
//   }
  
//   // Clamp all requested probes down to the micro_probe_multiplier constraint
//   for (auto& probe : probes) {
//       if (probe.target_data_rate > probe_rate) {
//           probe.target_data_rate = probe_rate;
//       }
//   }

//   if (!probes.empty()) {
//     update->probe_cluster_configs.insert(update->probe_cluster_configs.end(),
//                                          probes.begin(), probes.end());
//     StartProbeHold(now);
//     RTC_LOG(LS_VERBOSE) << "L4S: Micro-Probe requested at " << probe_rate.bps() << " bps";
//   }
// }



// int64_t webrtc::PragueCapacityEstimator::CalculateContextAwareAiStep(
//     int64_t time_scaled_ai_step, 
//     DataRate current_rate, 
//     Timestamp current_time, 
//     double elapsed_s) {
  
//   // Safety checks - fallback scales with time (e.g., 1 Mbps per second of elapsed time)
//   int64_t time_scaled_fallback = static_cast<int64_t>(1000000 * elapsed_s);
  
//   if (time_scaled_ai_step <= 0 || !current_rate.IsFinite() || current_time.IsInfinite() || elapsed_s <= 0.0) {
//     RTC_LOG(LS_WARNING) << "Prague: Invalid input parameters to CalculateContextAwareAiStep";
//     return time_scaled_fallback; 
//   }
  
//   double context_multiplier = 1.0;
  
//   // 1. Time since last congestion signal
//   TimeDelta since_congestion = current_time - last_congestion_signal_;
//   if (!last_congestion_signal_.IsInfinite()) {
//     if (since_congestion < TimeDelta::Seconds(1)) {
//       context_multiplier *= 0.3;
//     } else if (since_congestion < TimeDelta::Seconds(5)) {
//       context_multiplier *= 0.6;
//     } else if (since_congestion > TimeDelta::Seconds(10)) {
//       context_multiplier *= 1.5;
//     }
//   }
  
//   // 2. Alpha value (congestion severity)
//   if (alpha_ > 0.5) {
//     context_multiplier *= 0.2;
//   } else if (alpha_ > 0.1) {
//     context_multiplier *= 0.5;
//   } else if (alpha_ < 0.01) {
//     context_multiplier *= 1.3;
//   }
  
//   // 3. Rate magnitude
//   int64_t current_bps = current_rate.bps();
//   if (current_bps > 50000000) {        // > 50 Mbps
//     context_multiplier *= 0.1;
//   } else if (current_bps > 10000000) { // > 10 Mbps
//     context_multiplier *= 0.2;
//   } else if (current_bps > 5000000) {  // > 5 Mbps
//     context_multiplier *= 0.3;
//   } else if (current_bps > 1000000) {  // > 1 Mbps
//     context_multiplier *= 0.6;
//   } else if (current_bps < 300000) {   // < 300 Kbps
//     context_multiplier *= 1.5;
//   }
  
//   // 4. Direction flag stability
//   if (direction_flag_ == 1 && non_ce_packet_count_ > kNonCeThreshold * 2) {
//     context_multiplier *= 1.2;
//   }
  
//   // 5. RTT-based scaling
//   double rtt_seconds = current_rtt_.IsFinite() ? current_rtt_.seconds<double>() : 0.05;
//   if (rtt_seconds < 0.01) {
//     context_multiplier *= 0.1;
//   } else if (rtt_seconds < 0.05) {
//     context_multiplier *= 0.3;
//   } else if (rtt_seconds > 0.1) {
//     context_multiplier *= std::min(2.0, rtt_seconds / 0.05);
//   }
  
//   // 6. Apply multiplier to the ALREADY time-scaled step
//   int64_t context_ai_bps = static_cast<int64_t>(time_scaled_ai_step * context_multiplier);
  
//   // 7. Time-scaled bounding
//   int64_t min_step_bps = time_scaled_ai_step / 20; 
  
//   // The rate-based cap limits the step to 10% of current rate PER SECOND of elapsed time
//   int64_t rate_based_max_step = std::max(
//       static_cast<int64_t>(current_bps * 0.1 * elapsed_s), 
//       time_scaled_fallback 
//   );
  
//   int64_t max_step_bps = std::min(
//       time_scaled_ai_step * 2,  
//       rate_based_max_step       
//   );
  
//   context_ai_bps = std::max(min_step_bps, std::min(context_ai_bps, max_step_bps));
  
//   if (context_ai_bps <= 0 || context_ai_bps > 1000000000 || !std::isfinite(context_ai_bps)) {
//     RTC_LOG(LS_WARNING) << "Prague: Invalid context AI step calculated: " << context_ai_bps;
//     context_ai_bps = time_scaled_fallback; 
//   }
  
//   return context_ai_bps;
// }







// Add to L4SNetworkController in your .cc file: