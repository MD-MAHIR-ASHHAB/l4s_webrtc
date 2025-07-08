#include "modules/congestion_controller/l4s/l4s_network_controller.h"

#include <algorithm>
#include <memory>
#include <numeric>
#include <utility>

#include "absl/strings/match.h"
#include "api/field_trials_view.h"
#include "api/rtc_event_log/rtc_event_log.h"
#include "api/transport/bandwidth_usage.h"
#include "api/transport/goog_cc_factory.h"
#include "api/transport/network_types.h"
#include "api/units/data_rate.h"
#include "api/units/data_size.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "logging/rtc_event_log/events/rtc_event_probe_cluster_created.h"
#include "modules/congestion_controller/goog_cc/acknowledged_bitrate_estimator.h"
#include "modules/congestion_controller/goog_cc/delay_based_bwe.h"
#include "modules/congestion_controller/goog_cc/goog_cc_network_control.h"
#include "modules/congestion_controller/goog_cc/send_side_bandwidth_estimation.h"
#include "modules/congestion_controller/goog_cc/probe_controller.h"
#include "api/numerics/samples_stats_counter.h"
#include "system_wrappers/include/clock.h"
#include "rtc_base/checks.h"
#include "rtc_base/experiments/field_trial_parser.h"
#include "rtc_base/logging.h"
#include "system_wrappers/include/metrics.h"

// Metrics collection
#include "api/test/metrics/global_metrics_logger_and_exporter.h"
#include "api/test/metrics/metrics_logger.h"

namespace webrtc {

// AdaptiveCapacityEstimator implementation
AdaptiveCapacityEstimator::AdaptiveCapacityEstimator(DataRate initial_conservative_estimate)
    : conservative_estimate_(initial_conservative_estimate),
      probe_based_estimate_(initial_conservative_estimate),
      congestion_based_estimate_(initial_conservative_estimate),
      historical_estimate_(initial_conservative_estimate),
      last_update_time_(Timestamp::MinusInfinity()) {
  sustained_rates_history_.push_back(initial_conservative_estimate);
}

AdaptiveCapacityEstimator::~AdaptiveCapacityEstimator() = default;

void AdaptiveCapacityEstimator::UpdateFromProbeResult(DataRate probe_rate, bool successful) {
  if (successful) {
    // Successful probe suggests we can handle this rate
    DataRate proven_capacity = probe_rate;
    probe_based_estimate_ = std::min(proven_capacity * 1.2, kAbsoluteMaxLimit);
    RTC_LOG(LS_INFO) << "AdaptiveCapacity: Probe success at " << probe_rate.bps() 
                     << " bps, updating probe_based_estimate to " << probe_based_estimate_.bps() << " bps";
  } else {
    // Failed probe suggests we're at or near capacity
    probe_based_estimate_ = std::min(probe_rate * 0.8, probe_based_estimate_);
    RTC_LOG(LS_INFO) << "AdaptiveCapacity: Probe failed at " << probe_rate.bps() 
                     << " bps, reducing probe_based_estimate to " << probe_based_estimate_.bps() << " bps";
  }
}

void AdaptiveCapacityEstimator::UpdateFromCongestionSignal(DataRate current_rate, double ce_ratio) {
  if (ce_ratio > 0.1) {
    // High congestion - reduce estimate
    congestion_based_estimate_ = std::max(current_rate * 0.8, kAbsoluteMinLimit);
    RTC_LOG(LS_INFO) << "AdaptiveCapacity: High congestion (CE ratio=" << ce_ratio 
                     << "), reducing congestion_based_estimate to " << congestion_based_estimate_.bps() << " bps";
  } else if (ce_ratio < 0.01 && current_rate > congestion_based_estimate_ * 0.9) {
    // Low congestion and we're using most of our estimate - can increase
    congestion_based_estimate_ = std::min(current_rate * 1.3, kAbsoluteMaxLimit);
    RTC_LOG(LS_INFO) << "AdaptiveCapacity: Low congestion (CE ratio=" << ce_ratio 
                     << "), increasing congestion_based_estimate to " << congestion_based_estimate_.bps() << " bps";
  }
}

void AdaptiveCapacityEstimator::UpdateFromSustainedRate(DataRate sustained_rate) {
  // Add to history
  sustained_rates_history_.push_back(sustained_rate);
  if (sustained_rates_history_.size() > kHistoryWindowSize) {
    sustained_rates_history_.pop_front();
  }
  
  // Update historical estimate based on maximum sustained rate
  DataRate historical_max = *std::max_element(sustained_rates_history_.begin(), 
                                             sustained_rates_history_.end());
  historical_estimate_ = std::min(historical_max * 1.2, kAbsoluteMaxLimit);
}

void AdaptiveCapacityEstimator::UpdateFromRtt(TimeDelta rtt) {
  min_rtt_ = std::min(min_rtt_, rtt);
  
  // Update conservative estimate based on connection type
  ConnectionType type = DetectConnectionType(min_rtt_, GetMaxRealisticBandwidth());
  conservative_estimate_ = GetConservativeEstimateForType(type);
}

DataRate AdaptiveCapacityEstimator::GetMaxRealisticBandwidth() const {
  // Use the most conservative estimate that has been recently validated
  DataRate estimate = std::min({
      conservative_estimate_,
      probe_based_estimate_,
      congestion_based_estimate_,
      historical_estimate_
  });
  
  // Ensure we stay within absolute bounds
  return std::max(kAbsoluteMinLimit, std::min(estimate, kAbsoluteMaxLimit));
}

void AdaptiveCapacityEstimator::OnTimeUpdate(Timestamp current_time) {
  if (last_update_time_.IsInfinite()) {
    last_update_time_ = current_time;
    return;
  }
  
  TimeDelta elapsed = current_time - last_update_time_;
  if (elapsed >= kDecayInterval) {
    // Gradually decay estimates if not reinforced
    probe_based_estimate_ = std::max(probe_based_estimate_ * 0.95, conservative_estimate_);
    congestion_based_estimate_ = std::max(congestion_based_estimate_ * 0.95, conservative_estimate_);
    historical_estimate_ = std::max(historical_estimate_ * 0.95, conservative_estimate_);
    
    last_update_time_ = current_time;
  }
}

AdaptiveCapacityEstimator::ConnectionType AdaptiveCapacityEstimator::DetectConnectionType(
    TimeDelta rtt, DataRate estimate) const {
  if (rtt > TimeDelta::Millis(200)) return ConnectionType::MOBILE_SLOW;
  if (rtt > TimeDelta::Millis(100)) return ConnectionType::MOBILE_FAST;
  if (estimate < DataRate::KilobitsPerSec(50000)) return ConnectionType::WIFI_TYPICAL;
  return ConnectionType::WIRED_FAST;
}

DataRate AdaptiveCapacityEstimator::GetConservativeEstimateForType(ConnectionType type) const {
  switch (type) {
    case ConnectionType::MOBILE_SLOW:
      return DataRate::KilobitsPerSec(5000);    // 5 Mbps
    case ConnectionType::MOBILE_FAST:
      return DataRate::KilobitsPerSec(20000);   // 20 Mbps
    case ConnectionType::WIFI_TYPICAL:
      return DataRate::KilobitsPerSec(50000);   // 50 Mbps
    case ConnectionType::WIRED_FAST:
      return DataRate::KilobitsPerSec(100000);  // 100 Mbps
    case ConnectionType::UNKNOWN:
    default:
      return DataRate::KilobitsPerSec(10000);   // 10 Mbps conservative default
  }
}

L4SNetworkController::L4SNetworkController(NetworkControllerConfig config,
                                           L4SControllerConfig l4s_config,
                                           test::MetricsLogger* metrics_logger)
    : env_(config.env),
      fallback_to_gcc_(l4s_config.fallback_to_gcc),
      use_ect1_marking_(l4s_config.use_ect1_marking),
      prague_controller_(
          std::make_unique<L4SPragueController>(env_.field_trials())),
      // Initialize adaptive capacity estimator first (based on header order)
      capacity_estimator_(
          std::make_unique<AdaptiveCapacityEstimator>(
              DataRate::KilobitsPerSec(10000))),  // Start with 10 Mbps conservative estimate
      // Initialize GCC-style bandwidth estimation components
      acknowledged_bitrate_estimator_(
          std::make_unique<AcknowledgedBitrateEstimator>(&env_.field_trials())),
      delay_based_bwe_(std::make_unique<DelayBasedBwe>(
          &env_.field_trials(),
          &env_.event_log(),
          nullptr)),  // No network state predictor
      bandwidth_estimation_(
          std::make_unique<SendSideBandwidthEstimation>(&env_.field_trials(),
                                                        &env_.event_log())),
      probe_controller_(
          std::make_unique<ProbeController>(&env_.field_trials(),
                                            &env_.event_log())),
      // Initialize metrics collection
      metrics_enabled_(l4s_config.enable_metrics_collection && metrics_logger != nullptr),
      current_active_controller_("l4s_initializing") {
  
  // Initialize metrics collector if enabled
  if (metrics_enabled_) {
    metrics_collector_ = std::make_unique<L4SMetricsCollector>(
        metrics_logger, l4s_config.test_case_name, &env_.clock());
    RTC_LOG(LS_INFO) << "L4S: Metrics collection enabled for test case: " 
                     << l4s_config.test_case_name;
  } else {
    RTC_LOG(LS_INFO) << "L4S: Metrics collection disabled";
  }
  // Create GCC controller for fallback if needed
  if (fallback_to_gcc_) {
    GoogCcFactoryConfig factory_config;
    auto factory = std::make_unique<GoogCcNetworkControllerFactory>(
        std::move(factory_config));
    gcc_controller_ = factory->Create(config);
  }

  if (config.constraints.starting_rate) {
    starting_rate_ = config.constraints.starting_rate;
    // Validate starting rate
    if (!starting_rate_->IsFinite() || starting_rate_->bps() <= 0) {
      RTC_LOG(LS_WARNING) << "L4S: Invalid starting rate " << starting_rate_->bps() 
                          << " bps, using 300 kbps";
      starting_rate_ = DataRate::KilobitsPerSec(300);
    }
  }

  min_target_rate_ = config.constraints.min_data_rate;
  max_target_rate_ = config.constraints.max_data_rate;
  
  // Validate rate constraints
  if (min_target_rate_ && (!min_target_rate_->IsFinite() || min_target_rate_->bps() <= 0)) {
    RTC_LOG(LS_WARNING) << "L4S: Invalid min target rate, clearing";
    min_target_rate_.reset();
  }
  if (max_target_rate_ && (!max_target_rate_->IsFinite() || max_target_rate_->bps() <= 0)) {
    RTC_LOG(LS_WARNING) << "L4S: Invalid max target rate, clearing"; 
    max_target_rate_.reset();
  }
  
  // Initialize ProbeController with bitrate constraints
  start_bitrate_ = starting_rate_.value_or(DataRate::KilobitsPerSec(300));
  max_bitrate_ = max_target_rate_.value_or(DataRate::KilobitsPerSec(100000));
  
  // Enable periodic ALR probing for bandwidth discovery
  probe_controller_->EnablePeriodicAlrProbing(true);
  
  // Set initial bitrates in ProbeController to enable probing
  DataRate min_bitrate = min_target_rate_.value_or(DataRate::KilobitsPerSec(30));
  auto initial_probe_clusters = probe_controller_->SetBitrates(min_bitrate, start_bitrate_, max_bitrate_,
                                                               env_.clock().CurrentTime());
  
  // Log initial probe clusters (will be applied by pacing controller during startup)
  if (!initial_probe_clusters.empty()) {
    RTC_LOG(LS_INFO) << "L4S: Created with " << initial_probe_clusters.size() 
                     << " initial probe cluster(s)";
  }

  RTC_LOG(LS_WARNING) << "L4S network controller created"
                      << " fallback_to_gcc: " << fallback_to_gcc_
                      << " use_ect1_marking: " << use_ect1_marking_
                      << " starting_rate: "
                      << (starting_rate_ ? starting_rate_->bps() : 0) << " bps"
                      << " min_target: "
                      << (min_target_rate_ ? min_target_rate_->bps() : 0)
                      << " bps"
                      << " max_target: "
                      << (max_target_rate_ ? max_target_rate_->bps() : 0)
                      << " bps"
                      << " max_realistic_bandwidth: " << capacity_estimator_->GetMaxRealisticBandwidth().bps() << " bps";
                      
  // Initialize the adaptive capacity estimator with reasonable starting values
  if (config.constraints.starting_rate) {
    capacity_estimator_->UpdateFromSustainedRate(*config.constraints.starting_rate);
  }
  
  // Keep max_realistic_bandwidth_ as backup, but primary logic will use capacity_estimator_
  max_realistic_bandwidth_ = DataRate::KilobitsPerSec(100000);  // 100 Mbps upper limit
  
  // Validate critical member variables after initialization
  if (!max_realistic_bandwidth_.IsFinite()) {
    RTC_LOG(LS_ERROR) << "L4S: CRITICAL - max_realistic_bandwidth_ is not finite!";
    max_realistic_bandwidth_ = DataRate::KilobitsPerSec(100000);
  }
  if (!last_acknowledged_rate_.IsFinite()) {
    RTC_LOG(LS_ERROR) << "L4S: CRITICAL - last_acknowledged_rate_ is not finite!";
    last_acknowledged_rate_ = DataRate::Zero();
  }
  if (!last_delay_based_estimate_.IsFinite()) {
    RTC_LOG(LS_ERROR) << "L4S: CRITICAL - last_delay_based_estimate_ is not finite!";
    last_delay_based_estimate_ = DataRate::Zero();
  }
}

L4SNetworkController::~L4SNetworkController() = default;

webrtc::NetworkControlUpdate L4SNetworkController::OnNetworkAvailability(
    webrtc::NetworkAvailability msg) {
  webrtc::NetworkControlUpdate update;

  // Use ProbeController for initial probing when network becomes available
  auto probe_clusters = probe_controller_->OnNetworkAvailability(msg);
  if (!probe_clusters.empty()) {
    update.probe_cluster_configs = std::move(probe_clusters);
    RTC_LOG(LS_INFO) << "L4S: Network available, initiated " << update.probe_cluster_configs.size() 
                     << " initial probe(s)";
  }

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    auto gcc_update = gcc_controller_->OnNetworkAvailability(msg);
    // Merge probe configs if GCC also wants to probe
    if (!gcc_update.probe_cluster_configs.empty()) {
      update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                         gcc_update.probe_cluster_configs.begin(),
                                         gcc_update.probe_cluster_configs.end());
    }
  }

  return update;
}

webrtc::NetworkControlUpdate L4SNetworkController::OnNetworkRouteChange(
    webrtc::NetworkRouteChange msg) {
  webrtc::NetworkControlUpdate update;

  RTC_LOG(LS_WARNING) << "L4S: OnNetworkRouteChange called";

  // Validate input constraints to prevent crashes
  if (msg.constraints.starting_rate && 
      (!msg.constraints.starting_rate->IsFinite() || msg.constraints.starting_rate->bps() <= 0)) {
    RTC_LOG(LS_WARNING) << "L4S: Invalid starting rate in constraints: " 
                        << msg.constraints.starting_rate->bps() << " bps";
    msg.constraints.starting_rate.reset();
  }

  // Reset ECN support detection on network change
  ecn_supported_ = false;
  ecn_capable_network_ = false;

  // Set initial rates
  if (msg.constraints.starting_rate) {
    starting_rate_ = msg.constraints.starting_rate;
  }

  min_target_rate_ = msg.constraints.min_data_rate;
  max_target_rate_ = msg.constraints.max_data_rate;
  
  // Update ProbeController with new bitrate constraints
  DataRate min_rate = min_target_rate_.value_or(DataRate::KilobitsPerSec(30));
  DataRate start_rate = starting_rate_.value_or(DataRate::KilobitsPerSec(300));
  DataRate max_rate = max_target_rate_.value_or(DataRate::KilobitsPerSec(100000));
  
  auto probe_clusters = probe_controller_->SetBitrates(min_rate, start_rate, max_rate, 
                                                       msg.at_time);
  if (!probe_clusters.empty()) {
    update.probe_cluster_configs = std::move(probe_clusters);
    RTC_LOG(LS_INFO) << "L4S: Route change triggered " << update.probe_cluster_configs.size() 
                     << " probe(s)";
  }

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    auto gcc_update = gcc_controller_->OnNetworkRouteChange(msg);
    // Copy other relevant updates from GCC
    update.target_rate = gcc_update.target_rate;
    update.pacer_config = gcc_update.pacer_config;
  }

  return update;
}

webrtc::NetworkControlUpdate L4SNetworkController::OnProcessInterval(
    webrtc::ProcessInterval msg) {
  webrtc::NetworkControlUpdate update;

  // Log periodic metrics
  LogPeriodicMetrics(msg.at_time);
  LogControllerState(msg.at_time);

  // Get rate from Prague controller if active
  if (IsL4SActive()) {
    DataRate current_rate_for_transport_ =
        target_rate_.value_or(DataRate::KilobitsPerSec(300));

    // RTC_LOG(LS_INFO)
    //     << "L4S DEBUG: Starting cycle with current_rate_for_transport_="
    //     << current_rate_for_transport_.bps() << " bps (from target_rate_="
    //     << (target_rate_ ? target_rate_->bps() : -1) << ")";

    // Update the adaptive capacity estimator with time-based decay
    capacity_estimator_->OnTimeUpdate(msg.at_time);
    
    // Consider BWE estimates for capacity limiting using adaptive estimator
    DataRate adaptive_max_capacity = capacity_estimator_->GetMaxRealisticBandwidth();
    DataRate bwe_based_limit = adaptive_max_capacity;
    
    RTC_LOG(LS_INFO) << "L4S OnProcessInterval: Using adaptive capacity limit="
                     << bwe_based_limit.bps() << " bps";

    // Use delay-based estimate as the primary capacity indicator, but don't exceed adaptive capacity
    if (last_delay_based_estimate_ > DataRate::Zero()) {
      // Use 95% consistently (same as feedback processing) to avoid rate jumping
      DataRate delay_based_limit = std::min(last_delay_based_estimate_ * 0.95, adaptive_max_capacity);
      
      RTC_LOG(LS_INFO) << "L4S OnProcessInterval: Using delay-based limit="
                       << delay_based_limit.bps() << " bps (95% of "
                       << last_delay_based_estimate_.bps() << " bps, capped by adaptive_max="
                       << adaptive_max_capacity.bps() << " bps)";
      
      bwe_based_limit = delay_based_limit;
      
      // Update capacity estimator with sustained rate information
      capacity_estimator_->UpdateFromSustainedRate(last_delay_based_estimate_);
    } else {
      // No delay estimate yet, use a conservative portion of adaptive capacity
      bwe_based_limit = adaptive_max_capacity * 0.1;  // Start with 10% of adaptive max
      RTC_LOG(LS_INFO) << "L4S OnProcessInterval: No delay estimate, using conservative limit="
                       << bwe_based_limit.bps() << " bps (10% of adaptive_max)";
    }

    // Disable acknowledged rate limit for faster ramp-up - delay-based BWE is a
    // better capacity indicator Only apply acknowledged rate limit if it's MUCH
    // higher than delay estimate (rare case)
    if (last_acknowledged_rate_ > DataRate::Zero() &&
        last_acknowledged_rate_ >
            last_delay_based_estimate_ *
                5) {  // Very high threshold to rarely trigger
      DataRate acked_limit =
          last_acknowledged_rate_ * 1.5;  // 50% above acked rate
      bwe_based_limit = std::min(bwe_based_limit, acked_limit);
      RTC_LOG(LS_WARNING)
          << "L4S OnProcessInterval: Applied acknowledged rate limit="
          << acked_limit.bps() << " (150% of " << last_acknowledged_rate_.bps()
          << "), bwe_based_limit now=" << bwe_based_limit.bps();
    }

    // RTC_LOG(LS_INFO)
    //     << "L4S OnProcessInterval: Final BWE-based capacity limit: "
    //     << bwe_based_limit.bps() << " bps";

    // Allow Prague to increase up to BWE-based limit, even if current rate is
    // lower This prevents getting stuck when current rate is below the
    // available capacity Made more aggressive: 99.5% instead of 98% for faster
    // ramp-up and better video quality
    DataRate rate_for_prague =
        std::min(std::max(current_rate_for_transport_, bwe_based_limit * 0.995),
                 bwe_based_limit);
    if (rate_for_prague > current_rate_for_transport_) {
      // Smooth large rate increases to prevent system instability
      DataRate rate_diff = rate_for_prague - current_rate_for_transport_;
      DataRate max_increase_per_step = current_rate_for_transport_ * 0.3;  // Max 30% increase per step
      
      if (rate_diff > max_increase_per_step) {
        DataRate smoothed_rate = current_rate_for_transport_ + max_increase_per_step;
        RTC_LOG(LS_WARNING) << "L4S: Smoothing rate increase from " << current_rate_for_transport_.bps() 
                            << " to " << smoothed_rate.bps() << " bps instead of " 
                            << rate_for_prague.bps() << " bps (BWE limit: " << bwe_based_limit.bps() << " bps)";
        rate_for_prague = smoothed_rate;
      } else {
        RTC_LOG(LS_WARNING) << "L4S: Allowing Prague to target higher rate "
                            << rate_for_prague.bps() << " bps instead of current "
                            << current_rate_for_transport_.bps()
                            << " bps (BWE limit: " << bwe_based_limit.bps()
                            << " bps)";
      }
    }

    // RTC_LOG(LS_WARNING)
    //     << "L4S OnProcessInterval DEBUG: Passing rate_for_prague="
    //     << rate_for_prague.bps()
    //     << " to Prague (current=" << current_rate_for_transport_.bps()
    //     << ", bwe_limit=" << bwe_based_limit.bps() << ")";

    auto prague_rate =
        prague_controller_->GetTargetRate(msg.at_time, rate_for_prague);
    if (prague_rate) {
      // CRITICAL: Validate Prague controller output before using it
      if (!prague_rate->IsFinite() || prague_rate->bps() <= 0) {
        RTC_LOG(LS_WARNING) << "L4S: Prague controller returned invalid rate " 
                            << prague_rate->bps() << " bps in OnProcessInterval, using fallback";
        prague_rate = DataRate::KilobitsPerSec(300);
      }

      // CRITICAL: Cap Prague rate immediately to prevent any potential overflow/infinite values
      if (prague_rate.value() > bwe_based_limit) {
        // RTC_LOG(LS_WARNING) << "L4S: Prague suggested rate " << prague_rate.value().bps()
        //                     << " exceeds BWE-informed limit, capping to " << bwe_based_limit.bps();
        
        // Validate the capped rate before assignment
        if (bwe_based_limit.IsFinite() && bwe_based_limit > DataRate::Zero()) {
          prague_rate = bwe_based_limit;
          target_rate_ = prague_rate;
          last_target_bitrate_ = prague_rate.value();  // Track target bitrate for metrics
        } else {
          RTC_LOG(LS_WARNING) << "L4S: Invalid BWE limit " << bwe_based_limit.bps() 
                              << " bps, using safe fallback";
          prague_rate = DataRate::KilobitsPerSec(300);
          target_rate_ = prague_rate;
          last_target_bitrate_ = prague_rate.value();  // Track target bitrate for metrics
        }
        
        // Force a probe to discover if higher rates are actually available
        DataRate probe_min = min_target_rate_.value_or(DataRate::KilobitsPerSec(30));
        DataRate probe_start = target_rate_.value();
        DataRate probe_max = target_rate_.value() * 1.5;  // Probe up to 50% above current rate
        
        // Fix for low-rate deadlock: If start rate is below min, adjust min downward
        if (probe_start < probe_min) {
          probe_min = std::max(probe_start, DataRate::KilobitsPerSec(10));  // Never go below 10 kbps
          RTC_LOG(LS_WARNING) << "L4S: Adjusting probe min from " << min_target_rate_.value_or(DataRate::KilobitsPerSec(30)).bps()
                              << " to " << probe_min.bps() << " bps to match start rate " << probe_start.bps() << " bps";
        }
        
        // Validate probe rates before calling SetBitrates
        if (probe_min.IsFinite() && probe_start.IsFinite() && probe_max.IsFinite() &&
            probe_min > DataRate::Zero() && probe_start > DataRate::Zero() && probe_max > DataRate::Zero() &&
            probe_min <= probe_start && probe_start <= probe_max) {
          
          auto forced_probe_clusters = probe_controller_->SetBitrates(probe_min, probe_start, probe_max, msg.at_time);
          
          if (!forced_probe_clusters.empty()) {
            update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                               forced_probe_clusters.begin(), forced_probe_clusters.end());
            // RTC_LOG(LS_WARNING) << "L4S: Forced probes to discover capacity above " << target_rate_->bps() << " bps";
          }
        } else {
          RTC_LOG(LS_WARNING) << "L4S: Invalid probe rates - min=" << probe_min.bps() 
                              << " start=" << probe_start.bps() << " max=" << probe_max.bps() << " bps";
        }
      } else {
        // Prague rate is within BWE limit, validate and set it
        if (prague_rate.value().IsFinite() && prague_rate.value() > DataRate::Zero()) {
          target_rate_ = prague_rate;
          last_target_bitrate_ = prague_rate.value();  // Track target bitrate for metrics
        } else {
          RTC_LOG(LS_WARNING) << "L4S: Invalid Prague rate " << prague_rate.value().bps() 
                              << " bps, using safe fallback";
          target_rate_ = DataRate::KilobitsPerSec(300);
          last_target_bitrate_ = DataRate::KilobitsPerSec(300);  // Track target bitrate for metrics
        }
      }
      MaybeTriggerOnNetworkChanged(&update, msg.at_time);
    }
    
    // Final safety check: ensure target_rate_ is always valid before proceeding
    if (target_rate_.has_value() && 
        (!target_rate_->IsFinite() || target_rate_->bps() <= 0)) {
      RTC_LOG(LS_WARNING) << "L4S: Invalid final target_rate_ " << target_rate_->bps() 
                          << " bps, resetting to safe fallback";
      target_rate_ = DataRate::KilobitsPerSec(300);
      last_target_bitrate_ = DataRate::KilobitsPerSec(300);  // Track target bitrate for metrics
    }
    
    // Check if we're in ALR (Application Limited Region) to enable probing
    // ALR occurs when we're sending below our estimated capacity
    DataRate current_sending_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
    DataRate estimated_capacity = std::max(last_delay_based_estimate_, last_acknowledged_rate_);
    
    // Simplified ALR detection with hysteresis to prevent flickering
    bool should_be_in_alr = false;
    if (estimated_capacity > DataRate::Zero()) {
      if (!alr_start_time_) {
        // Not in ALR - require clear signal to enter (95% threshold)
        // OR if Prague controller wants to increase but is being capped
        bool underutilizing = current_sending_rate < estimated_capacity * 0.95;
        bool prague_being_capped = false;
        
        // Check if Prague wants to increase (indicating potential underutilization)
        auto test_prague_rate = prague_controller_->GetTargetRate(msg.at_time, current_sending_rate * 1.1);
        if (test_prague_rate && test_prague_rate.value() > current_sending_rate * 1.05) {
          prague_being_capped = true;
          // RTC_LOG(LS_WARNING) << "L4S: Prague wants to increase from " << current_sending_rate.bps() 
          //                     << " to " << test_prague_rate.value().bps() << " - triggering ALR for probing";
        }
        
        should_be_in_alr = underutilizing || prague_being_capped;
      } else {
        // In ALR - require stronger signal to exit (98% threshold for hysteresis)
        if (current_sending_rate < estimated_capacity * 0.98) {
          should_be_in_alr = true;
        }
      }
    }
    
    // Update ALR state
    if (should_be_in_alr && !alr_start_time_) {
      alr_start_time_ = msg.at_time;
      probe_controller_->SetAlrStartTimeMs(msg.at_time.ms());
      // RTC_LOG(LS_WARNING) << "L4S: Entered ALR state - sending " << current_sending_rate.bps() 
                          // << " bps < 95% of capacity " << estimated_capacity.bps() << " bps";
    } else if (!should_be_in_alr && alr_start_time_) {
      probe_controller_->SetAlrEndedTimeMs(msg.at_time.ms());
      alr_start_time_.reset();
      // RTC_LOG(LS_WARNING) << "L4S: Exited ALR state - sending " << current_sending_rate.bps() 
                          // << " bps >= 98% of capacity " << estimated_capacity.bps() << " bps";
    }
    
    // Log ALR state periodically for debugging
    static Timestamp last_alr_log = Timestamp::Zero();
    if (msg.at_time - last_alr_log > TimeDelta::Seconds(3)) {  // More frequent ALR logging
      last_alr_log = msg.at_time;
      // RTC_LOG(LS_WARNING) << "L4S ALR Debug: should_be_in_alr=" << (should_be_in_alr ? "yes" : "no")
      //                     << " alr_active=" << (alr_start_time_.has_value() ? "yes" : "no")
      //                     << " sending=" << current_sending_rate.bps() << " bps"
      //                     << " capacity=" << estimated_capacity.bps() << " bps"
      //                     << " threshold_95%=" << (estimated_capacity * 0.95).bps() << " bps"
      //                     << " threshold_98%=" << (estimated_capacity * 0.98).bps() << " bps"
      //                     << " delay_est=" << last_delay_based_estimate_.bps() << " bps"
      //                     << " acked_est=" << last_acknowledged_rate_.bps() << " bps";
    }
    
    // Update ProbeController with our current bandwidth estimate and network state
    // This helps it decide when and at what rates to probe
    if (target_rate_.has_value()) {
      BandwidthLimitedCause cause = BandwidthLimitedCause::kDelayBasedLimited;
      
      // Determine the limiting factor to inform ProbeController's decision making
      if (last_acknowledged_rate_ > DataRate::Zero() && 
          target_rate_->bps() <= last_acknowledged_rate_.bps() * 1.1) {
        // We're limited by acknowledgment-based rate (likely loss or ack rate)
        cause = BandwidthLimitedCause::kLossLimitedBwe;
      }
      
      // Set network state estimate to help ProbeController make better decisions
      // Use the higher of delay-based and acknowledged rate as the current estimate
      DataRate network_estimate = std::max(last_delay_based_estimate_, last_acknowledged_rate_);
      if (network_estimate > DataRate::Zero()) {
        NetworkStateEstimate state_estimate;
        state_estimate.update_time = msg.at_time;
        state_estimate.link_capacity = network_estimate;
        state_estimate.link_capacity_lower = network_estimate * 0.8;
        state_estimate.link_capacity_upper = network_estimate * 1.5;
        state_estimate.propagation_delay = TimeDelta::Millis(25);  // Default propagation delay
        state_estimate.confidence = 0.8;
        
        probe_controller_->SetNetworkStateEstimate(state_estimate);
        // RTC_LOG(LS_WARNING) << "L4S: SetNetworkStateEstimate called with rate=" << network_estimate.bps() << " bps";
      }
      
      auto estimate_probe_clusters = probe_controller_->SetEstimatedBitrate(
          *target_rate_, cause, msg.at_time);
      
      // RTC_LOG(LS_WARNING) << "L4S: SetEstimatedBitrate called with rate=" << target_rate_->bps() 
      //                     << " bps, cause=" << static_cast<int>(cause)
      //                     << " returned " << estimate_probe_clusters.size() << " probe(s)";
      
      if (!estimate_probe_clusters.empty()) {
        // Add any immediate probes triggered by the estimate update
        if (update.probe_cluster_configs.empty()) {
          update.probe_cluster_configs = std::move(estimate_probe_clusters);
        } else {
          // Merge with existing probes from Process() call
          update.probe_cluster_configs.insert(
              update.probe_cluster_configs.end(),
              std::make_move_iterator(estimate_probe_clusters.begin()),
              std::make_move_iterator(estimate_probe_clusters.end()));
        }
        
        RTC_LOG(LS_WARNING) << "L4S: Estimate update triggered " << estimate_probe_clusters.size() 
                            << " additional probe(s) at " << target_rate_->bps() << " bps";
      }
    }

    // Use ProbeController for bandwidth discovery
    // RTC_LOG(LS_WARNING) << "L4S: Calling ProbeController::Process at time " << msg.at_time.ms() << " ms";
    auto probe_clusters = probe_controller_->Process(msg.at_time);
    if (!probe_clusters.empty()) {
      update.probe_cluster_configs = std::move(probe_clusters);
      
      RTC_LOG(LS_WARNING) << "L4S: ProbeController initiated " << update.probe_cluster_configs.size() 
                          << " probe cluster(s) at time " << msg.at_time.ms() << " ms";
      for (const auto& probe : update.probe_cluster_configs) {
        RTC_LOG(LS_WARNING) << "L4S: Probe cluster id=" << probe.id 
                            << " target_rate=" << probe.target_data_rate.bps() << " bps"
                            << " duration=" << probe.target_duration.ms() << " ms"
                            << " count=" << probe.target_probe_count;
      }
      
      // Log probe context for debugging
      if (alr_start_time_) {
        auto alr_duration = msg.at_time - *alr_start_time_;
        RTC_LOG(LS_WARNING) << "L4S: Probing while in ALR for " << alr_duration.ms() << " ms";
      } else {
        RTC_LOG(LS_WARNING) << "L4S: Probing while not in ALR state";
      }
    } else {
      // Log why no probes are being generated (every 3 seconds for debugging)
      static Timestamp last_no_probe_log = Timestamp::Zero();
      static Timestamp last_forced_probe = Timestamp::Zero();
      if (msg.at_time - last_no_probe_log > TimeDelta::Seconds(3)) {
        last_no_probe_log = msg.at_time;
        RTC_LOG(LS_WARNING) << "L4S: No probes generated - ALR=" << (alr_start_time_.has_value() ? "yes" : "no")
                            << " sending_rate=" << current_sending_rate.bps() << " bps"
                            << " estimated_capacity=" << estimated_capacity.bps() << " bps"
                            << " target_rate=" << (target_rate_.has_value() ? target_rate_->bps() : -1) << " bps"
                            << " time=" << msg.at_time.ms() << " ms";
                            
        // Force a probe every 10 seconds if we haven't seen any probes for debugging
        if (msg.at_time - last_forced_probe > TimeDelta::Seconds(10) && target_rate_.has_value()) {
          last_forced_probe = msg.at_time;
          
          RTC_LOG(LS_WARNING) << "L4S: Attempting to force probe via multiple methods";
          
          // Method 1: Try SetBitrates with higher max bitrate
          DataRate current_max = max_bitrate_;
          DataRate probe_max = std::max(target_rate_.value() * 1.5, DataRate::KilobitsPerSec(3000));
          DataRate probe_min = min_target_rate_.value_or(DataRate::KilobitsPerSec(30));
          DataRate probe_start = target_rate_.value();
          
          // Fix for low-rate deadlock: If start rate is below min, adjust min downward
          if (probe_start < probe_min) {
            probe_min = std::max(probe_start, DataRate::KilobitsPerSec(10));  // Never go below 10 kbps
            RTC_LOG(LS_WARNING) << "L4S: Force probe - adjusting min from " << min_target_rate_.value_or(DataRate::KilobitsPerSec(30)).bps()
                                << " to " << probe_min.bps() << " bps to match start rate " << probe_start.bps() << " bps";
          }
          
          // Validate probe rate to prevent invalid values
          if (probe_max > DataRate::KilobitsPerSec(100000)) {
            RTC_LOG(LS_WARNING) << "L4S: Probe rate too high (" << probe_max.bps() 
                                << " bps), capping to 100 Mbps";
            probe_max = DataRate::KilobitsPerSec(100000);
          }
          
          // Temporarily update max_bitrate_ to allow higher probing
          max_bitrate_ = probe_max;
          
          auto forced_probe_clusters = probe_controller_->SetBitrates(
              probe_min,
              probe_start,
              probe_max,
              msg.at_time);
          
          // Restore original max_bitrate_
          max_bitrate_ = current_max;
          
          if (!forced_probe_clusters.empty()) {
            update.probe_cluster_configs = std::move(forced_probe_clusters);
            RTC_LOG(LS_WARNING) << "L4S: Forced " << update.probe_cluster_configs.size() 
                                << " probe(s) via SetBitrates with max=" << probe_max.bps() << " bps";
          } else {
            // Method 2: Try RequestProbe directly
            auto direct_probes = probe_controller_->RequestProbe(msg.at_time);
            if (!direct_probes.empty()) {
              update.probe_cluster_configs = std::move(direct_probes);
              RTC_LOG(LS_WARNING) << "L4S: Forced " << update.probe_cluster_configs.size() 
                                  << " probe(s) via RequestProbe";
            } else {
              // Method 3: Force ALR state to trigger probing
              if (!alr_start_time_) {
                alr_start_time_ = msg.at_time;
                probe_controller_->SetAlrStartTimeMs(msg.at_time.ms());
                RTC_LOG(LS_WARNING) << "L4S: Forced ALR state to trigger probing";
                
                // Try Process() again after setting ALR
                auto alr_probes = probe_controller_->Process(msg.at_time);
                if (!alr_probes.empty()) {
                  update.probe_cluster_configs = std::move(alr_probes);
                  RTC_LOG(LS_WARNING) << "L4S: ALR-triggered " << update.probe_cluster_configs.size() << " probe(s)";
                }
              } else {
                RTC_LOG(LS_WARNING) << "L4S: Failed to force probes with all methods";
              }
            }
          }
        }
      }
    }
  } else if (fallback_to_gcc_) {
    // Forward to GCC if we're not using L4S
    update = gcc_controller_->OnProcessInterval(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate L4SNetworkController::OnRemoteBitrateReport(
    webrtc::RemoteBitrateReport msg) {
  webrtc::NetworkControlUpdate update;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnRemoteBitrateReport(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate L4SNetworkController::OnRoundTripTimeUpdate(
    webrtc::RoundTripTimeUpdate msg) {
  webrtc::NetworkControlUpdate update;

  // Update Prague controller
  prague_controller_->UpdateRtt(msg.round_trip_time);
  
  // Update the adaptive capacity estimator with RTT information
  capacity_estimator_->UpdateFromRtt(msg.round_trip_time);

  // Update local RTT tracking for metrics
  last_rtt_ = msg.round_trip_time;
  
  // Log RTT metrics
  if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->LogDelayMetrics(
        Timestamp::Millis(env_.clock().TimeInMilliseconds()),
        msg.round_trip_time, msg.round_trip_time / 2);
  }

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    update = gcc_controller_->OnRoundTripTimeUpdate(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate L4SNetworkController::OnSentPacket(webrtc::SentPacket msg) {
  webrtc::NetworkControlUpdate update;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    update = gcc_controller_->OnSentPacket(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate L4SNetworkController::OnReceivedPacket(
    webrtc::ReceivedPacket msg) {
  webrtc::NetworkControlUpdate update;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnReceivedPacket(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate L4SNetworkController::OnStreamsConfig(webrtc::StreamsConfig msg) {
  webrtc::NetworkControlUpdate update;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    update = gcc_controller_->OnStreamsConfig(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate L4SNetworkController::OnTargetRateConstraints(
    webrtc::TargetRateConstraints msg) {
  webrtc::NetworkControlUpdate update;

  // Update constraints
  min_target_rate_ = msg.min_data_rate;
  max_target_rate_ = msg.max_data_rate;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    update = gcc_controller_->OnTargetRateConstraints(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate L4SNetworkController::OnTransportLossReport(
    webrtc::TransportLossReport msg) {
  webrtc::NetworkControlUpdate update;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnTransportLossReport(msg);
  }

  return update;
}

// In OnTransportPacketsFeedback method:
webrtc::NetworkControlUpdate L4SNetworkController::OnTransportPacketsFeedback(
    webrtc::TransportPacketsFeedback feedback) {
  webrtc::NetworkControlUpdate update;

  // Validate feedback time to prevent invalid timestamps
  if (!feedback.feedback_time.IsFinite() || feedback.feedback_time.us() < 0) {
    RTC_LOG(LS_WARNING) << "Invalid feedback time: "
                        << feedback.feedback_time.us()
                        << " us, using current time instead";
    feedback.feedback_time =
        Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }

  // Log feedback details before processing
  // RTC_LOG(LS_INFO) << "L4S OnTransportPacketsFeedback: feedback_time="
  //                  << feedback.feedback_time.us() << " us, packet_count="
  //                  << feedback.packet_feedbacks.size();

  // Process ECN feedback to detect if ECN is supported
  ProcessEcnFeedback(feedback);
  
  // Log metrics for feedback processing
  LogPeriodicMetrics(feedback.feedback_time);
  LogControllerState(feedback.feedback_time);
  
  // TODO: Handle probe results when ProbeClusterCreated events are received
  // For now, the ProbeController will handle probe result processing internally

  // GCC-style bandwidth estimation integration

  // 1. Calculate RTT metrics (like GCC does)
  TimeDelta max_feedback_rtt = TimeDelta::MinusInfinity();
  TimeDelta min_propagation_rtt = TimeDelta::PlusInfinity();
  Timestamp max_recv_time = Timestamp::MinusInfinity();

  std::vector<PacketResult> feedbacks = feedback.ReceivedWithSendInfo();
  for (const auto& fb : feedbacks)
    max_recv_time = std::max(max_recv_time, fb.receive_time);

  for (const auto& fb : feedbacks) {
    TimeDelta feedback_rtt = feedback.feedback_time - fb.sent_packet.send_time;
    TimeDelta min_pending_time = max_recv_time - fb.receive_time;
    TimeDelta propagation_rtt = feedback_rtt - min_pending_time;
    max_feedback_rtt = std::max(max_feedback_rtt, feedback_rtt);
    min_propagation_rtt = std::min(min_propagation_rtt, propagation_rtt);
  }

  // Update RTT estimates in bandwidth estimation
  if (max_feedback_rtt.IsFinite()) {
    feedback_max_rtts_.push_back(max_feedback_rtt.ms());
    const size_t kMaxFeedbackRttWindow = 32;
    if (feedback_max_rtts_.size() > kMaxFeedbackRttWindow)
      feedback_max_rtts_.pop_front();

    bandwidth_estimation_->UpdatePropagationRtt(feedback.feedback_time,
                                                min_propagation_rtt);

    // Calculate mean RTT for delay-based BWE
    if (!feedback_max_rtts_.empty()) {
      int64_t sum_rtt_ms =
          std::accumulate(feedback_max_rtts_.begin(), feedback_max_rtts_.end(),
                          static_cast<int64_t>(0));
      int64_t mean_rtt_ms = sum_rtt_ms / feedback_max_rtts_.size();
      delay_based_bwe_->OnRttUpdate(TimeDelta::Millis(mean_rtt_ms));

      // Update Prague controller with RTT
      prague_controller_->UpdateRtt(TimeDelta::Millis(mean_rtt_ms));
    }
  }

  // 2. Update acknowledged bitrate estimator
  acknowledged_bitrate_estimator_->IncomingPacketFeedbackVector(
      feedback.SortedByReceiveTime());
  auto acknowledged_bitrate = acknowledged_bitrate_estimator_->bitrate();
  if (acknowledged_bitrate) {
    last_acknowledged_rate_ = *acknowledged_bitrate;
    last_actual_bitrate_ = *acknowledged_bitrate;  // Track actual bitrate for metrics
    bandwidth_estimation_->SetAcknowledgedRate(acknowledged_bitrate,
                                               feedback.feedback_time);

    // RTC_LOG(LS_INFO) << "L4S: Acknowledged bitrate estimate: "
    //                  << acknowledged_bitrate->bps() << " bps";
  }

  // 3. Run delay-based BWE to get capacity estimate
  DelayBasedBwe::Result delay_result =
      delay_based_bwe_->IncomingPacketFeedbackVector(
          feedback, acknowledged_bitrate, std::nullopt, std::nullopt,
          false);

  if (delay_result.updated) {
    // Validate delay-based estimate before setting
    if (delay_result.target_bitrate.IsFinite() && delay_result.target_bitrate > DataRate::Zero()) {
      last_delay_based_estimate_ = delay_result.target_bitrate;
      // RTC_LOG(LS_INFO) << "L4S: Updated delay-based estimate to " << last_delay_based_estimate_.bps() << " bps";
    } else {
      RTC_LOG(LS_WARNING) << "L4S: Received invalid delay-based estimate: " 
                          << delay_result.target_bitrate.bps() << " bps, keeping previous value: "
                          << last_delay_based_estimate_.bps() << " bps";
    }
    
    bandwidth_estimation_->UpdateDelayBasedEstimate(
        feedback.feedback_time, delay_result.target_bitrate);

    // Log delay-based congestion state for debugging
    // const char* state_str = "UNKNOWN";
    // switch (delay_result.delay_detector_state) {
    //   case BandwidthUsage::kBwNormal:
    //     state_str = "NORMAL";
    //     break;
    //   case BandwidthUsage::kBwUnderusing:
    //     state_str = "UNDERUSING";
    //     break;
    //   case BandwidthUsage::kBwOverusing:
    //     state_str = "OVERUSING";
    //     break;
    //   case BandwidthUsage::kLast:
    //     state_str = "INVALID";
    //     break;
    // }

    // RTC_LOG(LS_INFO) << "L4S: Delay-based BWE estimate: "
    //                  << delay_result.target_bitrate.bps()
    //                  << " bps, state: " << state_str << ", recovered: "
    //                  << (delay_result.recovered_from_overuse ? "YES" : "NO");
                     
    // Update ProbeController with the estimated bitrate
    BandwidthLimitedCause bandwidth_limited_cause = BandwidthLimitedCause::kDelayBasedLimited;
    if (delay_result.delay_detector_state == BandwidthUsage::kBwOverusing) {
      bandwidth_limited_cause = BandwidthLimitedCause::kDelayBasedLimitedDelayIncreased;
    }
    
    auto probe_clusters = probe_controller_->SetEstimatedBitrate(
        delay_result.target_bitrate, bandwidth_limited_cause, feedback.feedback_time);
    
    // RTC_LOG(LS_WARNING) << "L4S: Feedback SetEstimatedBitrate called with rate=" << delay_result.target_bitrate.bps() 
    //                     << " bps, cause=" << static_cast<int>(bandwidth_limited_cause)
    //                     << " returned " << probe_clusters.size() << " probe(s)";
    
    // Create and set a network state estimate to help ProbeController make decisions
    NetworkStateEstimate network_estimate;
    network_estimate.update_time = feedback.feedback_time;
    network_estimate.link_capacity = delay_result.target_bitrate;
    network_estimate.link_capacity_lower = delay_result.target_bitrate * 0.8;  // Conservative lower bound
    network_estimate.link_capacity_upper = delay_result.target_bitrate * 1.5;  // Optimistic upper bound
    network_estimate.propagation_delay = last_estimated_round_trip_time_.IsFinite() ? 
                                         last_estimated_round_trip_time_ / 2 : 
                                         TimeDelta::Millis(25);  // Half RTT for propagation delay
    network_estimate.confidence = 0.8;  // Reasonably confident in L4S BWE
    
    probe_controller_->SetNetworkStateEstimate(network_estimate);
    // RTC_LOG(LS_WARNING) << "L4S: Set network state estimate - link_capacity=" << network_estimate.link_capacity.bps() 
    //                     << " bps, lower=" << network_estimate.link_capacity_lower.bps() 
    //                     << " bps, upper=" << network_estimate.link_capacity_upper.bps() << " bps";
        
    if (!probe_clusters.empty()) {
      // Add probes to the update - will be merged with any existing probes
      update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                         probe_clusters.begin(), probe_clusters.end());
      RTC_LOG(LS_WARNING) << "L4S: BWE update triggered " << probe_clusters.size() << " probe(s)";
    }
  }

  // 4. Update our network capacity estimate based on BWE results
  DataRate bwe_estimate = bandwidth_estimation_->target_rate();
  // RTC_LOG(LS_WARNING)
  //     << "L4S BWE Source Debug: bandwidth_estimation_->target_rate()="
  //     << bwe_estimate.bps()
  //     << " bps, max_realistic_bandwidth_=" << max_realistic_bandwidth_.bps()
  //     << " bps";

  // Allow max_realistic_bandwidth_ to increase if we observe higher bandwidth
  DataRate highest_observed_rate = DataRate::Zero();
  
  // Consider BWE estimate
  if (bwe_estimate > DataRate::Zero() && bwe_estimate.IsFinite()) {
    highest_observed_rate = std::max(highest_observed_rate, bwe_estimate);
  }
  
  // Consider delay-based estimate
  if (!last_delay_based_estimate_.IsZero() && last_delay_based_estimate_.IsFinite()) {
    highest_observed_rate = std::max(highest_observed_rate, last_delay_based_estimate_);
  }
  
  // Consider acknowledged bitrate
  if (!last_acknowledged_rate_.IsZero() && last_acknowledged_rate_.IsFinite()) {
    highest_observed_rate = std::max(highest_observed_rate, last_acknowledged_rate_);
  }
  
  // Update max_realistic_bandwidth_ if we observe significantly higher bandwidth
  if (highest_observed_rate > max_realistic_bandwidth_ * 1.2) {
    DataRate new_max = highest_observed_rate * 1.1; // Add 10% headroom
    RTC_LOG(LS_WARNING) << "L4S: Increasing max_realistic_bandwidth_ from "
                        << max_realistic_bandwidth_.bps() << " to " 
                        << new_max.bps() << " bps based on observed rate "
                        << highest_observed_rate.bps() << " bps";
    max_realistic_bandwidth_ = new_max;
  }

  // CRITICAL: Ensure max_realistic_bandwidth_ doesn't get stuck below
  // reasonable minimums
  DataRate minimum_bandwidth = DataRate::KilobitsPerSec(1000);  // 1 Mbps minimum
  if (max_realistic_bandwidth_ < minimum_bandwidth) {
    if (!last_delay_based_estimate_.IsZero() && last_delay_based_estimate_ > minimum_bandwidth) {
      max_realistic_bandwidth_ = last_delay_based_estimate_ * 1.1;  // Use delay estimate with headroom
      RTC_LOG(LS_WARNING) << "L4S: Boosting stuck max_realistic_bandwidth_ to "
                          << max_realistic_bandwidth_.bps()
                          << " bps based on delay estimate "
                          << last_delay_based_estimate_.bps() << " bps";
    } else {
      max_realistic_bandwidth_ = minimum_bandwidth;
      RTC_LOG(LS_WARNING) << "L4S: Setting max_realistic_bandwidth_ to minimum "
                          << max_realistic_bandwidth_.bps() << " bps";
    }
  }

  // Update our estimate of network capacity based on congestion signals
  UpdateNetworkCapacityEstimate(feedback);

  // Log before updating Prague controller
  RTC_LOG(LS_INFO) << "L4S calling Prague UpdateEcnFeedback with " 
                   << feedback.packet_feedbacks.size() << " packets";

  // Update Prague controller with ECN feedback
  prague_controller_->UpdateEcnFeedback(feedback);

  // Get updated target rate if L4S is active
  if (IsL4SActive()) {
    RTC_LOG(LS_INFO)
        << "L4S is active, getting target rate from Prague controller";
    DataRate current_rate_for_transport_ =
        target_rate_.value_or(DataRate::KilobitsPerSec(300));

    // Consider BWE estimates when determining capacity limits
    DataRate bwe_based_limit = max_realistic_bandwidth_;

    // Use delay-based estimate intelligently, but respect max_realistic_bandwidth_ as hard limit
    if (!last_delay_based_estimate_.IsFinite() || last_delay_based_estimate_.bps() < 0) {
      RTC_LOG(LS_WARNING) << "L4S: Invalid delay-based estimate " 
                          << last_delay_based_estimate_.bps() << " bps, using conservative limit";
      bwe_based_limit = max_realistic_bandwidth_ * 0.1;  // Conservative 10% of max
    } else {
      // Use delay-based estimate but cap it at max_realistic_bandwidth_
      DataRate delay_based_limit = std::min(last_delay_based_estimate_ * 0.98, max_realistic_bandwidth_);
      bwe_based_limit = delay_based_limit;
      
      // RTC_LOG(LS_WARNING) << "L4S OnTransportFeedback: Using delay-based limit="
      //                     << delay_based_limit.bps() << " bps (98% of "
      //                     << last_delay_based_estimate_.bps() << " bps, capped by max_realistic="
      //                     << max_realistic_bandwidth_.bps() << " bps)";
    }

    // RTC_LOG(LS_INFO) << "L4S BWE Debug: max_realistic_bandwidth_="
    //                  << max_realistic_bandwidth_.bps()
    //                  << ", last_delay_based_estimate_="
    //                  << last_delay_based_estimate_.bps()
    //                  << ", last_acknowledged_rate_="
    //                  << last_acknowledged_rate_.bps();

    // Use delay-based estimate as capacity indicator, but keep it simple
    if (last_delay_based_estimate_ > DataRate::Zero()) {
      // Use a conservative portion of delay estimate, capped by max_realistic_bandwidth_
      DataRate delay_limit = std::min(last_delay_based_estimate_ * 0.95, max_realistic_bandwidth_);
      bwe_based_limit = delay_limit;
      
      // RTC_LOG(LS_INFO) << "L4S: Using delay-based limit=" << delay_limit.bps() 
      //                  << " bps (95% of " << last_delay_based_estimate_.bps() 
      //                  << " bps, max_realistic=" << max_realistic_bandwidth_.bps() << " bps)";
    } else {
      // No delay estimate, use conservative limit
      bwe_based_limit = max_realistic_bandwidth_ * 0.1;
      // RTC_LOG(LS_INFO) << "L4S: No delay estimate, using conservative limit=" 
      //                  << bwe_based_limit.bps() << " bps";
    }

    // Disable acknowledged rate limit for faster ramp-up - delay-based BWE is a
    // better capacity indicator Only apply acknowledged rate limit if it's MUCH
    // higher than delay estimate (rare case)
    if (last_acknowledged_rate_ > DataRate::Zero() &&
        last_acknowledged_rate_ >
            last_delay_based_estimate_ *
                5) {  // Very high threshold to rarely trigger
      DataRate acked_limit =
          last_acknowledged_rate_ * 1.5;  // 50% above acked rate
      bwe_based_limit = std::min(bwe_based_limit, acked_limit);
      RTC_LOG(LS_INFO) << "L4S BWE Debug: Applied acknowledged rate limit="
                       << acked_limit.bps() << " (150% of "
                       << last_acknowledged_rate_.bps() << ")";
    }

    // RTC_LOG(LS_INFO) << "L4S: BWE-based capacity limit: "
    //                  << bwe_based_limit.bps()
    //                  << " bps (acked: " << last_acknowledged_rate_.bps()
    //                  << ", delay: " << last_delay_based_estimate_.bps() << ")";

    // Allow Prague to increase up to BWE-based limit, even if current rate is
    // lower This prevents getting stuck when current rate is below the
    // available capacity Made more aggressive: 99.5% instead of 98% for faster
    // ramp-up and better video quality
    DataRate rate_for_prague = std::min(
        std::max(current_rate_for_transport_, bwe_based_limit * 0.995), bwe_based_limit);
    if (rate_for_prague > current_rate_for_transport_) {
      RTC_LOG(LS_WARNING) << "L4S: Allowing Prague to target higher rate "
                          << rate_for_prague.bps() << " bps instead of current "
                          << current_rate_for_transport_.bps()
                          << " bps (BWE limit: " << bwe_based_limit.bps()
                          << " bps)";
    }

    auto prague_rate = prague_controller_->GetTargetRate(feedback.feedback_time,
                                                         rate_for_prague);
    if (prague_rate) {
      // CRITICAL: Validate Prague controller output before using it
      if (!prague_rate->IsFinite() || prague_rate->bps() <= 0) {
        RTC_LOG(LS_WARNING) << "L4S: Prague controller returned invalid rate " 
                            << prague_rate->bps() << " bps, using fallback";
        prague_rate = DataRate::KilobitsPerSec(300);
      }

      // Double-check Prague's output against BWE-informed bandwidth limits
      if (prague_rate.value() > bwe_based_limit) {
        RTC_LOG(LS_WARNING)
            << "L4S: Prague suggested rate " << prague_rate.value().bps()
            << " exceeds BWE-informed limit, capping to "
            << bwe_based_limit.bps();
        prague_rate = bwe_based_limit;

        // CRITICAL: Since we're capping, set target_rate_ to the capped value
        // so Prague gets the actual rate being used as input for the next
        // calculation
        target_rate_ = prague_rate;
      } else {
        target_rate_ = prague_rate;
      }

      // RTC_LOG(LS_INFO) << "L4S got target rate: " << prague_rate->bps()
      //                  << " bps";
      // RTC_LOG(LS_INFO) << "L4S setting target_rate_ from "
      //                  << (target_rate_ ? target_rate_->bps() : -1) << " to "
      //                  << prague_rate->bps() << " bps";
      MaybeTriggerOnNetworkChanged(&update, feedback.feedback_time);
    } else {
      RTC_LOG(LS_INFO) << "L4S Prague controller returned no target rate";
    }
  } else if (fallback_to_gcc_) {
    RTC_LOG(LS_INFO) << "L4S not active, forwarding to GCC";
    // Forward to GCC if we're not using L4S
    update = gcc_controller_->OnTransportPacketsFeedback(feedback);
  }

  return update;
}

webrtc::NetworkControlUpdate L4SNetworkController::OnNetworkStateEstimate(
    webrtc::NetworkStateEstimate msg) {
  webrtc::NetworkControlUpdate update;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnNetworkStateEstimate(msg);
  }

  return update;
}

// In CreateRateUpdate method:
webrtc::NetworkControlUpdate L4SNetworkController::CreateRateUpdate(
    webrtc::Timestamp at_time) const {
  webrtc::NetworkControlUpdate update;

  // Log input timestamp value before IsFinite check
  // RTC_LOG(LS_INFO) << "L4S CreateRateUpdate: input at_time.us()=" <<
  // at_time.us()
  //                  << " at_time.ms()=" << at_time.ms()
  //                  << " IsFinite()=" << at_time.IsFinite();

  // Add timestamp validation here (similar to what GoogCC does)
  if (!at_time.IsFinite()) {
    RTC_LOG(LS_WARNING)
        << "Invalid timestamp in L4S controller, using current time";
    int64_t current_time_ms = env_.clock().TimeInMilliseconds();
    // RTC_LOG(LS_INFO) << "L4S CreateRateUpdate: current_time_ms=" <<
    // current_time_ms;
    at_time = Timestamp::Millis(current_time_ms);
    // RTC_LOG(LS_INFO) << "L4S CreateRateUpdate: new at_time.us()=" <<
    // at_time.us();
  }

  // Apply rate constraints
  DataRate current_rate_for_transport_ = target_rate_.value_or(DataRate::KilobitsPerSec(300));

  // CRITICAL: Add validation to prevent IsFinite() crashes
  if (!current_rate_for_transport_.IsFinite() || current_rate_for_transport_.bps() <= 0) {
    RTC_LOG(LS_WARNING) << "L4S: Invalid target rate " << current_rate_for_transport_.bps() 
                        << " bps, using fallback";
    current_rate_for_transport_ = DataRate::KilobitsPerSec(300);
  }

  // RTC_LOG(LS_INFO) << "L4S CreateRateUpdate: target_rate_stored="
  //                  << (target_rate_ ? target_rate_->bps() : -1) << " bps"
  //                  << ", current_rate=" << current_rate_for_transport_.bps() << " bps";

  if (min_target_rate_ && current_rate_for_transport_ < *min_target_rate_) {
    // RTC_LOG(LS_INFO) << "L4S applying min_target_rate: " <<
    // min_target_rate_->bps() << " bps";
    current_rate_for_transport_ = *min_target_rate_;
  }
  if (max_target_rate_ && current_rate_for_transport_ > *max_target_rate_) {
    // RTC_LOG(LS_INFO) << "L4S applying max_target_rate: " <<
    // max_target_rate_->bps() << " bps";
    current_rate_for_transport_ = *max_target_rate_;
  }

  // Log final rate before creating update objects
  // RTC_LOG(LS_INFO) << "L4S final current_rate_bps=" << current_rate.bps();

  // Set target rate and bandwidth estimate
  update.target_rate = TargetTransferRate();
  update.target_rate->at_time = at_time;
  update.target_rate->network_estimate.at_time = at_time;
  update.target_rate->network_estimate.bandwidth = current_rate_for_transport_;
  update.target_rate->network_estimate.loss_rate_ratio = 0.0f;  // L4S should have minimal loss
  update.target_rate->network_estimate.round_trip_time = last_estimated_round_trip_time_.IsFinite() ?
                                                        last_estimated_round_trip_time_ :
                                                        TimeDelta::Millis(50);
  update.target_rate->network_estimate.bwe_period = TimeDelta::Millis(500);

  update.target_rate->target_rate = current_rate_for_transport_ ;

  // Set pacer config
  // RTC_LOG(LS_INFO) << "L4S before creating PacerConfig";
  update.pacer_config = PacerConfig();
  update.pacer_config->at_time = at_time;

  // Log before setting pacer config values
  // RTC_LOG(LS_INFO) << "L4S before setting pacer config with current_rate: "
  // << current_rate.bps() << " bps";

  // Set time window (e.g., 10ms)
  TimeDelta time_window = TimeDelta::Millis(10);
  // RTC_LOG(LS_INFO) << "L4S setting time_window: " << time_window.ms() << "
  // ms";
  update.pacer_config->time_window = time_window;

  // Calculate data window based on current rate
  // CRITICAL: Add validation before multiplication to prevent IsFinite() crash
  if (!current_rate_for_transport_.IsFinite() || !time_window.IsFinite() || 
      current_rate_for_transport_.bps() <= 0 || time_window.us() <= 0) {
    RTC_LOG(LS_WARNING) << "L4S: Invalid values for data window calculation - rate: " 
                        << current_rate_for_transport_.bps() << " bps, time: " 
                        << time_window.us() << " us";
    // Use safe fallback values
    current_rate_for_transport_ = DataRate::KilobitsPerSec(300);
    time_window = TimeDelta::Millis(10);
  }

  DataSize data_window = current_rate_for_transport_ * time_window;

  // RTC_LOG(LS_INFO) << "L4S POST-DATA-WINDOW-MULTIPLY: data_window.bytes()="
  // << data_window.bytes()
  //                  << ", data_window.IsFinite()=" << (data_window.IsFinite()
  //                  ? "true" : "false");
  update.pacer_config->data_window = data_window;

  // Set pad window to zero
  // RTC_LOG(LS_INFO) << "L4S setting pad_window to zero";
  update.pacer_config->pad_window = DataSize::Zero();

  return update;
}

// In MaybeTriggerOnNetworkChanged method:
void L4SNetworkController::MaybeTriggerOnNetworkChanged(
    NetworkControlUpdate* update,
    Timestamp at_time) {
  // Add timestamp validation
  if (!at_time.IsFinite() || at_time.us() < 0) {
    RTC_LOG(LS_WARNING)
        << "Invalid timestamp in network change, using current time";
    at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }

  // Log timestamp values before calling CreateRateUpdate
  // RTC_LOG(LS_INFO) << "L4S MaybeTriggerOnNetworkChanged: at_time_us=" <<
  // at_time.us()
  //                  << ", IsFinite=" << (at_time.IsFinite() ? "true" :
  //                  "false");

  // Create rate update (without the conditional, since your code didn't define
  // the variables) RTC_LOG(LS_INFO) << "L4S calling CreateRateUpdate";
  NetworkControlUpdate rate_update = CreateRateUpdate(at_time);

  // Log after CreateRateUpdate completes
  // RTC_LOG(LS_INFO) << "L4S CreateRateUpdate completed successfully";

  // Copy values from rate_update to update
  if (rate_update.pacer_config) {
    update->pacer_config = rate_update.pacer_config;
  }
  if (rate_update.target_rate) {
    update->target_rate = rate_update.target_rate;
  }
}

bool L4SNetworkController::IsL4SActive() const {
  // L4S is active if:
  // 1. Prague controller is active (received enough ECN feedback)
  // 2. ECN is supported by the connection
  // 3. The network appears to be ECN capable
  return prague_controller_->IsActive() && ecn_supported_ &&
         ecn_capable_network_;
}

void L4SNetworkController::ProcessEcnFeedback(
    const TransportPacketsFeedback& feedback) {
  if (feedback.packet_feedbacks.empty()) {
    RTC_LOG(LS_WARNING) << "ProcessEcnFeedback: Empty feedback, packet_feedbacks.size()=0";
    return;
  }

  // RTC_LOG(LS_INFO) << "ProcessEcnFeedback: Processing " << feedback.packet_feedbacks.size() 
  //                  << " packets, transport_supports_ecn=" << feedback.transport_supports_ecn;

  // Count ECT and CE packets
  int new_ect_count = 0;
  int new_ce_count = 0;

  for (const auto& packet : feedback.packet_feedbacks) {
    // RTC_LOG(LS_INFO) << "ProcessEcnFeedback: Packet seq=" 
    //                  << packet.sent_packet.sequence_number
    //                  << " ECN=" << static_cast<int>(packet.ecn) << " ("
    //                  << (packet.ecn == EcnMarking::kNotEct ? "NotECT" :
    //                      (packet.ecn == EcnMarking::kEct0 ? "ECT(0)" :
    //                       (packet.ecn == EcnMarking::kEct1 ? "ECT(1)" : "CE")))
    //                  << ")";

    if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1) {
      new_ect_count++;
    } else if (packet.ecn == EcnMarking::kCe) {
      new_ce_count++;
      last_congestion_signal_ = feedback.feedback_time;
      RTC_LOG(LS_WARNING) << "ProcessEcnFeedback: CE MARK DETECTED! Count=" << (ce_count_ + new_ce_count);
    }
  }

  // RTC_LOG(LS_INFO) << "ProcessEcnFeedback: ECT count=" << new_ect_count 
  //                  << ", CE count=" << new_ce_count 
  //                  << " (total so far: ECT=" << (ect_count_ + new_ect_count)
  //                  << ", CE=" << (ce_count_ + new_ce_count) << ")";

  // If we received any ECT or CE packets, consider ECN supported
  if (new_ect_count > 0 || new_ce_count > 0) {
    ecn_supported_ = true;
  }

  // Update total counts
  ect_count_ += new_ect_count;
  ce_count_ += new_ce_count;

  // Update the adaptive capacity estimator based on congestion signals
  if (ect_count_ + ce_count_ > 0) {
    double ce_ratio = static_cast<double>(ce_count_) / (ect_count_ + ce_count_);
    DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
    capacity_estimator_->UpdateFromCongestionSignal(current_rate, ce_ratio);
  }

  // Consider the network ECN capable if we've received at least 10 packets
  // and have seen at least one CE mark or a reasonable proportion of ECT
  // packets
  if (ect_count_ + ce_count_ >= 10) {
    ecn_capable_network_ =
        (ce_count_ > 0) ||
        (static_cast<double>(ect_count_) / (ect_count_ + ce_count_) > 0.8);
  }
}

void L4SNetworkController::UpdateNetworkCapacityEstimate(
    const TransportPacketsFeedback& feedback) {
  // Simple heuristic: if we consistently get high RTTs or CE marks at certain
  // rates, update our estimate of realistic network capacity

  DataRate current_sending_rate =
      target_rate_.value_or(DataRate::KilobitsPerSec(300));

  // If we see CE marking or high RTT increases, the network might be at
  // capacity
  size_t ce_packets = 0;
  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.ecn == EcnMarking::kCe) {
      ce_packets++;
    }
  }

  if (ce_packets > 0 && current_sending_rate < max_realistic_bandwidth_) {
    // We're getting congestion signals below our assumed capacity limit
    // This might indicate the actual network capacity is lower
    DataRate new_estimate =
        current_sending_rate * 0.9;  // 10% below where we see congestion
    if (new_estimate > max_realistic_bandwidth_) {
      max_realistic_bandwidth_ = new_estimate;
      RTC_LOG(LS_INFO) << "L4S: Reducing network capacity estimate to "
                       << max_realistic_bandwidth_.bps()
                       << " bps based on congestion at "
                       << current_sending_rate.bps() << " bps";
    }
  }
}

void L4SNetworkController::ProcessProbeClusterCreated(ProbeClusterConfig probe_cluster_config) {
  // This method would be called when a probe cluster is actually created by the pacer
  // For now, we'll log it for debugging
  RTC_LOG(LS_INFO) << "L4S: Probe cluster " << probe_cluster_config.id 
                   << " created at " << probe_cluster_config.target_data_rate.bps() << " bps";
}

void L4SNetworkController::ProcessProbeResultSuccess(DataRate probe_bitrate) {
  // Update the adaptive capacity estimator with successful probe result
  capacity_estimator_->UpdateFromProbeResult(probe_bitrate, true);
  
  // Log probe success metrics
  if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->LogProbeEvent(
        Timestamp::Millis(env_.clock().TimeInMilliseconds()),
        probe_bitrate, true);
  }
  
  // This method would be called when we detect a successful probe
  // Update max_realistic_bandwidth_ if this probe shows higher capacity
  if (probe_bitrate > max_realistic_bandwidth_) {
    max_realistic_bandwidth_ = probe_bitrate * 1.1; // Add 10% headroom
    RTC_LOG(LS_INFO) << "L4S: Successful probe increased max_realistic_bandwidth_ to "
                     << max_realistic_bandwidth_.bps() << " bps";
  }
}

void L4SNetworkController::ProcessProbeResultFailed(DataRate probe_bitrate) {
  // Update the adaptive capacity estimator with failed probe result
  capacity_estimator_->UpdateFromProbeResult(probe_bitrate, false);
  
  // Log probe failure metrics
  if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->LogProbeEvent(
        Timestamp::Millis(env_.clock().TimeInMilliseconds()),
        probe_bitrate, false);
  }
  
  RTC_LOG(LS_INFO) << "L4S: Probe failed at " << probe_bitrate.bps() 
                   << " bps, updated adaptive capacity estimate to "
                   << capacity_estimator_->GetMaxRealisticBandwidth().bps() << " bps";
}

// L4SMetricsCollector implementation
L4SMetricsCollector::L4SMetricsCollector(test::MetricsLogger* logger, 
                                        const std::string& test_case_name,
                                        Clock* clock)
    : logger_(logger), 
      test_case_name_(test_case_name), 
      clock_(clock) {
  RTC_CHECK(logger_);
  RTC_CHECK(clock_);
  RTC_LOG(LS_INFO) << "L4SMetricsCollector initialized for test case: " << test_case_name_;
}

void L4SMetricsCollector::LogBandwidthMetrics(Timestamp at_time, DataRate target_bitrate, 
                                             DataRate actual_bitrate, const std::string& controller) {
  if (at_time - last_bandwidth_log_ < kBandwidthLogInterval) {
    return; // Don't spam logs
  }
  
  last_bandwidth_log_ = at_time;
  UpdateThroughputStats(actual_bitrate);
  
  // Log time-series data for bandwidth
  logger_->LogSingleValueMetric("bandwidth_target_mbps", test_case_name_, target_bitrate.bps() / 1e6, 
                                webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"controller", controller}, {"timestamp_ms", std::to_string(at_time.ms())}});
  logger_->LogSingleValueMetric("bandwidth_actual_mbps", test_case_name_, actual_bitrate.bps() / 1e6, 
                                webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"controller", controller}, {"timestamp_ms", std::to_string(at_time.ms())}});
  
  // Calculate utilization ratio
  double utilization = target_bitrate.bps() > 0 ? (double)actual_bitrate.bps() / target_bitrate.bps() : 0.0;
  logger_->LogSingleValueMetric("bandwidth_utilization_ratio", test_case_name_, utilization, 
                                webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"controller", controller}, {"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogDelayMetrics(Timestamp at_time, TimeDelta rtt, TimeDelta one_way_delay) {
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
  
  logger_->LogSingleValueMetric("packet_loss_fraction", test_case_name_, loss_fraction, 
                                webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  logger_->LogSingleValueMetric("packets_lost_count", test_case_name_, packets_lost, 
                                webrtc::test::Unit::kCount, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogCongestionMetrics(Timestamp at_time, int ce_count, int ect_count, 
                                              double congestion_ratio) {
  logger_->LogSingleValueMetric("congestion_ce_count", test_case_name_, ce_count, 
                                webrtc::test::Unit::kCount, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  logger_->LogSingleValueMetric("congestion_ect_count", test_case_name_, ect_count, 
                                webrtc::test::Unit::kCount, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  logger_->LogSingleValueMetric("congestion_ratio", test_case_name_, congestion_ratio, 
                                webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogControllerState(Timestamp at_time, const std::string& active_controller,
                                           const std::string& state_info) {
  logger_->LogSingleValueMetric("active_controller", test_case_name_, 0, // Value not meaningful for string metrics
                                webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kNeitherIsBetter,
                                {{"controller", active_controller}, 
                                 {"state_info", state_info},
                                 {"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogProbeEvent(Timestamp at_time, DataRate probe_rate, bool successful) {
  logger_->LogSingleValueMetric("probe_rate_mbps", test_case_name_, probe_rate.bps() / 1e6, 
                                webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"success", successful ? "true" : "false"},
                                 {"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogControllerSwitch(Timestamp at_time, const std::string& from_controller,
                                             const std::string& to_controller, const std::string& reason) {
  logger_->LogSingleValueMetric("controller_switch", test_case_name_, 0, // Value not meaningful
                                webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kNeitherIsBetter,
                                {{"from", from_controller}, 
                                 {"to", to_controller},
                                 {"reason", reason},
                                 {"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogNetworkEvent(Timestamp at_time, const std::string& event_type,
                                        const std::string& event_data) {
  logger_->LogSingleValueMetric("network_event", test_case_name_, 0, // Value not meaningful
                                webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kNeitherIsBetter,
                                {{"event_type", event_type}, 
                                 {"event_data", event_data},
                                 {"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogPeriodicSummary(Timestamp at_time) {
  if (at_time - last_summary_log_ < kSummaryLogInterval) {
    return;
  }
  
  last_summary_log_ = at_time;
  
  // Log summary statistics
  if (throughput_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric("throughput_avg_mbps", test_case_name_, throughput_stats_.GetAverage() / 1e6, 
                                  webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "throughput"}});
    logger_->LogSingleValueMetric("throughput_std_mbps", test_case_name_, throughput_stats_.GetStandardDeviation() / 1e6, 
                                  webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "std_dev"}, {"metric", "throughput"}});
  }
  
  if (delay_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric("delay_avg_ms", test_case_name_, delay_stats_.GetAverage(), 
                                  webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "delay"}});
    logger_->LogSingleValueMetric("delay_std_ms", test_case_name_, delay_stats_.GetStandardDeviation(), 
                                  webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "std_dev"}, {"metric", "delay"}});
  }
  
  if (loss_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric("loss_avg_fraction", test_case_name_, loss_stats_.GetAverage(), 
                                  webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "loss"}});
  }
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

// L4SNetworkController metrics helper methods implementation
void L4SNetworkController::LogPeriodicMetrics(Timestamp at_time) {
  if (!metrics_enabled_ || !metrics_collector_) {
    return;
  }
  
  // Check if it's time to log metrics
  if (at_time - metrics_last_logged_ < kMetricsLoggingInterval) {
    return;
  }
  
  metrics_last_logged_ = at_time;
  
  // Log bandwidth metrics
  DataRate target_rate = target_rate_.value_or(DataRate::Zero());
  DataRate actual_rate = last_actual_bitrate_;
  metrics_collector_->LogBandwidthMetrics(at_time, target_rate, actual_rate, current_active_controller_);
  
  // Log delay metrics
  if (last_rtt_.IsFinite()) {
    metrics_collector_->LogDelayMetrics(at_time, last_rtt_, last_rtt_ / 2); // Estimate one-way delay
  }
  
  // Log loss metrics
  metrics_collector_->LogLossMetrics(at_time, last_loss_fraction_, 0); // TODO: Track packet count
  
  // Log congestion metrics (L4S-specific)
  double congestion_ratio = (ce_count_ + ect_count_) > 0 ? 
                           (double)ce_count_ / (ce_count_ + ect_count_) : 0.0;
  metrics_collector_->LogCongestionMetrics(at_time, ce_count_, ect_count_, congestion_ratio);
  
  // Log periodic summary
  metrics_collector_->LogPeriodicSummary(at_time);
}

void L4SNetworkController::LogControllerState(Timestamp at_time) {
  if (!metrics_enabled_ || !metrics_collector_) {
    return;
  }
  
  // Determine which controller is active
  std::string active_controller = IsL4SActive() ? "l4s_prague" : "gcc_fallback";
  
  // Create state info string
  std::string state_info = "target_rate=" + std::to_string(target_rate_.value_or(DataRate::Zero()).bps()) + 
                          ";delay_est=" + std::to_string(last_delay_based_estimate_.bps()) +
                          ";acked_est=" + std::to_string(last_acknowledged_rate_.bps()) +
                          ";ecn_supported=" + (ecn_supported_ ? "true" : "false");
  
  // Log controller state only if it changed
  if (current_active_controller_ != active_controller) {
    std::string old_controller = current_active_controller_;
    current_active_controller_ = active_controller;
    
    metrics_collector_->LogControllerSwitch(at_time, old_controller, active_controller, 
                                           ecn_supported_ ? "ecn_available" : "ecn_unavailable");
  }
  
  metrics_collector_->LogControllerState(at_time, active_controller, state_info);
}

}  // namespace webrtc
