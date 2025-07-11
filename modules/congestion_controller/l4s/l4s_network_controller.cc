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
AdaptiveCapacityEstimator::AdaptiveCapacityEstimator(DataRate starting_rate, DataRate min_target_rate, DataRate max_target_rate)
    : historic_min_(starting_rate),
      historic_max_(starting_rate),
      congestion_based_estimate_(starting_rate),
      min_target_rate_(min_target_rate),
      max_target_rate_(max_target_rate),
      last_update_time_(Timestamp::MinusInfinity()) {
  // Clamp all to min/max target rates
  if (historic_min_ < min_target_rate_) historic_min_ = min_target_rate_;
  if (historic_max_ < min_target_rate_) historic_max_ = min_target_rate_;
  if (congestion_based_estimate_ < min_target_rate_) congestion_based_estimate_ = min_target_rate_;
  if (historic_min_ > max_target_rate_) historic_min_ = max_target_rate_;
  if (historic_max_ > max_target_rate_) historic_max_ = max_target_rate_;
  if (congestion_based_estimate_ > max_target_rate_) congestion_based_estimate_ = max_target_rate_;
}

AdaptiveCapacityEstimator::~AdaptiveCapacityEstimator() = default;


void AdaptiveCapacityEstimator::UpdateFromCongestionSignal(DataRate current_rate, double ce_ratio, Timestamp current_time) {
  constexpr int kDefaultMssBytes = 1000; // Typical Ethernet MSS

  TimeDelta rtt = min_rtt_.IsFinite() ? min_rtt_ : TimeDelta::Millis(10);
  double rtt_seconds = rtt.seconds<double>();
  if (rtt_seconds < 0.002) {
    rtt_seconds = 0.002; // Avoid division by zero
  }




  // // typical prague

  // if (ce_ratio > 0.05) {
    // DCTCP-style alpha update (EWMA of CE ratio)
    //  constexpr double g = 1.0 / 16.0;       // DCTCP recommended EWMA gain
    // alpha_ = (1.0 - g) * alpha_ + g * ce_ratio;
  //   // Proportional decrease on CE marks
  //   double reduction_factor = 1.0 - alpha_ / 2.0;
  //   DataRate reduced = std::max(current_rate * reduction_factor, min_target_rate_);
  //   congestion_based_estimate_ = reduced;
  //   if (congestion_based_estimate_ < historic_min_) {
  //     historic_min_ = congestion_based_estimate_;
  //   }
  //   // Clamp to min_target_rate_
  //   if (historic_min_ < min_target_rate_) historic_min_ = min_target_rate_;
  //   if (congestion_based_estimate_ < min_target_rate_) congestion_based_estimate_ = min_target_rate_;
  //   RTC_LOG(LS_INFO) << "AdaptiveCapacity: DCTCP-style decrease (alpha=" << alpha_
  //                    << ", ce_ratio=" << ce_ratio
  //                    << "), reducing congestion_based_estimate to " << congestion_based_estimate_.bps() << " bps";
  // } 

//according to rfc 6679 section 7.3.3

  if (ce_ratio > 0.05) {
    // Proportional decrease on CE marks
    double reduction_factor = 0.5;
    DataRate reduced = std::max(current_rate * reduction_factor, min_target_rate_);
    congestion_based_estimate_ = reduced;
    if (congestion_based_estimate_ < historic_min_) {
      historic_min_ = congestion_based_estimate_;
    }
    // Clamp to min_target_rate_
    if (historic_min_ < min_target_rate_) historic_min_ = min_target_rate_;
    if (congestion_based_estimate_ < min_target_rate_) congestion_based_estimate_ = min_target_rate_;
    RTC_LOG(LS_INFO) << "AdaptiveCapacity: DCTCP-style decrease (alpha=" << alpha_
                     << ", ce_ratio=" << ce_ratio
                     << "), reducing congestion_based_estimate to " << congestion_based_estimate_.bps() << " bps";
  }  else if (ce_ratio < 0.005 && current_rate >= congestion_based_estimate_ * 0.9) {
    // Linear, RTT-aware additive increase: +1 MSS per RTT
    int64_t bits_per_rtt = kDefaultMssBytes * 8;
    double ai_factor = 0.1; // 0.5 MSS per RTT
    int64_t increase_bps = rtt_seconds > 0 ? static_cast<int64_t>((bits_per_rtt * ai_factor) / rtt_seconds) : 0;
    DataRate increased = std::min(congestion_based_estimate_ + DataRate::BitsPerSec(increase_bps), max_target_rate_);
        
    // int64_t bits_per_rtt = kDefaultMssBytes * 8;
    // int64_t increase_bps = rtt_seconds > 0 ? static_cast<int64_t>(bits_per_rtt / rtt_seconds) : 0;
    // DataRate increased = std::min(congestion_based_estimate_ + DataRate::BitsPerSec(increase_bps), max_target_rate_);
    congestion_based_estimate_ = increased;
    if (congestion_based_estimate_ > historic_max_) {
      historic_max_ = congestion_based_estimate_;
    }
    // Clamp to max_target_rate_
    if (historic_max_ > max_target_rate_) historic_max_ = max_target_rate_;
    if (congestion_based_estimate_ > max_target_rate_) congestion_based_estimate_ = max_target_rate_;
    RTC_LOG(LS_INFO) << "AdaptiveCapacity: Linear AI (+1 MSS/RTT, rtt=" << rtt.ms() << " ms), "
                     << "increasing congestion_based_estimate to " << congestion_based_estimate_.bps() << " bps";
  }
  else{
    // No action needed if CE ratio is low and rate is stable

  }
  last_update_time_ = current_time;
}

void AdaptiveCapacityEstimator::UpdateFromSustainedRate(DataRate sustained_rate, Timestamp current_time) {
  // Add to history
  sustained_rates_history_.push_back(sustained_rate);
  if (sustained_rates_history_.size() > kHistoryWindowSize) {
    sustained_rates_history_.pop_front();
  }
  // Update historical estimate based on maximum sustained rate
  DataRate historical_max = *std::max_element(sustained_rates_history_.begin(), 
                                             sustained_rates_history_.end());
  historic_max_ = historical_max;

  last_update_time_ = current_time;
}

void AdaptiveCapacityEstimator::UpdateFromRtt(TimeDelta rtt) {

  min_rtt_ = std::min(min_rtt_, rtt);
  if (min_rtt_ < TimeDelta::Millis(20)) {
    min_rtt_ = TimeDelta::Millis(20); // Ensure non-negative RTT
  }
}


DataRate AdaptiveCapacityEstimator::GetMaxRealisticBandwidth() const {
  // Use the higher of congestion-based and historic min, but never above max_target_rate_
  DataRate estimate = std::max(congestion_based_estimate_, historic_min_);
  estimate = std::min(estimate, max_target_rate_);
  // Ensure we stay within absolute bounds
  DataRate periodic_max = std::max(min_target_rate_, std::min(estimate, max_target_rate_));
  RTC_LOG(LS_INFO) << "AdaptiveCapacity: Current max realistic bandwidth estimate is " 
                   << periodic_max.bps() << " bps (congestion: " << congestion_based_estimate_.bps()
                   << ", historic_min: " << historic_min_.bps()
                   << ", historic_max: " << historic_max_.bps() << ")";
  return periodic_max;
}

void AdaptiveCapacityEstimator::OnTimeUpdate(Timestamp current_time) {
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

void AdaptiveCapacityEstimator::OnPacketLoss(DataRate current_rate, Timestamp current_time) {
  // Multiplicative decrease, fallback for loss (e.g., halve the rate)
  DataRate reduced = std::max(current_rate * 0.5, min_target_rate_);
  congestion_based_estimate_ = reduced;
  if (congestion_based_estimate_ < historic_min_) {
    historic_min_ = congestion_based_estimate_;
  }
  // Clamp to min_target_rate_
  if (historic_min_ < min_target_rate_) historic_min_ = min_target_rate_;
  if (congestion_based_estimate_ < min_target_rate_) congestion_based_estimate_ = min_target_rate_;
  RTC_LOG(LS_WARNING) << "AdaptiveCapacity: Packet loss detected, halving congestion_based_estimate to "
                      << congestion_based_estimate_.bps() << " bps";

  last_update_time_ = current_time;
}



L4SNetworkController::L4SNetworkController(NetworkControllerConfig config,
                                           L4SControllerConfig l4s_config,
                                           test::MetricsLogger* metrics_logger)
    : env_(config.env),
      fallback_to_gcc_(l4s_config.fallback_to_gcc),
      use_ect1_marking_(l4s_config.use_ect1_marking),
      // Initialize adaptive capacity estimator first (based on header order)
      capacity_estimator_(
          std::make_unique<AdaptiveCapacityEstimator>(
              DataRate::KilobitsPerSec(100), DataRate::KilobitsPerSec(30), DataRate::KilobitsPerSec(20000))),
      // Initialize metrics collection
      metrics_enabled_(true),
      current_active_controller_("l4s_initializing") {
  
  // Always initialize metrics collector with a valid logger
  using webrtc::test::GetGlobalMetricsLogger;
  test::MetricsLogger* logger_to_use = metrics_logger;
  if (!logger_to_use) {
    logger_to_use = GetGlobalMetricsLogger();
  }
  metrics_collector_ = std::make_unique<L4SMetricsCollector>(
      logger_to_use, l4s_config.test_case_name, &env_.clock());
  RTC_LOG(LS_INFO) << "L4S: Metrics collection enabled for test case: " 
                   << l4s_config.test_case_name;
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
      starting_rate_ = DataRate::KilobitsPerSec(300); // Default to 300 kbps
    }
  }

  min_target_rate_ = config.constraints.min_data_rate;
  max_target_rate_ = config.constraints.max_data_rate;
  
  // Validate rate constraints
  if (min_target_rate_ && (!min_target_rate_->IsFinite() || min_target_rate_->bps() <= 0)) {
    RTC_LOG(LS_WARNING) << "L4S: Invalid min target rate, clearing";
    min_target_rate_.reset();
    min_target_rate_ = DataRate::KilobitsPerSec(30);  // Default to 30 kbps
  }
  if (max_target_rate_ && (!max_target_rate_->IsFinite() || max_target_rate_->bps() <= 0)) {
    RTC_LOG(LS_WARNING) << "L4S: Invalid max target rate, clearing"; 
    max_target_rate_.reset();
    max_target_rate_ = DataRate::KilobitsPerSec(100000);  // Default to 100 Mbps
  }
  
  // Export L4S metrics to JSON if enabled (at construction, set up export path)
  if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->ExportToJsonFile("l4s_test_1.json");
    RTC_LOG(LS_INFO) << "L4S: Metrics will be exported to l4s_test_1.json";
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
    capacity_estimator_->UpdateFromSustainedRate(*config.constraints.starting_rate, env_.clock().CurrentTime());
  }
  
  // Keep max_realistic_bandwidth_ as backup, but primary logic will use capacity_estimator_
  max_realistic_bandwidth_ = DataRate::KilobitsPerSec(100000);  // 100 Mbps upper limit
  
  // Validate critical member variables after initialization
  if (!max_realistic_bandwidth_.IsFinite()) {
    RTC_LOG(LS_ERROR) << "L4S: CRITICAL - max_realistic_bandwidth_ is not finite!";
    max_realistic_bandwidth_ = DataRate::KilobitsPerSec(100000); // Fallback to 100 Mbps
  }
}

L4SNetworkController::~L4SNetworkController() {
  // Export metrics if enabled and collector exists
  if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->ExportToJsonFile("l4s_test_1.json");
    RTC_LOG(LS_INFO) << "L4S: Exported metrics to l4s_test_1.json in destructor.";
  }
}

webrtc::NetworkControlUpdate L4SNetworkController::OnNetworkAvailability(
    webrtc::NetworkAvailability msg) {
  webrtc::NetworkControlUpdate update;


  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    auto gcc_update = gcc_controller_->OnNetworkAvailability(msg);
    }
  return update;
}

webrtc::NetworkControlUpdate  webrtc::L4SNetworkController::OnNetworkRouteChange(
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
  ect_count_ = 0;
  ce_count_ = 0;

  // Set initial rates
  if (msg.constraints.starting_rate) {
    starting_rate_ = msg.constraints.starting_rate;
  }

  min_target_rate_ = msg.constraints.min_data_rate;
  max_target_rate_ = msg.constraints.max_data_rate;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    auto gcc_update = gcc_controller_->OnNetworkRouteChange(msg);
    // Copy other relevant updates from GCC
    update.target_rate = gcc_update.target_rate;
    update.pacer_config = gcc_update.pacer_config;
  }

  return update;
}




//new update onProcessInterval method
// This method is called periodically to update the network controller state

webrtc::NetworkControlUpdate  webrtc::L4SNetworkController::OnProcessInterval(
    webrtc::ProcessInterval msg) {
  webrtc::NetworkControlUpdate update;

  // Log periodic metrics
  LogPeriodicMetrics(msg.at_time);


  if (IsL4SActive()) {
    // Use only ECN-based estimator for rate selection
    DataRate ecn_based_limit = capacity_estimator_->GetMaxRealisticBandwidth();

    // Set target rate to estimator output
    target_rate_ = ecn_based_limit;
    last_target_bitrate_ = ecn_based_limit;

    RTC_LOG(LS_INFO) << "L4S: ECN-based target rate: " << ecn_based_limit.bps() << " bps";

    MaybeTriggerOnNetworkChanged(&update, msg.at_time);
  } else if (fallback_to_gcc_) {
    update = gcc_controller_->OnProcessInterval(msg);
  }

  return update;
}



webrtc::NetworkControlUpdate  webrtc::L4SNetworkController::OnRemoteBitrateReport(
    webrtc::RemoteBitrateReport msg) {
  webrtc::NetworkControlUpdate update;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnRemoteBitrateReport(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate  webrtc::L4SNetworkController::OnRoundTripTimeUpdate(
    webrtc::RoundTripTimeUpdate msg) {
  webrtc::NetworkControlUpdate update;

  
  // Update the adaptive capacity estimator with RTT information
  capacity_estimator_->UpdateFromRtt(msg.round_trip_time);

  // Update local RTT tracking for metrics
  last_rtt_ = msg.round_trip_time;
  
  // Log RTT metrics
  if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->LogDelayMetrics(
        Timestamp::Millis(env_.clock().TimeInMilliseconds()),
        msg.round_trip_time, msg.round_trip_time / 2, jitter_);
  }

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    update = gcc_controller_->OnRoundTripTimeUpdate(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate  webrtc::L4SNetworkController::OnSentPacket(webrtc::SentPacket msg) {
  webrtc::NetworkControlUpdate update;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    update = gcc_controller_->OnSentPacket(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate  webrtc::L4SNetworkController::OnReceivedPacket(
    webrtc::ReceivedPacket msg) {
  webrtc::NetworkControlUpdate update;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnReceivedPacket(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate  webrtc::L4SNetworkController::OnStreamsConfig(webrtc::StreamsConfig msg) {
  webrtc::NetworkControlUpdate update;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    update = gcc_controller_->OnStreamsConfig(msg);
  }

  return update;
}

webrtc::NetworkControlUpdate  webrtc::L4SNetworkController::OnTargetRateConstraints(
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

webrtc::NetworkControlUpdate  webrtc::L4SNetworkController::OnTransportLossReport(
    webrtc::TransportLossReport msg) {
  webrtc::NetworkControlUpdate update;

  // Only apply loss fallback if L4S is active
  if (IsL4SActive()) {
    DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
    capacity_estimator_->OnPacketLoss(current_rate, msg.receive_time);
  }

  // Update loss metrics for logging
  int total_packets = msg.packets_lost_delta + msg.packets_received_delta;
  if (total_packets > 0) {
    last_loss_fraction_ = static_cast<double>(msg.packets_lost_delta) / total_packets;
  } else {
    last_loss_fraction_ = 0.0;
  }
  last_packets_lost_ = static_cast<int>(msg.packets_lost_delta);

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnTransportLossReport(msg);
  }

  return update;
}

// new OnTransportPacketsFeedback method:
// This method processes transport feedback packets and updates the network controller state

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnTransportPacketsFeedback(
    webrtc::TransportPacketsFeedback feedback) {
  webrtc::NetworkControlUpdate update;

  // Process ECN feedback to update estimator
  ProcessEcnFeedback(feedback);

  // Use only ECN-based estimator for rate selection
  if (IsL4SActive()) {
    DataRate ecn_based_limit = capacity_estimator_->GetMaxRealisticBandwidth();
    target_rate_ = ecn_based_limit;
    last_target_bitrate_ = ecn_based_limit;

    RTC_LOG(LS_INFO) << "L4S: ECN-based target rate (feedback): " << ecn_based_limit.bps() << " bps";
    MaybeTriggerOnNetworkChanged(&update, feedback.feedback_time);
  } else if (fallback_to_gcc_) {
    update = gcc_controller_->OnTransportPacketsFeedback(feedback);
  }


  // Per-packet delay and jitter logging (this is outside the above if/else)
  TimeDelta prev_delay = TimeDelta::Zero();
  bool first = true;
  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.receive_time.IsFinite() && packet.sent_packet.send_time.IsFinite()) {
      TimeDelta delay = packet.receive_time - packet.sent_packet.send_time;
      if (!first) {
        TimeDelta jitter = (delay - prev_delay).Abs();
        jitter_ = jitter;
        metrics_collector_->LogDelayMetrics(packet.receive_time, delay, delay / 2, jitter);
      }
      prev_delay = delay;
      first = false;
    }
  }

  return update;
}


webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnNetworkStateEstimate(
    webrtc::NetworkStateEstimate msg) {
  webrtc::NetworkControlUpdate update;

  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnNetworkStateEstimate(msg);
  }

  return update;
}

// In CreateRateUpdate method:
webrtc::NetworkControlUpdate webrtc::L4SNetworkController::CreateRateUpdate(
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
    RTC_LOG(LS_INFO) << "L4S applying min_target_rate: " <<
    min_target_rate_->bps() << " bps";
    current_rate_for_transport_ = *min_target_rate_;
  }
  if (max_target_rate_ && current_rate_for_transport_ > *max_target_rate_) {
    RTC_LOG(LS_INFO) << "L4S applying max_target_rate: " <<
    max_target_rate_->bps() << " bps";
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
  return ecn_supported_ && ecn_capable_network_;
}

void L4SNetworkController::ProcessEcnFeedback(
    const TransportPacketsFeedback& feedback) {
  if (feedback.packet_feedbacks.empty()) {
    RTC_LOG(LS_WARNING) << "ProcessEcnFeedback: Empty feedback, packet_feedbacks.size()=0";
    return;
  }

  int new_ect_count = 0;
  int new_ce_count = 0;
  ce_count_ = 0;
  ect_count_ = 0;

  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.ecn == EcnMarking::kEct0 ||
        packet.ecn == EcnMarking::kEct1 ||
        packet.ecn == EcnMarking::kCe) {
      new_ect_count++;
    }
    if (packet.ecn == EcnMarking::kCe) {
      new_ce_count++;
      last_congestion_signal_ = feedback.feedback_time;
      RTC_LOG(LS_WARNING) << "ProcessEcnFeedback: CE MARK DETECTED! Count=" << new_ce_count
                         << ", ECT count=" << new_ect_count;

      if (metrics_enabled_ && metrics_collector_) {
        double congestion_ratio = (new_ce_count + new_ect_count) > 0 ?
                                  static_cast<double>(new_ce_count) / (new_ce_count + new_ect_count) : 0.0;
        metrics_collector_->LogCongestionMetrics(feedback.feedback_time, new_ce_count, new_ect_count, congestion_ratio);
      }
    }
  }

  // If we received any ECT or CE packets, consider ECN supported
  if (new_ect_count > 0 || new_ce_count > 0) {
    ecn_supported_ = true;
  }

  // Use only the current interval for CE ratio:
  if (new_ect_count + new_ce_count > 0) {
    double ce_ratio = static_cast<double>(new_ce_count) / (new_ect_count + new_ce_count);
    DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
    capacity_estimator_->UpdateFromCongestionSignal(current_rate, ce_ratio, feedback.feedback_time);
    ecn_capable_network_ = true;
  }
  ce_count_ += new_ce_count;
  ect_count_ += new_ect_count;

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
                                             DataRate actual_bitrate) {
  if (at_time - last_bandwidth_log_ < kBandwidthLogInterval) {
    return; // Don't spam logs
  }
  
  last_bandwidth_log_ = at_time;
  UpdateThroughputStats(actual_bitrate);
  
  // Log time-series data for bandwidth
  logger_->LogSingleValueMetric("bandwidth_target_mbps", test_case_name_, target_bitrate.bps() / 1e6, 
                                webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  logger_->LogSingleValueMetric("bandwidth_actual_mbps", test_case_name_, actual_bitrate.bps() / 1e6, 
                                webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  
  // Calculate utilization ratio
  double utilization = target_bitrate.bps() > 0 ? (double)actual_bitrate.bps() / target_bitrate.bps() : 0.0;
  logger_->LogSingleValueMetric("bandwidth_utilization_ratio", test_case_name_, utilization, 
                                webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogDelayMetrics(Timestamp at_time, TimeDelta rtt, TimeDelta one_way_delay, 
                                         TimeDelta jitter) {
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
  if (jitter.IsFinite()) {
    logger_->LogSingleValueMetric("jitter_ms", test_case_name_, jitter.ms(), 
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
  // Periodically export all metrics to JSON
  ExportToJsonFile("l4s_test_1.json");
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
  metrics_collector_->LogBandwidthMetrics(at_time, target_rate, actual_rate);
  
  // Log delay metrics
  if (last_rtt_.IsFinite()) {
    metrics_collector_->LogDelayMetrics(at_time, last_rtt_, last_rtt_ / 2, jitter_); // Estimate one-way delay
  }
  
  // Log loss metrics
  metrics_collector_->LogLossMetrics(at_time, last_loss_fraction_, last_packets_lost_); // TODO: Track packet count
  
  // Log congestion metrics (L4S-specific)
  double congestion_ratio = (ce_count_ + ect_count_) > 0 ? 
                           (double)ce_count_ / (ce_count_ + ect_count_) : 0.0;
  metrics_collector_->LogCongestionMetrics(at_time, ce_count_, ect_count_, congestion_ratio);
  
  // Log periodic summary
  metrics_collector_->LogPeriodicSummary(at_time);
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
    fprintf(f, "    \"test_case\": \"%s\",\n", m.test_case.c_str());
    fprintf(f, "    \"unit\": %d,\n", (int)m.unit);
    fprintf(f, "    \"improvement_direction\": %d,\n", (int)m.improvement_direction);
    fprintf(f, "    \"stats\": {\n");
    fprintf(f, "      \"mean\": %s,\n", m.stats.mean ? std::to_string(*m.stats.mean).c_str() : "null");
    fprintf(f, "      \"stddev\": %s,\n", m.stats.stddev ? std::to_string(*m.stats.stddev).c_str() : "null");
    fprintf(f, "      \"min\": %s,\n", m.stats.min ? std::to_string(*m.stats.min).c_str() : "null");
    fprintf(f, "      \"max\": %s\n", m.stats.max ? std::to_string(*m.stats.max).c_str() : "null");
    fprintf(f, "    },\n");
    // Export metric-level metadata
    fprintf(f, "    \"metadata\": {");
    size_t meta_count = 0;
    for (const auto& kv : m.metric_metadata) {
      if (meta_count > 0) fprintf(f, ", ");
      fprintf(f, "\"%s\": \"%s\"", kv.first.c_str(), kv.second.c_str());
      ++meta_count;
    }
    fprintf(f, "},\n");
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

}  // namespace webrtc
