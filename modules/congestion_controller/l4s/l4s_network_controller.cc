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
#include "rtc_base/checks.h"
#include "rtc_base/experiments/field_trial_parser.h"
#include "rtc_base/logging.h"
#include "system_wrappers/include/metrics.h"

namespace webrtc {

L4SNetworkController::L4SNetworkController(NetworkControllerConfig config,
                                           L4SControllerConfig l4s_config)
    : env_(config.env),
      fallback_to_gcc_(l4s_config.fallback_to_gcc),
      use_ect1_marking_(l4s_config.use_ect1_marking),
      prague_controller_(
          std::make_unique<L4SPragueController>(env_.field_trials())),
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
                                            &env_.event_log())) {
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
                      << " max_realistic_bandwidth: " << max_realistic_bandwidth_.bps() << " bps";
                      
  // Initialize max_realistic_bandwidth_ to a reasonable starting value
  // Keep this as a hard upper limit - don't modify it during operation
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

  // Get rate from Prague controller if active
  if (IsL4SActive()) {
    DataRate current_rate_for_transport_ =
        target_rate_.value_or(DataRate::KilobitsPerSec(300));

    RTC_LOG(LS_INFO)
        << "L4S DEBUG: Starting cycle with current_rate_for_transport_="
        << current_rate_for_transport_.bps() << " bps (from target_rate_="
        << (target_rate_ ? target_rate_->bps() : -1) << ")";

    // Consider BWE estimates for capacity limiting (consistent with
    // OnTransportPacketsFeedback)
    DataRate bwe_based_limit = max_realistic_bandwidth_;
    RTC_LOG(LS_WARNING)
        << "L4S OnProcessInterval DEBUG: Initial bwe_based_limit="
        << bwe_based_limit.bps() << " bps";

    // Use delay-based estimate as the primary capacity indicator, but don't exceed max_realistic_bandwidth_
    
    // If we have a valid delay-based estimate, use it intelligently
    if (last_delay_based_estimate_ > DataRate::Zero()) {
      // Check if delay-based estimate suggests we can increase without hitting congestion
      DataRate delay_based_limit = std::min(last_delay_based_estimate_ * 0.98, max_realistic_bandwidth_);
      
      RTC_LOG(LS_WARNING) << "L4S OnProcessInterval: Using delay-based limit="
                          << delay_based_limit.bps() << " bps (98% of "
                          << last_delay_based_estimate_.bps() << " bps, capped by max_realistic="
                          << max_realistic_bandwidth_.bps() << " bps)";
      
      bwe_based_limit = delay_based_limit;
    } else {
      // No delay estimate yet, use a conservative portion of max_realistic_bandwidth_
      bwe_based_limit = max_realistic_bandwidth_ * 0.1;  // Start with 10% of max
      RTC_LOG(LS_WARNING) << "L4S OnProcessInterval: No delay estimate, using conservative limit="
                          << bwe_based_limit.bps() << " bps (10% of max_realistic)";
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

    RTC_LOG(LS_INFO)
        << "L4S OnProcessInterval: Final BWE-based capacity limit: "
        << bwe_based_limit.bps() << " bps";

    // Allow Prague to increase up to BWE-based limit, even if current rate is
    // lower This prevents getting stuck when current rate is below the
    // available capacity Made more aggressive: 99.5% instead of 98% for faster
    // ramp-up and better video quality
    DataRate rate_for_prague =
        std::min(std::max(current_rate_for_transport_, bwe_based_limit * 0.995),
                 bwe_based_limit);
    if (rate_for_prague > current_rate_for_transport_) {
      RTC_LOG(LS_WARNING) << "L4S: Allowing Prague to target higher rate "
                          << rate_for_prague.bps() << " bps instead of current "
                          << current_rate_for_transport_.bps()
                          << " bps (BWE limit: " << bwe_based_limit.bps()
                          << " bps)";
    }

    RTC_LOG(LS_WARNING)
        << "L4S OnProcessInterval DEBUG: Passing rate_for_prague="
        << rate_for_prague.bps()
        << " to Prague (current=" << current_rate_for_transport_.bps()
        << ", bwe_limit=" << bwe_based_limit.bps() << ")";

    auto prague_rate =
        prague_controller_->GetTargetRate(msg.at_time, rate_for_prague);
    if (prague_rate) {
      // CRITICAL: Validate Prague controller output before using it
      if (!prague_rate->IsFinite() || prague_rate->bps() <= 0) {
        RTC_LOG(LS_WARNING) << "L4S: Prague controller returned invalid rate " 
                            << prague_rate->bps() << " bps in OnProcessInterval, using fallback";
        prague_rate = DataRate::KilobitsPerSec(300);
      }

      // Double-check the Prague controller's output against BWE-informed limits
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

      MaybeTriggerOnNetworkChanged(&update, msg.at_time);
    }
    
    // Check if we're in ALR (Application Limited Region) to enable probing
    // ALR occurs when we're sending below our estimated capacity
    DataRate current_sending_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
    DataRate estimated_capacity = std::max(last_delay_based_estimate_, last_acknowledged_rate_);
    
    if (estimated_capacity > DataRate::Zero() && 
        current_sending_rate < estimated_capacity * 0.8) {  // Using less than 80% of capacity
      if (!alr_start_time_) {
        alr_start_time_ = msg.at_time;
        probe_controller_->SetAlrStartTimeMs(msg.at_time.ms());
        RTC_LOG(LS_INFO) << "L4S: Entered ALR state - sending " << current_sending_rate.bps() 
                         << " bps < 80% of capacity " << estimated_capacity.bps() << " bps";
      }
    } else {
      if (alr_start_time_) {
        probe_controller_->SetAlrEndedTimeMs(msg.at_time.ms());
        alr_start_time_.reset();
        RTC_LOG(LS_INFO) << "L4S: Exited ALR state";
      }
    }
    
    // Use ProbeController for bandwidth discovery
    auto probe_clusters = probe_controller_->Process(msg.at_time);
    if (!probe_clusters.empty()) {
      update.probe_cluster_configs = std::move(probe_clusters);
      
      RTC_LOG(LS_INFO) << "L4S: ProbeController initiated " << update.probe_cluster_configs.size() 
                       << " probe cluster(s)";
      for (const auto& probe : update.probe_cluster_configs) {
        RTC_LOG(LS_INFO) << "L4S: Probe at " << probe.target_data_rate.bps() << " bps";
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
    bandwidth_estimation_->SetAcknowledgedRate(acknowledged_bitrate,
                                               feedback.feedback_time);

    RTC_LOG(LS_INFO) << "L4S: Acknowledged bitrate estimate: "
                     << acknowledged_bitrate->bps() << " bps";
  }

  // 3. Run delay-based BWE to get capacity estimate
  DelayBasedBwe::Result delay_result =
      delay_based_bwe_->IncomingPacketFeedbackVector(
          feedback, acknowledged_bitrate, std::nullopt, std::nullopt,
          false);

  if (delay_result.updated) {
    last_delay_based_estimate_ = delay_result.target_bitrate;
    bandwidth_estimation_->UpdateDelayBasedEstimate(
        feedback.feedback_time, delay_result.target_bitrate);

    // Log delay-based congestion state for debugging
    const char* state_str = "UNKNOWN";
    switch (delay_result.delay_detector_state) {
      case BandwidthUsage::kBwNormal:
        state_str = "NORMAL";
        break;
      case BandwidthUsage::kBwUnderusing:
        state_str = "UNDERUSING";
        break;
      case BandwidthUsage::kBwOverusing:
        state_str = "OVERUSING";
        break;
      case BandwidthUsage::kLast:
        state_str = "INVALID";
        break;
    }

    RTC_LOG(LS_INFO) << "L4S: Delay-based BWE estimate: "
                     << delay_result.target_bitrate.bps()
                     << " bps, state: " << state_str << ", recovered: "
                     << (delay_result.recovered_from_overuse ? "YES" : "NO");
                     
    // Update ProbeController with the estimated bitrate
    BandwidthLimitedCause bandwidth_limited_cause = BandwidthLimitedCause::kDelayBasedLimited;
    if (delay_result.delay_detector_state == BandwidthUsage::kBwOverusing) {
      bandwidth_limited_cause = BandwidthLimitedCause::kDelayBasedLimitedDelayIncreased;
    }
    
    auto probe_clusters = probe_controller_->SetEstimatedBitrate(
        delay_result.target_bitrate, bandwidth_limited_cause, feedback.feedback_time);
    if (!probe_clusters.empty()) {
      // Add probes to the update - will be merged with any existing probes
      update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                         probe_clusters.begin(), probe_clusters.end());
      RTC_LOG(LS_INFO) << "L4S: BWE update triggered " << probe_clusters.size() << " probe(s)";
    }
  }

  // 4. Update our network capacity estimate based on BWE results
  DataRate bwe_estimate = bandwidth_estimation_->target_rate();
  RTC_LOG(LS_WARNING)
      << "L4S BWE Source Debug: bandwidth_estimation_->target_rate()="
      << bwe_estimate.bps()
      << " bps, max_realistic_bandwidth_=" << max_realistic_bandwidth_.bps()
      << " bps";

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
  // RTC_LOG(LS_INFO) << "L4S calling Prague UpdateEcnFeedback";

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
      
      RTC_LOG(LS_WARNING) << "L4S OnTransportFeedback: Using delay-based limit="
                          << delay_based_limit.bps() << " bps (98% of "
                          << last_delay_based_estimate_.bps() << " bps, capped by max_realistic="
                          << max_realistic_bandwidth_.bps() << " bps)";
    }

    RTC_LOG(LS_INFO) << "L4S BWE Debug: max_realistic_bandwidth_="
                     << max_realistic_bandwidth_.bps()
                     << ", last_delay_based_estimate_="
                     << last_delay_based_estimate_.bps()
                     << ", last_acknowledged_rate_="
                     << last_acknowledged_rate_.bps();

    // Use delay-based estimate as capacity indicator, but keep it simple
    if (last_delay_based_estimate_ > DataRate::Zero()) {
      // Use a conservative portion of delay estimate, capped by max_realistic_bandwidth_
      DataRate delay_limit = std::min(last_delay_based_estimate_ * 0.95, max_realistic_bandwidth_);
      bwe_based_limit = delay_limit;
      
      RTC_LOG(LS_INFO) << "L4S: Using delay-based limit=" << delay_limit.bps() 
                       << " bps (95% of " << last_delay_based_estimate_.bps() 
                       << " bps, max_realistic=" << max_realistic_bandwidth_.bps() << " bps)";
    } else {
      // No delay estimate, use conservative limit
      bwe_based_limit = max_realistic_bandwidth_ * 0.1;
      RTC_LOG(LS_INFO) << "L4S: No delay estimate, using conservative limit=" 
                       << bwe_based_limit.bps() << " bps";
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

    RTC_LOG(LS_INFO) << "L4S: BWE-based capacity limit: "
                     << bwe_based_limit.bps()
                     << " bps (acked: " << last_acknowledged_rate_.bps()
                     << ", delay: " << last_delay_based_estimate_.bps() << ")";

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

      RTC_LOG(LS_INFO) << "L4S got target rate: " << prague_rate->bps()
                       << " bps";
      RTC_LOG(LS_INFO) << "L4S setting target_rate_ from "
                       << (target_rate_ ? target_rate_->bps() : -1) << " to "
                       << prague_rate->bps() << " bps";
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

  RTC_LOG(LS_INFO) << "L4S CreateRateUpdate: target_rate_stored="
                   << (target_rate_ ? target_rate_->bps() : -1) << " bps"
                   << ", current_rate=" << current_rate_for_transport_.bps() << " bps";

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
  update.target_rate->at_time = at_time;  // Set the top-level at_time field
  update.target_rate->network_estimate.at_time = at_time;
  update.target_rate->network_estimate.bandwidth = current_rate_for_transport_;
  update.target_rate->network_estimate.loss_rate_ratio = 0.0f;

  // Log before creating TimeDelta objects that might cause unit_base.h
  // assertion RTC_LOG(LS_INFO) << "L4S before TimeDelta::Millis(50) for
  // round_trip_time";
  update.target_rate->network_estimate.round_trip_time = TimeDelta::Millis(50);

  // RTC_LOG(LS_INFO) << "L4S before TimeDelta::Millis(500) for bwe_period";
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
    return;
  }

  // Count ECT and CE packets
  int new_ect_count = 0;
  int new_ce_count = 0;

  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1) {
      new_ect_count++;
    } else if (packet.ecn == EcnMarking::kCe) {
      new_ce_count++;
      last_congestion_signal_ = feedback.feedback_time;
    }
  }

  // If we received any ECT or CE packets, consider ECN supported
  if (new_ect_count > 0 || new_ce_count > 0) {
    ecn_supported_ = true;
  }

  // Update total counts
  ect_count_ += new_ect_count;
  ce_count_ += new_ce_count;

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
  // This method would be called when we detect a successful probe
  // Update max_realistic_bandwidth_ if this probe shows higher capacity
  if (probe_bitrate > max_realistic_bandwidth_) {
    max_realistic_bandwidth_ = probe_bitrate * 1.1; // Add 10% headroom
    RTC_LOG(LS_INFO) << "L4S: Successful probe increased max_realistic_bandwidth_ to "
                     << max_realistic_bandwidth_.bps() << " bps";
  }
}

}  // namespace webrtc
