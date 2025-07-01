#include "modules/congestion_controller/l4s/l4s_network_controller.h"

#include <algorithm>
#include <memory>
#include <utility>

#include "api/transport/goog_cc_factory.h"
#include "api/transport/goog_cc_factory.h"  // Added for GoogCcFactory
#include "api/transport/network_types.h"
#include "api/units/timestamp.h"
#include "modules/congestion_controller/goog_cc/goog_cc_network_control.h"
#include "rtc_base/logging.h"


#include "absl/strings/match.h"
#include "api/field_trials_view.h"
#include "api/rtc_event_log/rtc_event_log.h"

#include "api/units/data_rate.h"
#include "api/units/data_size.h"
#include "api/units/time_delta.h"

#include "logging/rtc_event_log/events/rtc_event_probe_cluster_created.h"
#include "rtc_base/checks.h"
#include "rtc_base/experiments/field_trial_parser.h"
#include "system_wrappers/include/metrics.h"


namespace webrtc {

L4SNetworkController::L4SNetworkController(
    NetworkControllerConfig config,
    L4SControllerConfig l4s_config)
    : env_(config.env),
      fallback_to_gcc_(l4s_config.fallback_to_gcc),
      use_ect1_marking_(l4s_config.use_ect1_marking),
      prague_controller_(std::make_unique<L4SPragueController>(env_.field_trials())),
      probe_controller_(std::make_unique<ProbeController>(&config.env.field_trials(), 
                                                         &config.env.event_log())) {
  
  // Create GCC controller for fallback if needed
  if (fallback_to_gcc_) {
    GoogCcFactoryConfig factory_config;
    auto factory = 
        std::make_unique<GoogCcNetworkControllerFactory>(std::move(factory_config));
    gcc_controller_ = factory->Create(config);
  }
  
  if (config.constraints.starting_rate) {
    starting_rate_ = config.constraints.starting_rate;
  }
  
  min_target_rate_ = config.constraints.min_data_rate;
  max_target_rate_ = config.constraints.max_data_rate;
  
  RTC_LOG(LS_INFO) << "L4S network controller created"
                   << " fallback_to_gcc: " << fallback_to_gcc_
                   << " use_ect1_marking: " << use_ect1_marking_;
}

L4SNetworkController::~L4SNetworkController() = default;

NetworkControlUpdate L4SNetworkController::OnNetworkAvailability(
    NetworkAvailability msg) {
  NetworkControlUpdate update;
  
  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnNetworkAvailability(msg);
  }
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnNetworkRouteChange(
    NetworkRouteChange msg) {
  NetworkControlUpdate update;
  
  // Reset ECN support detection on network change
  ecn_supported_ = false;
  ecn_capable_network_ = false;
  
  // Set initial rates
  if (msg.constraints.starting_rate) {
    starting_rate_ = msg.constraints.starting_rate;
  }
  
  min_target_rate_ = msg.constraints.min_data_rate;
  max_target_rate_ = msg.constraints.max_data_rate;
  
  // Configure the probe controller with the new bitrates
  auto probes = probe_controller_->SetBitrates(
      *starting_rate_, 
      min_target_rate_.value_or(DataRate::Zero()), 
      max_target_rate_.value_or(DataRate::PlusInfinity()), 
      msg.at_time);
  
  // Add probe clusters to the update
  for (const auto& probe : probes) {
    update.probe_cluster_configs.push_back(probe);
  }
  
  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    auto gcc_update = gcc_controller_->OnNetworkRouteChange(msg);
    // Merge updates if needed
    if (!update.probe_cluster_configs.empty() && 
        !gcc_update.probe_cluster_configs.empty()) {
      update.probe_cluster_configs.insert(
          update.probe_cluster_configs.end(),
          gcc_update.probe_cluster_configs.begin(),
          gcc_update.probe_cluster_configs.end());
    } else if (update.probe_cluster_configs.empty()) {
      update.probe_cluster_configs = std::move(gcc_update.probe_cluster_configs);
    }
  }
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnProcessInterval(
    ProcessInterval msg) {
  NetworkControlUpdate update;
  
  // Get rate from Prague controller if active
  if (IsL4SActive()) {
    auto prague_rate = prague_controller_->GetTargetRate(msg.at_time);
    if (prague_rate) {
      target_rate_ = prague_rate;
      MaybeTriggerOnNetworkChanged(&update, msg.at_time);
    }
    
    // Generate probes if needed
    auto probes = probe_controller_->Process(msg.at_time);
    update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                        probes.begin(), probes.end());
  } else if (fallback_to_gcc_) {
    // Forward to GCC if we're not using L4S
    update = gcc_controller_->OnProcessInterval(msg);
  }
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnRemoteBitrateReport(
    RemoteBitrateReport msg) {
  NetworkControlUpdate update;
  
  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnRemoteBitrateReport(msg);
  }
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnRoundTripTimeUpdate(
    RoundTripTimeUpdate msg) {
  NetworkControlUpdate update;
  
  // Update Prague controller
  prague_controller_->UpdateRtt(msg.round_trip_time);
  
  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    update = gcc_controller_->OnRoundTripTimeUpdate(msg);
  }
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnSentPacket(
    SentPacket msg) {
  NetworkControlUpdate update;
  
  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    update = gcc_controller_->OnSentPacket(msg);
  }
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnReceivedPacket(
    ReceivedPacket msg) {
  NetworkControlUpdate update;
  
  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnReceivedPacket(msg);
  }
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnStreamsConfig(
    StreamsConfig msg) {
  NetworkControlUpdate update;
  
  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    update = gcc_controller_->OnStreamsConfig(msg);
  }
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnTargetRateConstraints(
    TargetRateConstraints msg) {
  NetworkControlUpdate update;
  
  // Update constraints
  min_target_rate_ = msg.min_data_rate;
  max_target_rate_ = msg.max_data_rate;
  
  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_) {
    update = gcc_controller_->OnTargetRateConstraints(msg);
  }
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnTransportLossReport(
    TransportLossReport msg) {
  NetworkControlUpdate update;
  
  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnTransportLossReport(msg);
  }
  
  return update;
}

// In OnTransportPacketsFeedback method:
NetworkControlUpdate L4SNetworkController::OnTransportPacketsFeedback(
    TransportPacketsFeedback feedback) {
  NetworkControlUpdate update;
  
  // Validate feedback time to prevent invalid timestamps
  if (!feedback.feedback_time.IsFinite() || feedback.feedback_time.us() < 0) {
    RTC_LOG(LS_WARNING) << "Invalid feedback time: " << feedback.feedback_time.us()
                        << " us, using current time instead";
    feedback.feedback_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }
  
  // Log feedback details before processing
  // RTC_LOG(LS_INFO) << "L4S OnTransportPacketsFeedback: feedback_time=" 
  //                  << feedback.feedback_time.us() << " us, packet_count=" 
  //                  << feedback.packet_feedbacks.size();
  
  // Process ECN feedback to detect if ECN is supported
  ProcessEcnFeedback(feedback);
  
  // Log before updating Prague controller
  // RTC_LOG(LS_INFO) << "L4S calling Prague UpdateEcnFeedback";
  
  // Update Prague controller with ECN feedback
  prague_controller_->UpdateEcnFeedback(feedback);
  
  // Get updated target rate if L4S is active
  if (IsL4SActive()) {
    // RTC_LOG(LS_INFO) << "L4S is active, getting target rate from Prague controller";
    auto prague_rate = prague_controller_->GetTargetRate(feedback.feedback_time);
    if (prague_rate) {
      // RTC_LOG(LS_INFO) << "L4S got target rate: " << prague_rate->bps() << " bps";
      target_rate_ = prague_rate;
      MaybeTriggerOnNetworkChanged(&update, feedback.feedback_time);
    } else {
      // RTC_LOG(LS_INFO) << "L4S Prague controller returned no target rate";
    }
  } else if (fallback_to_gcc_) {
    // RTC_LOG(LS_INFO) << "L4S not active, forwarding to GCC";
    // Forward to GCC if we're not using L4S
    update = gcc_controller_->OnTransportPacketsFeedback(feedback);
  }
  
  return update;
}

NetworkControlUpdate L4SNetworkController::OnNetworkStateEstimate(
    NetworkStateEstimate msg) {
  NetworkControlUpdate update;
  
  // Forward to GCC if we're using it as fallback
  if (fallback_to_gcc_ && !IsL4SActive()) {
    update = gcc_controller_->OnNetworkStateEstimate(msg);
  }
  
  return update;
}

// In CreateRateUpdate method:
NetworkControlUpdate L4SNetworkController::CreateRateUpdate(
    Timestamp at_time) const {
  NetworkControlUpdate update;
  
  // Log input timestamp value before IsFinite check
  // RTC_LOG(LS_INFO) << "L4S CreateRateUpdate: input at_time.us()=" << at_time.us() 
  //                  << " at_time.ms()=" << at_time.ms()
  //                  << " IsFinite()=" << at_time.IsFinite();
  
  // Add timestamp validation here (similar to what GoogCC does)
  if (!at_time.IsFinite()) {
    RTC_LOG(LS_WARNING) << "Invalid timestamp in L4S controller, using current time";
    int64_t current_time_ms = env_.clock().TimeInMilliseconds();
    // RTC_LOG(LS_INFO) << "L4S CreateRateUpdate: current_time_ms=" << current_time_ms;
    at_time = Timestamp::Millis(current_time_ms);
    // RTC_LOG(LS_INFO) << "L4S CreateRateUpdate: new at_time.us()=" << at_time.us();
  }
  
  // Apply rate constraints
  DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  
  // Log the raw values before applying constraints and creating units
  // RTC_LOG(LS_INFO) << "L4S CreateRateUpdate: target_rate_bps=" 
  //                  << (target_rate_ ? target_rate_->bps() : -1)
  //                  << ", current_rate_bps=" << current_rate.bps()
  //                  << ", at_time_us=" << at_time.us();
  
  if (min_target_rate_ && current_rate < *min_target_rate_) {
    // RTC_LOG(LS_INFO) << "L4S applying min_target_rate: " << min_target_rate_->bps() << " bps";
    current_rate = *min_target_rate_;
  }
  if (max_target_rate_ && current_rate > *max_target_rate_) {
    // RTC_LOG(LS_INFO) << "L4S applying max_target_rate: " << max_target_rate_->bps() << " bps";
    current_rate = *max_target_rate_;
  }
  
  // Log final rate before creating update objects
  // RTC_LOG(LS_INFO) << "L4S final current_rate_bps=" << current_rate.bps();
  
  // Set target rate and bandwidth estimate
  update.target_rate = TargetTransferRate();
  update.target_rate->at_time = at_time;  // Set the top-level at_time field
  update.target_rate->network_estimate.at_time = at_time;
  update.target_rate->network_estimate.bandwidth = current_rate;
  update.target_rate->network_estimate.loss_rate_ratio = 0.0f;
  
  // Log before creating TimeDelta objects that might cause unit_base.h assertion
  // RTC_LOG(LS_INFO) << "L4S before TimeDelta::Millis(50) for round_trip_time";
  update.target_rate->network_estimate.round_trip_time = TimeDelta::Millis(50);
  
  // RTC_LOG(LS_INFO) << "L4S before TimeDelta::Millis(500) for bwe_period";
  update.target_rate->network_estimate.bwe_period = TimeDelta::Millis(500);
  
  update.target_rate->target_rate = current_rate;
  
  // Set pacer config
  // RTC_LOG(LS_INFO) << "L4S before creating PacerConfig";
  update.pacer_config = PacerConfig();
  update.pacer_config->at_time = at_time;
  
  // Log before setting pacer config values
  // RTC_LOG(LS_INFO) << "L4S before setting pacer config with current_rate: " << current_rate.bps() << " bps";
  
  // Set time window (e.g., 10ms)
  TimeDelta time_window = TimeDelta::Millis(10);
  // RTC_LOG(LS_INFO) << "L4S setting time_window: " << time_window.ms() << " ms";
  update.pacer_config->time_window = time_window;
  
  // Calculate data window based on current rate
  // Log before the multiplication that might trigger unit_base.h assertion
  // RTC_LOG(LS_INFO) << "L4S PRE-DATA-WINDOW-MULTIPLY: current_rate.bps()=" << current_rate.bps()
  //                  << ", time_window.ms()=" << time_window.ms()
  //                  << ", time_window.us()=" << time_window.us()
  //                  << ", current_rate.IsFinite()=" << (current_rate.IsFinite() ? "true" : "false")
  //                  << ", time_window.IsFinite()=" << (time_window.IsFinite() ? "true" : "false");
  
  DataSize data_window = current_rate * time_window;
  
  // RTC_LOG(LS_INFO) << "L4S POST-DATA-WINDOW-MULTIPLY: data_window.bytes()=" << data_window.bytes()
  //                  << ", data_window.IsFinite()=" << (data_window.IsFinite() ? "true" : "false");
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
    RTC_LOG(LS_WARNING) << "Invalid timestamp in network change, using current time";
    at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }
  
  // Log timestamp values before calling CreateRateUpdate
  // RTC_LOG(LS_INFO) << "L4S MaybeTriggerOnNetworkChanged: at_time_us=" << at_time.us()
  //                  << ", IsFinite=" << (at_time.IsFinite() ? "true" : "false");
  
  // Create rate update (without the conditional, since your code didn't define the variables)
  // RTC_LOG(LS_INFO) << "L4S calling CreateRateUpdate";
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
  return prague_controller_->IsActive() && 
         ecn_supported_ && 
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
  // and have seen at least one CE mark or a reasonable proportion of ECT packets
  if (ect_count_ + ce_count_ >= 10) {
    ecn_capable_network_ = (ce_count_ > 0) || 
                           (static_cast<double>(ect_count_) / 
                            (ect_count_ + ce_count_) > 0.8);
  }
}

}  // namespace webrtc
