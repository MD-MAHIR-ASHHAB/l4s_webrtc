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

namespace webrtc {

L4SNetworkController::L4SNetworkController(
    NetworkControllerConfig config,
    L4SControllerConfig l4s_config)
    : env_(config.env),
      fallback_to_gcc_(l4s_config.fallback_to_gcc),
      use_ect1_marking_(l4s_config.use_ect1_marking),
      prague_controller_(std::make_unique<L4SPragueController>(env_.field_trials())),
      probe_controller_(std::make_unique<ProbeController>(&config.env.field_trials(), 
                                                         config.env.event_log())) {
  
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

NetworkControlUpdate L4SNetworkController::OnTransportPacketsFeedback(
    TransportPacketsFeedback feedback) {
  NetworkControlUpdate update;
  
  // Process ECN feedback to detect if ECN is supported
  ProcessEcnFeedback(feedback);
  
  // Update Prague controller with ECN feedback
  prague_controller_->UpdateEcnFeedback(feedback);
  
  // Get updated target rate if L4S is active
  if (IsL4SActive()) {
    auto prague_rate = prague_controller_->GetTargetRate(feedback.feedback_time);
    if (prague_rate) {
      target_rate_ = prague_rate;
      MaybeTriggerOnNetworkChanged(&update, feedback.feedback_time);
    }
  } else if (fallback_to_gcc_) {
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

NetworkControlUpdate L4SNetworkController::CreateRateUpdate(
    Timestamp at_time) const {
  NetworkControlUpdate update;
  
  // Apply rate constraints
  DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  if (min_target_rate_ && current_rate < *min_target_rate_) {
    current_rate = *min_target_rate_;
  }
  if (max_target_rate_ && current_rate > *max_target_rate_) {
    current_rate = *max_target_rate_;
  }
  
  // Set target rate and bandwidth estimate
  update.target_rate = TargetTransferRate();
  update.target_rate->network_estimate.at_time = at_time;
  update.target_rate->network_estimate.bandwidth = current_rate;
  update.target_rate->network_estimate.loss_rate_ratio = 0.0f;
  update.target_rate->network_estimate.round_trip_time = TimeDelta::Millis(50);
  update.target_rate->network_estimate.bwe_period = TimeDelta::Millis(500);
  update.target_rate->target_rate = current_rate;
  
  // Set pacer config
  update.pacer_config = PacerConfig();
  update.pacer_config->at_time = at_time;
  update.pacer_config->time_window = TimeDelta::Millis(500);
  
  // For L4S, we want a smaller pacer queue to reduce delay
  const DataRate pacing_rate = current_rate * 1.5;
  update.pacer_config->data_window = DataSize::Bytes(
      (pacing_rate * TimeDelta::Millis(50)).bytes());
  update.pacer_config->pad_window = DataSize::Zero();
  update.pacer_config->pad_rate() = DataRate::Zero();

  return update;
}

void L4SNetworkController::MaybeTriggerOnNetworkChanged(
    NetworkControlUpdate* update,
    Timestamp at_time) {
  if (!update || !target_rate_) {
    return;
  }
  
  // Create rate update
  *update = CreateRateUpdate(at_time);
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