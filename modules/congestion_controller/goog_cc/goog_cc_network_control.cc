/*
 *  Copyright (c) 2018 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/goog_cc/goog_cc_network_control.h"

#include <stdio.h>

#include <algorithm>
#include <cstdint>
#include <unistd.h>
#include <memory>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "api/environment/environment.h"
#include "api/field_trials_view.h"
#include "api/transport/bandwidth_usage.h"
#include "api/transport/network_control.h"
#include "api/transport/network_types.h"
#include "api/units/data_rate.h"
#include "api/units/data_size.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "logging/rtc_event_log/events/rtc_event_remote_estimate.h"
#include "modules/congestion_controller/goog_cc/acknowledged_bitrate_estimator_interface.h"
#include "modules/congestion_controller/goog_cc/alr_detector.h"
#include "modules/congestion_controller/goog_cc/congestion_window_pushback_controller.h"
#include "modules/congestion_controller/goog_cc/delay_based_bwe.h"
#include "modules/congestion_controller/goog_cc/loss_based_bwe_v2.h"
#include "modules/congestion_controller/goog_cc/probe_bitrate_estimator.h"
#include "modules/congestion_controller/goog_cc/probe_controller.h"
#include "modules/congestion_controller/goog_cc/send_side_bandwidth_estimation.h"
#include "modules/remote_bitrate_estimator/include/bwe_defines.h"
#include "rtc_base/checks.h"
#include "rtc_base/experiments/field_trial_parser.h"
#include "rtc_base/experiments/rate_control_settings.h"
#include "rtc_base/logging.h"
#include "api/test/metrics/global_metrics_logger_and_exporter.h"
#include "api/test/metrics/metrics_logger.h"

namespace webrtc {

namespace {
// From RTCPSender video report interval.
constexpr TimeDelta kLossUpdateInterval = TimeDelta::Millis(1000);

// Pacing-rate relative to our target send rate.
// Multiplicative factor that is applied to the target bitrate to calculate
// the number of bytes that can be transmitted per interval.
// Increasing this factor will result in lower delays in cases of bitrate
// overshoots from the encoder.
constexpr float kDefaultPaceMultiplier = 2.5f;

// If the probe result is far below the current throughput estimate
// it's unlikely that the probe is accurate, so we don't want to drop too far.
// However, if we actually are overusing, we want to drop to something slightly
// below the current throughput estimate to drain the network queues.
constexpr double kProbeDropThroughputFraction = 0.85;

BandwidthLimitedCause GetBandwidthLimitedCause(LossBasedState loss_based_state,
                                               bool is_rtt_above_limit,
                                               BandwidthUsage bandwidth_usage) {
  if (bandwidth_usage == BandwidthUsage::kBwOverusing ||
      bandwidth_usage == BandwidthUsage::kBwUnderusing) {
    return BandwidthLimitedCause::kDelayBasedLimitedDelayIncreased;
  } else if (is_rtt_above_limit) {
    return BandwidthLimitedCause::kRttBasedBackOffHighRtt;
  }
  switch (loss_based_state) {
    case LossBasedState::kDecreasing:
      // Probes may not be sent in this state.
      return BandwidthLimitedCause::kLossLimitedBwe;
    case LossBasedState::kIncreaseUsingPadding:
      // Probes may not be sent in this state.
      return BandwidthLimitedCause::kLossLimitedBwe;
    case LossBasedState::kIncreasing:
      // Probes may be sent in this state.
      return BandwidthLimitedCause::kLossLimitedBweIncreasing;
    case LossBasedState::kDelayBasedEstimate:
      return BandwidthLimitedCause::kDelayBasedLimited;
  }
}

}  // namespace

GoogCcNetworkController::GoogCcNetworkController(NetworkControllerConfig config,
                                                 GoogCcConfig goog_cc_config,
                                                 test::MetricsLogger* metrics_logger)
    : env_(config.env),
      packet_feedback_only_(false),
      safe_reset_on_route_change_("Enabled"),
      safe_reset_acknowledged_rate_("ack"),
      use_min_allocatable_as_lower_bound_(
          !env_.field_trials().IsDisabled("WebRTC-Bwe-MinAllocAsLowerBound")),
      ignore_probes_lower_than_network_estimate_(
          !env_.field_trials().IsDisabled(
              "WebRTC-Bwe-IgnoreProbesLowerThanNetworkStateEstimate")),
      limit_probes_lower_than_throughput_estimate_(
          !env_.field_trials().IsDisabled(
              "WebRTC-Bwe-LimitProbesLowerThanThroughputEstimate")),
      rate_control_settings_(env_.field_trials()),
      limit_pacingfactor_by_upper_link_capacity_estimate_(
          env_.field_trials().IsEnabled(
              "WebRTC-Bwe-LimitPacingFactorByUpperLinkCapacityEstimate")),
      probe_controller_(
          new ProbeController(&env_.field_trials(), &env_.event_log())),
      congestion_window_pushback_controller_(
          rate_control_settings_.UseCongestionWindowPushback()
              ? std::make_unique<CongestionWindowPushbackController>(
                    env_.field_trials())
              : nullptr),
      bandwidth_estimation_(
          std::make_unique<SendSideBandwidthEstimation>(&env_.field_trials(),
                                                        &env_.event_log())),
      alr_detector_(std::make_unique<AlrDetector>(&env_.field_trials(),
                                                  &env_.event_log())),
      ecn_based_bwe_(std::make_unique<EcnBasedBwe>()),
      probe_bitrate_estimator_(new ProbeBitrateEstimator(&env_.event_log())),
      network_estimator_(std::move(goog_cc_config.network_state_estimator)),
      network_state_predictor_(
          std::move(goog_cc_config.network_state_predictor)),
      delay_based_bwe_(new DelayBasedBwe(&env_.field_trials(),
                                         &env_.event_log(),
                                         network_state_predictor_.get())),
      acknowledged_bitrate_estimator_(
          AcknowledgedBitrateEstimatorInterface::Create(&env_.field_trials())),
      initial_config_(config),
      last_loss_based_target_rate_(*config.constraints.starting_rate),
      last_pushback_target_rate_(last_loss_based_target_rate_),
      last_stable_target_rate_(last_loss_based_target_rate_),
      last_loss_base_state_(LossBasedState::kDelayBasedEstimate),
      pacing_factor_(config.stream_based_config.pacing_factor.value_or(
          kDefaultPaceMultiplier)),
      min_total_allocated_bitrate_(
          config.stream_based_config.min_total_allocated_bitrate.value_or(
              DataRate::Zero())),
      max_padding_rate_(config.stream_based_config.max_padding_rate.value_or(
          DataRate::Zero())),
      metrics_enabled_(true),
      current_active_controller_("GCC_initializing") {

  using webrtc::test::GetGlobalMetricsLogger;
  test::MetricsLogger* logger_to_use = metrics_logger;
  if (!logger_to_use) {
    logger_to_use = GetGlobalMetricsLogger();
  }
  metrics_collector_ = std::make_unique<GCCMetricsCollector>(
      logger_to_use, goog_cc_config.test_case_name, &env_.clock());
  RTC_LOG(LS_INFO) << "GCC: Metrics collection enabled for test case: "
                   << goog_cc_config.test_case_name;


  RTC_DCHECK(config.constraints.at_time.IsFinite());
  ParseFieldTrial(
      {&safe_reset_on_route_change_, &safe_reset_acknowledged_rate_},
      env_.field_trials().Lookup("WebRTC-Bwe-SafeResetOnRouteChange"));
  if (delay_based_bwe_)
    delay_based_bwe_->SetMinBitrate(kCongestionControllerMinBitrate);
}

GoogCcNetworkController::~GoogCcNetworkController() {}

NetworkControlUpdate GoogCcNetworkController::OnNetworkAvailability(
    NetworkAvailability msg) {
  NetworkControlUpdate update;
  update.probe_cluster_configs = probe_controller_->OnNetworkAvailability(msg);
  return update;
}

NetworkControlUpdate GoogCcNetworkController::OnNetworkRouteChange(
    NetworkRouteChange msg) {
  if (safe_reset_on_route_change_) {
    std::optional<DataRate> estimated_bitrate;
    if (safe_reset_acknowledged_rate_) {
      estimated_bitrate = acknowledged_bitrate_estimator_->bitrate();
      if (!estimated_bitrate)
        estimated_bitrate = acknowledged_bitrate_estimator_->PeekRate();
    } else {
      estimated_bitrate = bandwidth_estimation_->target_rate();
    }
    if (estimated_bitrate) {
      if (msg.constraints.starting_rate) {
        msg.constraints.starting_rate =
            std::min(*msg.constraints.starting_rate, *estimated_bitrate);
      } else {
        msg.constraints.starting_rate = estimated_bitrate;
      }
    }
  }

  acknowledged_bitrate_estimator_ =
      AcknowledgedBitrateEstimatorInterface::Create(&env_.field_trials());
  probe_bitrate_estimator_.reset(new ProbeBitrateEstimator(&env_.event_log()));
  if (network_estimator_)
    network_estimator_->OnRouteChange(msg);
  delay_based_bwe_.reset(new DelayBasedBwe(
      &env_.field_trials(), &env_.event_log(), network_state_predictor_.get()));
  bandwidth_estimation_->OnRouteChange();
  probe_controller_->Reset(msg.at_time);
  NetworkControlUpdate update;
  update.probe_cluster_configs = ResetConstraints(msg.constraints);
  MaybeTriggerOnNetworkChanged(&update, msg.at_time);
  return update;
}

NetworkControlUpdate GoogCcNetworkController::OnProcessInterval(
    ProcessInterval msg) {
  NetworkControlUpdate update;
  if (initial_config_) {
    update.probe_cluster_configs =
        ResetConstraints(initial_config_->constraints);
    update.pacer_config = GetPacingRates(msg.at_time);

    if (initial_config_->stream_based_config.requests_alr_probing) {
      probe_controller_->EnablePeriodicAlrProbing(
          *initial_config_->stream_based_config.requests_alr_probing);
    }
    if (initial_config_->stream_based_config.enable_repeated_initial_probing) {
      probe_controller_->EnableRepeatedInitialProbing(
          *initial_config_->stream_based_config
               .enable_repeated_initial_probing);
    }
    std::optional<DataRate> total_bitrate =
        initial_config_->stream_based_config.max_total_allocated_bitrate;
    if (total_bitrate) {
      auto probes = probe_controller_->OnMaxTotalAllocatedBitrate(
          *total_bitrate, msg.at_time);
      update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                          probes.begin(), probes.end());
    }
    initial_config_.reset();
  }
  if (congestion_window_pushback_controller_ && msg.pacer_queue) {
    congestion_window_pushback_controller_->UpdatePacingQueue(
        msg.pacer_queue->bytes());
  }
  bandwidth_estimation_->UpdateEstimate(msg.at_time);
  std::optional<int64_t> start_time_ms =
      alr_detector_->GetApplicationLimitedRegionStartTime();
  probe_controller_->SetAlrStartTimeMs(start_time_ms);

  auto probes = probe_controller_->Process(msg.at_time);
  update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                      probes.begin(), probes.end());

  if (rate_control_settings_.UseCongestionWindow() &&
      !feedback_max_rtts_.empty()) {
    UpdateCongestionWindowSize();
  }
  if (congestion_window_pushback_controller_ && current_data_window_) {
    congestion_window_pushback_controller_->SetDataWindow(
        *current_data_window_);
  } else {
    update.congestion_window = current_data_window_;
  }

  // Log metrics periodically on each process interval
  LogPeriodicMetrics(msg.at_time);
  MaybeTriggerOnNetworkChanged(&update, msg.at_time);
 
 
  return update;

}

NetworkControlUpdate GoogCcNetworkController::OnRemoteBitrateReport(
    RemoteBitrateReport msg) {
  if (packet_feedback_only_) {
    RTC_LOG(LS_ERROR) << "Received REMB for packet feedback only GoogCC";
    return NetworkControlUpdate();
  }
  bandwidth_estimation_->UpdateReceiverEstimate(msg.receive_time,
                                                msg.bandwidth);
  return NetworkControlUpdate();
}

NetworkControlUpdate GoogCcNetworkController::OnRoundTripTimeUpdate(
    RoundTripTimeUpdate msg) {
  RTC_LOG(LS_VERBOSE) << "GCC RTT update: " << msg.round_trip_time.ms()
                      << " ms";
  if (msg.smoothed) {
    return NetworkControlUpdate();
  }
  RTC_DCHECK(!msg.round_trip_time.IsZero());

  last_rtt_ = msg.round_trip_time;
  
  // Log RTT metrics.
  if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->LogDelayMetrics(
        Timestamp::Millis(env_.clock().TimeInMilliseconds()),
        msg.round_trip_time, msg.round_trip_time / 2, jitter_);
  }

  if (delay_based_bwe_)
    delay_based_bwe_->OnRttUpdate(msg.round_trip_time);
  bandwidth_estimation_->UpdateRtt(msg.round_trip_time, msg.receive_time);
  return NetworkControlUpdate();
}

NetworkControlUpdate GoogCcNetworkController::OnSentPacket(
    SentPacket sent_packet) {
  alr_detector_->OnBytesSent(sent_packet.size.bytes(),
                             sent_packet.send_time.ms());
  acknowledged_bitrate_estimator_->SetAlr(
      alr_detector_->GetApplicationLimitedRegionStartTime().has_value());

  if (!first_packet_sent_) {
    first_packet_sent_ = true;
    // Initialize feedback time to send time to allow estimation of RTT until
    // first feedback is received.
    bandwidth_estimation_->UpdatePropagationRtt(sent_packet.send_time,
                                                TimeDelta::Zero());
  }
  bandwidth_estimation_->OnSentPacket(sent_packet);


  // --- NEW: Calculate Pacer's Actual Send Rate ---
  constexpr TimeDelta kSendWindow = TimeDelta::Millis(500); 
  
  if (sent_packet.send_time.IsFinite()) {
    send_rate_window_.emplace_back(sent_packet.send_time, sent_packet.size.bytes());
  }

  // Evict packets older than the window
  while (!send_rate_window_.empty() && 
         (sent_packet.send_time - send_rate_window_.front().first) > kSendWindow) {
    send_rate_window_.pop_front();
  }

  // Calculate the transmission rate if we have a valid time gap
  if (send_rate_window_.size() > 1) {
    Timestamp window_start = send_rate_window_.front().first;
    Timestamp window_end = send_rate_window_.back().first;
    TimeDelta window_interval = window_end - window_start;

    if (window_interval >= TimeDelta::Millis(100)) {
      int64_t window_bytes = 0;
      for (const auto& entry : send_rate_window_) {
        window_bytes += entry.second;
      }
      last_send_rate_ = DataRate::BitsPerSec(
          static_cast<int64_t>((window_bytes * 8) / window_interval.seconds<double>()));
    }
  }

  if (congestion_window_pushback_controller_) {
    congestion_window_pushback_controller_->UpdateOutstandingData(
        sent_packet.data_in_flight.bytes());
    NetworkControlUpdate update;
    MaybeTriggerOnNetworkChanged(&update, sent_packet.send_time);
    return update;
  } else {
    return NetworkControlUpdate();
  }
}

NetworkControlUpdate GoogCcNetworkController::OnReceivedPacket(
    ReceivedPacket /* received_packet */) {
  return NetworkControlUpdate();
}

NetworkControlUpdate GoogCcNetworkController::OnStreamsConfig(
    StreamsConfig msg) {
  NetworkControlUpdate update;
  if (msg.requests_alr_probing) {
    probe_controller_->EnablePeriodicAlrProbing(*msg.requests_alr_probing);
  }
  if (msg.max_total_allocated_bitrate) {
    update.probe_cluster_configs =
        probe_controller_->OnMaxTotalAllocatedBitrate(
            *msg.max_total_allocated_bitrate, msg.at_time);
  }

  bool pacing_changed = false;
  if (msg.pacing_factor && *msg.pacing_factor != pacing_factor_) {
    pacing_factor_ = *msg.pacing_factor;
    pacing_changed = true;
  }
  if (msg.min_total_allocated_bitrate &&
      *msg.min_total_allocated_bitrate != min_total_allocated_bitrate_) {
    min_total_allocated_bitrate_ = *msg.min_total_allocated_bitrate;
    pacing_changed = true;

    if (use_min_allocatable_as_lower_bound_) {
      ClampConstraints();
      delay_based_bwe_->SetMinBitrate(min_data_rate_);
      bandwidth_estimation_->SetMinMaxBitrate(min_data_rate_, max_data_rate_);
    }
  }
  if (msg.max_padding_rate && *msg.max_padding_rate != max_padding_rate_) {
    max_padding_rate_ = *msg.max_padding_rate;
    pacing_changed = true;
  }

  if (pacing_changed)
    update.pacer_config = GetPacingRates(msg.at_time);
  return update;
}

NetworkControlUpdate GoogCcNetworkController::OnTargetRateConstraints(
    TargetRateConstraints constraints) {
  NetworkControlUpdate update;
  update.probe_cluster_configs = ResetConstraints(constraints);
  MaybeTriggerOnNetworkChanged(&update, constraints.at_time);
  return update;
}

void GoogCcNetworkController::ClampConstraints() {
  // TODO(holmer): We should make sure the default bitrates are set to 10 kbps,
  // and that we don't try to set the min bitrate to 0 from any applications.
  // The congestion controller should allow a min bitrate of 0.
  min_data_rate_ = std::max(min_target_rate_, kCongestionControllerMinBitrate);
  if (use_min_allocatable_as_lower_bound_) {
    min_data_rate_ = std::max(min_data_rate_, min_total_allocated_bitrate_);
  }
  if (max_data_rate_ < min_data_rate_) {
    RTC_LOG(LS_WARNING) << "max bitrate smaller than min bitrate";
    max_data_rate_ = min_data_rate_;
  }
  if (starting_rate_ && starting_rate_ < min_data_rate_) {
    RTC_LOG(LS_WARNING) << "start bitrate smaller than min bitrate";
    starting_rate_ = min_data_rate_;
  }
}

std::vector<ProbeClusterConfig> GoogCcNetworkController::ResetConstraints(
    TargetRateConstraints new_constraints) {
  min_target_rate_ = new_constraints.min_data_rate.value_or(DataRate::Zero());
  max_data_rate_ =
      new_constraints.max_data_rate.value_or(DataRate::PlusInfinity());
  starting_rate_ = new_constraints.starting_rate;
  ClampConstraints();

  bandwidth_estimation_->SetBitrates(starting_rate_, min_data_rate_,
                                     max_data_rate_, new_constraints.at_time);

  if (starting_rate_)
    delay_based_bwe_->SetStartBitrate(*starting_rate_);
  delay_based_bwe_->SetMinBitrate(min_data_rate_);

  if (ecn_based_bwe_) {
    ecn_based_bwe_->SetMinMaxBitrate(min_data_rate_, max_data_rate_);
  }

  return probe_controller_->SetBitrates(
      min_data_rate_, starting_rate_.value_or(DataRate::Zero()), max_data_rate_,
      new_constraints.at_time);
}

NetworkControlUpdate GoogCcNetworkController::OnTransportLossReport(
    TransportLossReport msg) {
  int64_t total_packets_delta =
      msg.packets_received_delta + msg.packets_lost_delta;

  // 1. Update logging metrics FIRST (Matches Candidate 1 baseline)
  if (total_packets_delta > 0) {
    last_loss_fraction_ = static_cast<double>(msg.packets_lost_delta) / total_packets_delta;
  } else {
    last_loss_fraction_ = 0.0;
  }
  last_packets_lost_ = static_cast<int>(msg.packets_lost_delta);
  if (last_packets_lost_ > 0) {
    RTC_LOG(LS_INFO) << "GCC: Transport loss report - "
                 << "Lost: " << msg.packets_lost_delta
                 << ", Received: " << msg.packets_received_delta
                 << ", Loss Fraction: " << last_loss_fraction_;
  }
  // 2. NOW check if we should short-circuit the bandwidth estimator update
  if (packet_feedback_only_) {
    return NetworkControlUpdate();
  }

  // 3. Otherwise, pass the physical loss to the legacy bandwidth estimator
  bandwidth_estimation_->UpdatePacketsLost(
      msg.packets_lost_delta, total_packets_delta, msg.receive_time);
      
  return NetworkControlUpdate();
}
void GoogCcNetworkController::UpdateCongestionWindowSize() {
  TimeDelta min_feedback_max_rtt = TimeDelta::Millis(
      *std::min_element(feedback_max_rtts_.begin(), feedback_max_rtts_.end()));

  const DataSize kMinCwnd = DataSize::Bytes(2 * 1500);
  TimeDelta time_window =
      min_feedback_max_rtt +
      TimeDelta::Millis(
          rate_control_settings_.GetCongestionWindowAdditionalTimeMs());

  DataSize data_window = last_loss_based_target_rate_ * time_window;
  if (current_data_window_) {
    data_window =
        std::max(kMinCwnd, (data_window + current_data_window_.value()) / 2);
  } else {
    data_window = std::max(kMinCwnd, data_window);
  }
  current_data_window_ = data_window;
}

NetworkControlUpdate GoogCcNetworkController::OnTransportPacketsFeedback(
    TransportPacketsFeedback report) {


  //     // --- ADD THIS VERIFICATION LOG ---
  // if (!report.packet_feedbacks.empty()) {
  //   RTC_LOG(LS_INFO) << "[GCC FRONT DOOR] Received feedback batch! "
  //                    << report.packet_feedbacks.size() << " packets. "
  //                    << "CE Count: " << report.ce_count;
  // }


  if (report.packet_feedbacks.empty()) {
    // TODO(bugs.webrtc.org/10125): Design a better mechanism to safe-guard
    // against building very large network queues.
    return NetworkControlUpdate();
  }

  if (congestion_window_pushback_controller_) {
    congestion_window_pushback_controller_->UpdateOutstandingData(
        report.data_in_flight.bytes());
  }
  TimeDelta max_feedback_rtt = TimeDelta::MinusInfinity();
  TimeDelta min_propagation_rtt = TimeDelta::PlusInfinity();
  Timestamp max_recv_time = Timestamp::MinusInfinity();

  std::vector<PacketResult> feedbacks = report.ReceivedWithSendInfo();
  for (const auto& feedback : feedbacks)
    max_recv_time = std::max(max_recv_time, feedback.receive_time);

  for (const auto& feedback : feedbacks) {
    TimeDelta feedback_rtt =
        report.feedback_time - feedback.sent_packet.send_time;
    TimeDelta min_pending_time = max_recv_time - feedback.receive_time;
    TimeDelta propagation_rtt = feedback_rtt - min_pending_time;
    max_feedback_rtt = std::max(max_feedback_rtt, feedback_rtt);
    min_propagation_rtt = std::min(min_propagation_rtt, propagation_rtt);
  }

  if (max_feedback_rtt.IsFinite()) {
    feedback_max_rtts_.push_back(max_feedback_rtt.ms());
    const size_t kMaxFeedbackRttWindow = 32;
    if (feedback_max_rtts_.size() > kMaxFeedbackRttWindow)
      feedback_max_rtts_.pop_front();
    // TODO(srte): Use time since last unacknowledged packet.
    bandwidth_estimation_->UpdatePropagationRtt(report.feedback_time,
                                                min_propagation_rtt);
  }
  if (packet_feedback_only_) {
    if (!feedback_max_rtts_.empty()) {
      int64_t sum_rtt_ms =
          std::accumulate(feedback_max_rtts_.begin(), feedback_max_rtts_.end(),
                          static_cast<int64_t>(0));
      int64_t mean_rtt_ms = sum_rtt_ms / feedback_max_rtts_.size();
      if (delay_based_bwe_)
        delay_based_bwe_->OnRttUpdate(TimeDelta::Millis(mean_rtt_ms));
    }

    TimeDelta feedback_min_rtt = TimeDelta::PlusInfinity();
    for (const auto& packet_feedback : feedbacks) {
      TimeDelta pending_time = max_recv_time - packet_feedback.receive_time;
      TimeDelta rtt = report.feedback_time -
                      packet_feedback.sent_packet.send_time - pending_time;
      // Value used for predicting NACK round trip time in FEC controller.
      feedback_min_rtt = std::min(rtt, feedback_min_rtt);
    }
    if (feedback_min_rtt.IsFinite()) {
      bandwidth_estimation_->UpdateRtt(feedback_min_rtt, report.feedback_time);
    }
  // Keep ECN Controller's RTT synchronized
    if (ecn_based_bwe_) {
      ecn_based_bwe_->UpdateRtt(feedback_min_rtt);
    }

    expected_packets_since_last_loss_update_ +=
        report.PacketsWithFeedback().size();
    for (const auto& packet_feedback : report.PacketsWithFeedback()) {
      if (!packet_feedback.IsReceived())
        lost_packets_since_last_loss_update_ += 1;
    }
    if (report.feedback_time > next_loss_update_) {
      next_loss_update_ = report.feedback_time + kLossUpdateInterval;
      bandwidth_estimation_->UpdatePacketsLost(
          lost_packets_since_last_loss_update_,
          expected_packets_since_last_loss_update_, report.feedback_time);
      expected_packets_since_last_loss_update_ = 0;
      lost_packets_since_last_loss_update_ = 0;
    }
  }
  std::optional<int64_t> alr_start_time =
      alr_detector_->GetApplicationLimitedRegionStartTime();

  if (previously_in_alr_ && !alr_start_time.has_value()) {
    int64_t now_ms = report.feedback_time.ms();
    acknowledged_bitrate_estimator_->SetAlrEndedTime(report.feedback_time);
    probe_controller_->SetAlrEndedTimeMs(now_ms);
  }
  previously_in_alr_ = alr_start_time.has_value();
  acknowledged_bitrate_estimator_->IncomingPacketFeedbackVector(
      report.SortedByReceiveTime());
  auto acknowledged_bitrate = acknowledged_bitrate_estimator_->bitrate();
  bandwidth_estimation_->SetAcknowledgedRate(acknowledged_bitrate,
                                             report.feedback_time);
  for (const auto& feedback : report.SortedByReceiveTime()) {
    if (feedback.sent_packet.pacing_info.probe_cluster_id !=
        PacedPacketInfo::kNotAProbe) {
      probe_bitrate_estimator_->HandleProbeAndEstimateBitrate(feedback);
    }
  }

  if (network_estimator_) {
    network_estimator_->OnTransportPacketsFeedback(report);
    SetNetworkStateEstimate(network_estimator_->GetCurrentEstimate());
  }
  std::optional<DataRate> probe_bitrate =
      probe_bitrate_estimator_->FetchAndResetLastEstimatedBitrate();
  if (ignore_probes_lower_than_network_estimate_ && probe_bitrate &&
      estimate_ && *probe_bitrate < delay_based_bwe_->last_estimate() &&
      *probe_bitrate < estimate_->link_capacity_lower) {
    probe_bitrate.reset();
  }
  if (limit_probes_lower_than_throughput_estimate_ && probe_bitrate &&
      acknowledged_bitrate) {
    // Limit the backoff to something slightly below the acknowledged
    // bitrate. ("Slightly below" because we want to drain the queues
    // if we are actually overusing.)
    // The acknowledged bitrate shouldn't normally be higher than the delay
    // based estimate, but it could happen e.g. due to packet bursts or
    // encoder overshoot. We use std::min to ensure that a probe result
    // below the current BWE never causes an increase.
    DataRate limit =
        std::min(delay_based_bwe_->last_estimate(),
                 *acknowledged_bitrate * kProbeDropThroughputFraction);
    probe_bitrate = std::max(*probe_bitrate, limit);
  }

  NetworkControlUpdate update;
  bool recovered_from_overuse = false;

  DelayBasedBwe::Result result;
  result = delay_based_bwe_->IncomingPacketFeedbackVector(
      report, acknowledged_bitrate, probe_bitrate, estimate_,
      alr_start_time.has_value());


   // --- 2. PLUG IN YOUR ECN-BASED BWE ---
  // Treat ECN-based BWE as a cap provider rather than an override. If the ECN
  // module observed CE marks it will request a multiplicative decrease; we
  // enforce that by capping the delay-based target or, if delay-based did not
  // produce an update, by producing an update that lowers the current target.
  std::optional<DataRate> ecn_cap;
  if (ecn_based_bwe_ && report.transport_supports_ecn) {

        // --- EXACT SYNC FIX ---
    // Force the ECN module's internal state to exactly match GCC's overarching target
    // before computing any Multiplicative Decreases.
    ecn_based_bwe_->SetTargetBitrate(bandwidth_estimation_->target_rate());
    
    EcnBasedBwe::ECNResult ecn_result = ecn_based_bwe_->IncomingPacketFeedbackVector(
        report, acknowledged_bitrate, probe_bitrate, estimate_,
        alr_start_time.has_value());

    if (ecn_result.updated) {
      ecn_cap = ecn_result.target_bitrate;
      RTC_LOG(LS_VERBOSE) << "[ECN BWE] Suggested cap: "
                          << ecn_cap->kbps() << " kbps";
    }
  }

  // If ECN provided a cap, apply it conservatively.
  if (ecn_cap.has_value()) {
    if (result.updated) {
// // Apply cap to the delay-based target.
//       if (*ecn_cap < result.target_bitrate) {
//           result.target_bitrate = *ecn_cap;
          
//           // NEW: Force DelayBasedBwe's internal state down to the ECN cap
//           delay_based_bwe_->SetStartBitrate(*ecn_cap); 
//       }

      // Apply cap to the delay-based target.
      result.target_bitrate = std::min(result.target_bitrate, *ecn_cap);
      RTC_LOG(LS_VERBOSE) << "[ECN BWE] Capped GCC target rate to: "
                          << result.target_bitrate.kbps() << " kbps";
    } else {
      // Delay-based didn't request an update, but ECN demands a reduction.
      // Force an update to lower the current target to the ECN cap (or keep
      // current target if already below the cap).
      DataRate current_target = bandwidth_estimation_->target_rate();
      DataRate new_target = std::min(current_target, *ecn_cap);
      if (new_target < current_target) {
        result.updated = true;
        result.target_bitrate = new_target;
        result.recovered_from_overuse = false;

        // // NEW: Force DelayBasedBwe's internal state down to the ECN cap
        // delay_based_bwe_->SetStartBitrate(new_target);

        RTC_LOG(LS_VERBOSE) << "[ECN BWE] Forcing reduction to ECN cap: "
                            << result.target_bitrate.kbps() << " kbps";
      }
    }
  }
  // -------------------------------------- 

  if (result.updated) {
    if (result.probe) {
      bandwidth_estimation_->SetSendBitrate(result.target_bitrate,
                                            report.feedback_time);
    }
    // Since SetSendBitrate now resets the delay-based estimate, we have to
    // call UpdateDelayBasedEstimate after SetSendBitrate.
    bandwidth_estimation_->UpdateDelayBasedEstimate(report.feedback_time,
                                                    result.target_bitrate);
  }
  bandwidth_estimation_->UpdateLossBasedEstimator(
      report, result.delay_detector_state, probe_bitrate,
      alr_start_time.has_value());
  if (result.updated) {
    // Update the estimate in the ProbeController, in case we want to probe.
    MaybeTriggerOnNetworkChanged(&update, report.feedback_time);
  }

  recovered_from_overuse = result.recovered_from_overuse;

  if (recovered_from_overuse) {
    probe_controller_->SetAlrStartTimeMs(alr_start_time);
    auto probes = probe_controller_->RequestProbe(report.feedback_time);
    update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                        probes.begin(), probes.end());
  }

  // No valid RTT could be because send-side BWE isn't used, in which case
  // we don't try to limit the outstanding packets.
  if (rate_control_settings_.UseCongestionWindow() &&
      max_feedback_rtt.IsFinite()) {
    UpdateCongestionWindowSize();
  }
  if (congestion_window_pushback_controller_ && current_data_window_) {
    congestion_window_pushback_controller_->SetDataWindow(
        *current_data_window_);
  } else {
    update.congestion_window = current_data_window_;
  }


  // --- Throughput calculation using a sliding window for stability ---
  constexpr TimeDelta kThroughputWindow = TimeDelta::Millis(500);
  Timestamp now = report.feedback_time;

  // Add new packets to the window
  for (const auto& packet : report.packet_feedbacks) {
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
  Timestamp window_start = now;
  Timestamp window_end = now;
  if (!throughput_window_.empty()) {
    window_start = throughput_window_.front().first;
    window_end = throughput_window_.back().first;
    for (const auto& entry : throughput_window_) {
      window_bytes += entry.second;
    }
  }
  TimeDelta window_interval = window_end - window_start;
  if (window_interval > TimeDelta::Millis(1)) {
    last_actual_bitrate_ = DataRate::BitsPerSec(static_cast<int64_t>((window_bytes * 8) / window_interval.seconds<double>()));
    
    RTC_LOG(LS_VERBOSE) << "GCC Throughput Calculation: "
              << "packets=" << throughput_window_.size()
              << " bytes=" << window_bytes
              << " interval=" << window_interval.ms() << " ms"
              << " throughput=" << last_actual_bitrate_.bps() / 1e6
              << " Mbps"
              << " ack_estimate="
              << (acknowledged_bitrate_estimator_->bitrate()
                    .has_value()
                  ? acknowledged_bitrate_estimator_->bitrate()
                      ->bps() /
                      1e6
                  : 0.0)
              << " Mbps";
  } else {
    last_actual_bitrate_ = DataRate::Zero();
  }



  return update;
}

NetworkControlUpdate GoogCcNetworkController::OnNetworkStateEstimate(
    NetworkStateEstimate msg) {
  if (!network_estimator_) {
    SetNetworkStateEstimate(msg);
  }
  return NetworkControlUpdate();
}

void GoogCcNetworkController::SetNetworkStateEstimate(
    std::optional<NetworkStateEstimate> estimate) {
  auto prev_estimate = estimate_;
  estimate_ = estimate;
  if (estimate_ && (!prev_estimate ||
                    estimate_->update_time != prev_estimate->update_time)) {
    env_.event_log().Log(std::make_unique<RtcEventRemoteEstimate>(
        estimate_->link_capacity_lower, estimate_->link_capacity_upper));
    probe_controller_->SetNetworkStateEstimate(*estimate_);
  }
}

NetworkControlUpdate GoogCcNetworkController::GetNetworkState(
    Timestamp at_time) const {
  NetworkControlUpdate update;
  update.target_rate = TargetTransferRate();
  update.target_rate->network_estimate.at_time = at_time;
  update.target_rate->network_estimate.loss_rate_ratio =
      last_estimated_fraction_loss_.value_or(0) / 255.0;
  update.target_rate->network_estimate.round_trip_time =
      last_estimated_round_trip_time_;
  update.target_rate->network_estimate.bwe_period =
      delay_based_bwe_->GetExpectedBwePeriod();

  update.target_rate->at_time = at_time;
  update.target_rate->target_rate = last_pushback_target_rate_;
  update.target_rate->stable_target_rate =
      bandwidth_estimation_->GetEstimatedLinkCapacity();
  update.pacer_config = GetPacingRates(at_time);
  update.congestion_window = current_data_window_;
  return update;
}

void GoogCcNetworkController::MaybeTriggerOnNetworkChanged(
    NetworkControlUpdate* update,
    Timestamp at_time) {
  uint8_t fraction_loss = bandwidth_estimation_->fraction_loss();
  TimeDelta round_trip_time = bandwidth_estimation_->round_trip_time();
  DataRate loss_based_target_rate = bandwidth_estimation_->target_rate();
  LossBasedState loss_based_state = bandwidth_estimation_->loss_based_state();
  DataRate pushback_target_rate = loss_based_target_rate;

  double cwnd_reduce_ratio = 0.0;
  if (congestion_window_pushback_controller_) {
    int64_t pushback_rate =
        congestion_window_pushback_controller_->UpdateTargetBitrate(
            loss_based_target_rate.bps());
    pushback_rate = std::max<int64_t>(bandwidth_estimation_->GetMinBitrate(),
                                      pushback_rate);
    pushback_target_rate = DataRate::BitsPerSec(pushback_rate);
    if (rate_control_settings_.UseCongestionWindowDropFrameOnly()) {
      cwnd_reduce_ratio = static_cast<double>(loss_based_target_rate.bps() -
                                              pushback_target_rate.bps()) /
                          loss_based_target_rate.bps();
    }
  }
  DataRate stable_target_rate =
      bandwidth_estimation_->GetEstimatedLinkCapacity();
  stable_target_rate = std::min(stable_target_rate, pushback_target_rate);

  if ((loss_based_target_rate != last_loss_based_target_rate_) ||
      (loss_based_state != last_loss_base_state_) ||
      (fraction_loss != last_estimated_fraction_loss_) ||
      (round_trip_time != last_estimated_round_trip_time_) ||
      (pushback_target_rate != last_pushback_target_rate_) ||
      (stable_target_rate != last_stable_target_rate_)) {
    last_loss_based_target_rate_ = loss_based_target_rate;
    last_pushback_target_rate_ = pushback_target_rate;
    last_estimated_fraction_loss_ = fraction_loss;
    last_estimated_round_trip_time_ = round_trip_time;
    last_stable_target_rate_ = stable_target_rate;
    last_loss_base_state_ = loss_based_state;

    alr_detector_->SetEstimatedBitrate(loss_based_target_rate.bps());

    TimeDelta bwe_period = delay_based_bwe_->GetExpectedBwePeriod();

    TargetTransferRate target_rate_msg;
    target_rate_msg.at_time = at_time;
    if (rate_control_settings_.UseCongestionWindowDropFrameOnly()) {
      target_rate_msg.target_rate = loss_based_target_rate;
      target_rate_msg.cwnd_reduce_ratio = cwnd_reduce_ratio;
    } else {
      target_rate_msg.target_rate = pushback_target_rate;
    }
    target_rate_msg.stable_target_rate = stable_target_rate;
    target_rate_msg.network_estimate.at_time = at_time;
    target_rate_msg.network_estimate.round_trip_time = round_trip_time;
    target_rate_msg.network_estimate.loss_rate_ratio = fraction_loss / 255.0f;
    target_rate_msg.network_estimate.bwe_period = bwe_period;

    update->target_rate = target_rate_msg;

    auto probes = probe_controller_->SetEstimatedBitrate(
        loss_based_target_rate,
        GetBandwidthLimitedCause(bandwidth_estimation_->loss_based_state(),
                                 bandwidth_estimation_->IsRttAboveLimit(),
                                 delay_based_bwe_->last_state()),
        at_time);
    update->probe_cluster_configs.insert(update->probe_cluster_configs.end(),
                                         probes.begin(), probes.end());
    update->pacer_config = GetPacingRates(at_time);
    RTC_LOG(LS_VERBOSE) << "bwe " << at_time.ms() << " pushback_target_bps="
                        << last_pushback_target_rate_.bps()
                        << " estimate_bps=" << loss_based_target_rate.bps();
  }
    if (metrics_collector_) {
    last_target_rate_ = bandwidth_estimation_->target_rate();
    last_actual_bitrate_ = acknowledged_bitrate_estimator_->bitrate().value_or(DataRate::Zero());
    last_rtt_ = round_trip_time;
    last_loss_fraction_ = fraction_loss / 255.0f;
  }
}

PacerConfig GoogCcNetworkController::GetPacingRates(Timestamp at_time) const {
  // Pacing rate is based on target rate before congestion window pushback,
  // because we don't want to build queues in the pacer when pushback occurs.
  DataRate pacing_rate =
      std::max(min_total_allocated_bitrate_, last_loss_based_target_rate_) *
      pacing_factor_;

  if (limit_pacingfactor_by_upper_link_capacity_estimate_ && estimate_ &&
      estimate_->link_capacity_upper.IsFinite() &&
      pacing_rate > estimate_->link_capacity_upper) {
    pacing_rate =
        std::max({estimate_->link_capacity_upper, min_total_allocated_bitrate_,
                  last_loss_based_target_rate_});
  }

  DataRate padding_rate =
      (last_loss_base_state_ == LossBasedState::kIncreaseUsingPadding)
          ? std::max(max_padding_rate_, last_loss_based_target_rate_)
          : max_padding_rate_;
  padding_rate = std::min(padding_rate, last_pushback_target_rate_);
  PacerConfig msg;
  msg.at_time = at_time;
  msg.time_window = TimeDelta::Seconds(1);
  msg.data_window = pacing_rate * msg.time_window;
  msg.pad_window = padding_rate * msg.time_window;
  return msg;
}




// GCCMetricsCollector implementation
webrtc::GCCMetricsCollector::GCCMetricsCollector(webrtc::test::MetricsLogger* logger,
                                                 const std::string& test_case_name,
                                                 webrtc::Clock* clock)
    : logger_(logger),
      test_case_name_(test_case_name),
      clock_(clock) {
  RTC_CHECK(logger_);
  RTC_CHECK(clock_);
  RTC_LOG(LS_INFO) << "GCCMetricsCollector initialized for test case: " << test_case_name_;
}

void GCCMetricsCollector::LogBandwidthMetrics(
    Timestamp at_time,
    DataRate target_bitrate,
    DataRate actual_bitrate,
    std::optional<DataRate> acked_bitrate,
    std::optional<DataRate> send_rate) {
    
  if (at_time - last_acked_rate_log_ < kAckedRateLogInterval) {
    return;
  }
  last_acked_rate_log_ = at_time;

  // 1. Target Rate (The Software Budget)
  if (target_bitrate.IsFinite()) {
      logger_->LogSingleValueMetric("target_rate_mbps", test_case_name_,
                                    target_bitrate.bps() / 1e6,
                                    webrtc::test::Unit::kUnitless,
                                    webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                    {{"timestamp_ms", std::to_string(at_time.ms())}});
  }

  // 2. Send Rate (The Pacer's Physical Exhaust)
  DataRate tx_rate = send_rate.value_or(DataRate::Zero());
  logger_->LogSingleValueMetric("send_rate_mbps", test_case_name_,
                                tx_rate.bps() / 1e6,
                                webrtc::test::Unit::kUnitless,
                                webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});

  // 3. Actual Rate (Raw Physical Throughput Window)
  logger_->LogSingleValueMetric("actual_rate_mbps", test_case_name_,
                                actual_bitrate.bps() / 1e6,
                                webrtc::test::Unit::kUnitless,
                                webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});

  // 4. Acked Rate (GCC-Parity Kalman Filtered Throughput)
  DataRate filtered_acked_rate = acked_bitrate.value_or(DataRate::Zero());
  UpdateAckedRateStats(filtered_acked_rate); 
  logger_->LogSingleValueMetric("acked_rate_mbps", test_case_name_,
                                filtered_acked_rate.bps() / 1e6,
                                webrtc::test::Unit::kUnitless,
                                webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
}



void GCCMetricsCollector::LogDelayMetrics(Timestamp at_time, TimeDelta rtt, TimeDelta one_way_delay, 
                                         TimeDelta jitter) {
  if (at_time - last_delay_log_ < kDelayLogInterval) {
    return;
  }
  
  last_delay_log_ = at_time;
  UpdateDelayStats(rtt, one_way_delay);
  logger_->LogSingleValueMetric("rtt_ms", test_case_name_, rtt.ms(), 
                                webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  if (one_way_delay.IsFinite()) {
    logger_->LogSingleValueMetric("one_way_delay_ms", test_case_name_, one_way_delay.ms(), 
                                  webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"timestamp_ms", std::to_string(at_time.ms())}});
  }
  if (jitter.IsFinite()) {
    // logger_->LogSingleValueMetric("jitter_ms", test_case_name_, jitter.ms(), 
    //                               webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
    //                               {{"timestamp_ms", std::to_string(at_time.ms())}});
  }
}

void GCCMetricsCollector::LogLossMetrics(Timestamp at_time, double loss_fraction,
                                         int packets_lost_count) {
  if (at_time - last_loss_log_ < kLossLogInterval) {
    return;
  }
  
  last_loss_log_ = at_time;
  UpdateLossStats(loss_fraction);
  
  logger_->LogSingleValueMetric("packet_loss_fraction", test_case_name_, loss_fraction, 
                                webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  logger_->LogSingleValueMetric("packets_lost_count", test_case_name_, packets_lost_count,
                                webrtc::test::Unit::kCount, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
}


void GCCMetricsCollector::LogPeriodicSummary(Timestamp at_time) {
  if (at_time - last_summary_log_ < kSummaryLogInterval) {
    return;
  }
  
  last_summary_log_ = at_time;
  
  // Log summary statistics
  if (acked_rate_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric("acked_rate_avg_mbps", test_case_name_, acked_rate_stats_.GetAverage() / 1e6,
                                  webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "acked_rate"}});
    logger_->LogSingleValueMetric("acked_rate_std_mbps", test_case_name_, acked_rate_stats_.GetStandardDeviation() / 1e6,
                                  webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "std_dev"}, {"metric", "acked_rate"}});
  }

  if (rtt_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric("rtt_avg_ms", test_case_name_, rtt_stats_.GetAverage(),
                                  webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "rtt"}});
    logger_->LogSingleValueMetric("rtt_std_ms", test_case_name_, rtt_stats_.GetStandardDeviation(),
                                  webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "std_dev"}, {"metric", "rtt"}});
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
    logger_->LogSingleValueMetric("packet_loss_avg_fraction", test_case_name_,
                                  loss_stats_.GetAverage(),
                                  webrtc::test::Unit::kUnitless,
                                  webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "packet_loss"}});
    logger_->LogSingleValueMetric("packet_loss_std_fraction", test_case_name_,
                                  loss_stats_.GetStandardDeviation(),
                                  webrtc::test::Unit::kUnitless,
                                  webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "std_dev"}, {"metric", "packet_loss"}});
  }
  // Periodically export all metrics to JSON
  ExportToJsonFile("gcc_test_c2.json");
}


GCCMetricsCollector::~GCCMetricsCollector() {
  RTC_LOG(LS_INFO) << "Test complete. Safely exporting metrics to JSON...";
  ExportToJsonFile("gcc_test_c2.json");
}

void GCCMetricsCollector::UpdateAckedRateStats(DataRate acked_rate) {
  acked_rate_stats_.AddSample(acked_rate.bps());
}

void GCCMetricsCollector::UpdateDelayStats(TimeDelta rtt,
                                           TimeDelta one_way_delay) {
  if (rtt.IsFinite()) {
    rtt_stats_.AddSample(rtt.ms());
  }
  if (one_way_delay.IsFinite()) {
    delay_stats_.AddSample(one_way_delay.ms());
  }
}

void GCCMetricsCollector::UpdateLossStats(double loss_fraction) {
  loss_stats_.AddSample(loss_fraction);
}

// GCCNetworkController metrics helper methods implementation
void GoogCcNetworkController::LogPeriodicMetrics(Timestamp at_time) {
  if (!metrics_enabled_ || !metrics_collector_) {
    return;
  }
  
  if (at_time - metrics_last_logged_ < kMetricsLoggingInterval) {
    return;
  }
  metrics_last_logged_ = at_time;
  
  // Pull the Kalman-filtered acked rate
  DataRate acknowledged_rate =
      acknowledged_bitrate_estimator_->bitrate().value_or(DataRate::Zero());

  // Log all 4 bandwidth metrics simultaneously
  metrics_collector_->LogBandwidthMetrics(at_time, 
                                          last_target_rate_,     // 1. Target
                                          last_actual_bitrate_,  // 2. Actual
                                          acknowledged_rate,     // 3. Acked
                                          last_send_rate_);      // 4. Send
  
  // Log delay metrics
  if (last_rtt_.IsFinite()) {
    metrics_collector_->LogDelayMetrics(at_time, last_rtt_, last_rtt_ / 2, jitter_); 
  }
  
  // Log loss metrics
  metrics_collector_->LogLossMetrics(at_time, last_loss_fraction_, last_packets_lost_); 

  // Log periodic summary
  metrics_collector_->LogPeriodicSummary(at_time);
}



void GCCMetricsCollector::ExportToJsonFile(const std::string& filename) {
  if (!logger_) {
    return;
  }

  auto metrics = logger_->GetCollectedMetrics();

  // Write to a temporary file first.
  std::string temp_filename = filename + ".tmp";

  FILE* f = fopen(temp_filename.c_str(), "w");
  if (!f) {
    RTC_LOG(LS_ERROR)
        << "Failed to open temporary metrics file: "
        << temp_filename;
    return;
  }

  fprintf(f, "[\n");

  for (size_t i = 0; i < metrics.size(); ++i) {
    const auto& m = metrics[i];

    fprintf(f, "  {\n");
    fprintf(f, "    \"name\": \"%s\",\n", m.name.c_str());
    fprintf(f, "    \"samples\": [");

    for (size_t j = 0; j < m.time_series.samples.size(); ++j) {
      const auto& s = m.time_series.samples[j];

      fprintf(
          f,
          "%s{\"timestamp_ms\": %lld, \"value\": %f}",
          (j > 0 ? ", " : ""),
          static_cast<long long>(s.timestamp.ms()),
          s.value);
    }

    fprintf(
        f,
        "]\n"
        "  }%s\n",
        (i + 1 < metrics.size()) ? "," : "");
  }

  fprintf(f, "]\n");

  // Flush stdio buffers.
  if (fflush(f) != 0) {
    RTC_LOG(LS_ERROR)
        << "fflush failed for temporary metrics file: "
        << temp_filename;
    fclose(f);
    return;
  }

  // Flush kernel buffers to disk.
  if (fsync(fileno(f)) != 0) {
    RTC_LOG(LS_ERROR)
        << "fsync failed for temporary metrics file: "
        << temp_filename;
    fclose(f);
    return;
  }

  // Close the file.
  if (fclose(f) != 0) {
    RTC_LOG(LS_ERROR)
        << "Failed to close temporary metrics file: "
        << temp_filename;
    return;
  }

  // Atomically replace the old file with the new one.
  if (rename(temp_filename.c_str(), filename.c_str()) != 0) {
    RTC_LOG(LS_ERROR)
        << "Failed to rename "
        << temp_filename
        << " -> "
        << filename;

    remove(temp_filename.c_str());
    return;
  }

  // Heartbeat logging.
  static int export_call_count = 0;
  ++export_call_count;

  if (export_call_count % 100 == 0) {
    RTC_LOG(LS_INFO)
        << "[Metrics Heartbeat] JSON export completed successfully "
        << export_call_count
        << " times. Safely written to "
        << filename;
  }
}



}  // namespace webrtc
