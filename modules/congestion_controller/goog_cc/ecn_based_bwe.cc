/*
 * Copyright (c) 2026 The WebRTC project authors. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license
 * that can be found in the LICENSE file in the root of the source
 * tree. An additional intellectual property rights grant can be found
 * in the file PATENTS.  All contributing project authors may
 * be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/goog_cc/ecn_based_bwe.h"

#include <algorithm>

#include "rtc_base/logging.h"

namespace webrtc {

EcnBasedBwe::EcnBasedBwe() = default;

void EcnBasedBwe::SetMinMaxBitrate(DataRate min_bitrate, DataRate max_bitrate) {
  if (min_bitrate.IsFinite()) {
    min_bitrate_ = min_bitrate;
  }
  if (max_bitrate.IsFinite()) {
    max_bitrate_ = std::max(min_bitrate_, max_bitrate);
  }
  current_target_rate_ = std::clamp(current_target_rate_, min_bitrate_, max_bitrate_);
}

void EcnBasedBwe::SetTargetBitrate(DataRate starting_rate) {
  if (starting_rate.IsFinite() && starting_rate > DataRate::Zero()) {
    current_target_rate_ = std::clamp(starting_rate, min_bitrate_, max_bitrate_);
  }
}

void EcnBasedBwe::UpdateRtt(TimeDelta rtt) {
  if (rtt.IsFinite() && rtt > TimeDelta::Zero()) {
    current_rtt_ = rtt;
  }
}

EcnBasedBwe::ECNResult EcnBasedBwe::IncomingPacketFeedbackVector(
    const TransportPacketsFeedback& report,
    std::optional<DataRate> acknowledged_bitrate,
    std::optional<DataRate> probe_bitrate,
    std::optional<NetworkStateEstimate> estimate,
    bool in_alr) {
    
  ECNResult result;
  
  if (report.packet_feedbacks.empty()) {
    return result;
  }

  Timestamp now = report.feedback_time;
  if (last_update_time_.IsInfinite()) {
    last_update_time_ = now;
    return result;
  }

  // TimeDelta delta_time = now - last_update_time_;
  last_update_time_ = now;

  // --- 1. DCTCP Alpha Calculation ---
  int total_ecn_packets = report.ect_count + report.ce_count;
  double raw_ce_ratio = 0.0;
  
  if (total_ecn_packets > 0) {
    raw_ce_ratio = static_cast<double>(report.ce_count) / total_ecn_packets;
  }

  // Classic DCTCP gain factor (g = 1/16)
  constexpr double g = 1.0 / 16.0; 
  alpha_ = (1.0 - g) * alpha_ + g * raw_ce_ratio;

  // --- 2. Rate Control State Machine ---
  // Only execute Multiplicative Decrease (MD) on CE marks. Do not perform
  // additive increases here — let the existing GCC AIMD/rate-control logic
  // handle increases so AI remains adaptive and consistent with delay/loss.
  bool executed_md = false;

  if (report.ce_count > 0) {
    // Multiplicative Decrease (MD)
    // Enforce 1-RTT pipeline delay to prevent multiple cuts for the same congestion event
    TimeDelta pipeline_delay = std::max(current_rtt_, TimeDelta::Millis(50));

    if (last_md_time_.IsInfinite() || (now - last_md_time_ >= pipeline_delay)) {
      double reduction_factor = 1.0 - (alpha_ / 2.0);
      current_target_rate_ = current_target_rate_ * reduction_factor;

      last_md_time_ = now;
      executed_md = true;

      RTC_LOG(LS_INFO) << "[ECN BWE] Executed Cut. alpha: " << alpha_
                       << ", raw_ce: " << raw_ce_ratio
                       << ", new target: " << current_target_rate_.kbps()
                       << " kbps";
    }
  }

  // --- 3. Enforce Bounds and Output ---
  current_target_rate_ = std::clamp(current_target_rate_, min_bitrate_, max_bitrate_);

  // Only report an update when we executed a multiplicative decrease. When
  // no CE marks are seen, do not override GCC's adaptive increase logic.
  result.updated = executed_md;
  if (result.updated) {
    result.target_bitrate = current_target_rate_;
    result.recovered_from_overuse = false;
  }

  return result;
}

}  // namespace webrtc