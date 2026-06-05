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

  last_update_time_ = now;

  // --- 1. Calculate CE Ratio ---
  double ce_ratio = 0.0;
  if (!report.packet_feedbacks.empty()) {
    ce_ratio = static_cast<double>(report.ce_count) / report.packet_feedbacks.size();
  }

  // --- 2. Rate Control State Machine ---
  bool executed_md = false;

  // Enforce the 5% Tolerance Threshold (0.05)
  if (ce_ratio >= 0.05) {
    // Multiplicative Decrease (MD)
    TimeDelta pipeline_delay = std::max(current_rtt_, TimeDelta::Millis(50));

    if (last_md_time_.IsInfinite() || (now - last_md_time_ >= pipeline_delay)) {
      
      // Calculate Proportional Reduction: Target = Target * (1 - 0.5 * ce_ratio)
      double reduction_factor = 1.0 - (0.5 * ce_ratio);
      
      // Enforce a hard floor of 0.5 (maximum 50% cut per RTT) to prevent bottoming out
      reduction_factor = std::max(0.5, reduction_factor);

      // Apply the cut to the synchronized overarching target
      current_target_rate_ = current_target_rate_ * reduction_factor;

      last_md_time_ = now;
      executed_md = true;

      RTC_LOG(LS_INFO) << "[ECN BWE] Executed Cut. CE Ratio: " << ce_ratio 
                       << " | Factor: " << reduction_factor 
                       << " | New target: " << current_target_rate_.kbps() << " kbps";
    }
  }

  // --- 3. Enforce Bounds and Output ---
  current_target_rate_ = std::clamp(current_target_rate_, min_bitrate_, max_bitrate_);

  // Only report an update when we executed a multiplicative decrease. When
  // no CE marks (or < 5%) are seen, do not override GCC's adaptive increase logic.
  result.updated = executed_md;
  if (result.updated) {
    result.target_bitrate = current_target_rate_;
    result.recovered_from_overuse = false;
  }

  return result;
}




}  // namespace webrtc