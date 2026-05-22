/*
 *  Copyright (c) 2026 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/goog_cc/ecn_based_bwe.h"

#include <algorithm>

#include "rtc_base/logging.h"

namespace webrtc {

void EcnBasedBwe::SetMinMaxBitrate(DataRate min_bitrate, DataRate max_bitrate) {
  if (min_bitrate.IsFinite()) {
    min_bitrate_ = min_bitrate;
  }
  if (max_bitrate.IsFinite()) {
    max_bitrate_ = std::max(min_bitrate_, max_bitrate);
  }
}

void EcnBasedBwe::Reset() {
  ecn_limited_bandwidth_ = DataRate::PlusInfinity();
  last_ce_feedback_time_ = Timestamp::MinusInfinity();
  ce_fraction_ewma_ = 0.0;
  update_count_ = 0;
}

void EcnBasedBwe::UpdateBandwidthEstimate(const TransportPacketsFeedback& report,
                                          DataRate delay_based_estimate,
                                          bool /* in_alr */) {
  if (report.packet_feedbacks.empty() || !delay_based_estimate.IsFinite()) {
    return;
  }

  int ect_count = 0;
  int ce_count = 0;
  for (const PacketResult& packet : report.packet_feedbacks) {
    if (packet.ecn == EcnMarking::kCe) {
      ++ce_count;
      ++ect_count;
    } else if (packet.ecn == EcnMarking::kEct0 ||
               packet.ecn == EcnMarking::kEct1) {
      ++ect_count;
    }
  }

  ++update_count_;
  if (ce_count == 0) {
    return;
  }

  const double ecn_fraction =
      ect_count > 0 ? static_cast<double>(ce_count) / ect_count : 1.0;
  ce_fraction_ewma_ = 0.8 * ce_fraction_ewma_ + 0.2 * ecn_fraction;

  double backoff_fraction = std::clamp(0.5 * ce_fraction_ewma_, 0.05, 0.5);
  DataRate candidate = delay_based_estimate * (1.0 - backoff_fraction);
  candidate = std::max(candidate, min_bitrate_);
  if (max_bitrate_.IsFinite()) {
    candidate = std::min(candidate, max_bitrate_);
  }

  if (!ecn_limited_bandwidth_.IsFinite()) {
    ecn_limited_bandwidth_ = candidate;
  } else {
    ecn_limited_bandwidth_ = std::min(ecn_limited_bandwidth_, candidate);
  }
  last_ce_feedback_time_ = report.feedback_time;
}

std::optional<DataRate> EcnBasedBwe::GetEcnLimitedBandwidth(
    Timestamp at_time) const {
  if (!last_ce_feedback_time_.IsFinite()) {
    return std::nullopt;
  }
  if (at_time - last_ce_feedback_time_ > kEcnHoldDuration) {
    return std::nullopt;
  }
  if (!ecn_limited_bandwidth_.IsFinite()) {
    return std::nullopt;
  }
  return ecn_limited_bandwidth_;
}

}  // namespace webrtc
