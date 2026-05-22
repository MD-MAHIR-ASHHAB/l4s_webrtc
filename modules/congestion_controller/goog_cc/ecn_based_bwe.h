/*
 *  Copyright (c) 2026 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_CONGESTION_CONTROLLER_GOOG_CC_ECN_BASED_BWE_H_
#define MODULES_CONGESTION_CONTROLLER_GOOG_CC_ECN_BASED_BWE_H_

#include <optional>

#include "api/transport/network_types.h"
#include "api/units/data_rate.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"

namespace webrtc {

class EcnBasedBwe {
 public:
  EcnBasedBwe() = default;
  ~EcnBasedBwe() = default;

  void SetMinMaxBitrate(DataRate min_bitrate, DataRate max_bitrate);
  void Reset();

  void UpdateBandwidthEstimate(const TransportPacketsFeedback& report,
                               DataRate delay_based_estimate,
                               bool in_alr);

  std::optional<DataRate> GetEcnLimitedBandwidth(Timestamp at_time) const;

 private:
  static constexpr TimeDelta kEcnHoldDuration = TimeDelta::Seconds(3);

  DataRate min_bitrate_ = DataRate::KilobitsPerSec(1);
  DataRate max_bitrate_ = DataRate::PlusInfinity();
  DataRate ecn_limited_bandwidth_ = DataRate::PlusInfinity();
  Timestamp last_ce_feedback_time_ = Timestamp::MinusInfinity();
  double ce_fraction_ewma_ = 0.0;
  int update_count_ = 0;
};

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_GOOG_CC_ECN_BASED_BWE_H_
