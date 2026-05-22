/*
 * Copyright (c) 2026 The WebRTC project authors. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license
 * that can be found in the LICENSE file in the root of the source
 * tree. An additional intellectual property rights grant can be found
 * in the file PATENTS.  All contributing project authors may
 * be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_CONGESTION_CONTROLLER_GOOG_CC_ECN_BASED_BWE_H_
#define MODULES_CONGESTION_CONTROLLER_GOOG_CC_ECN_BASED_BWE_H_

#include <optional>

#include "api/units/data_rate.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/congestion_controller/rtp/transport_feedback_adapter.h" // Needed for TransportPacketsFeedback

namespace webrtc {

class EcnBasedBwe {
 public:
  struct ECNResult {
    bool updated = false;
    DataRate target_bitrate = DataRate::Zero();
    bool recovered_from_overuse = false;
  };

  EcnBasedBwe();
  ~EcnBasedBwe() = default;

  void SetMinMaxBitrate(DataRate min_bitrate, DataRate max_bitrate);
  void SetTargetBitrate(DataRate starting_rate);
  void UpdateRtt(TimeDelta rtt);

  ECNResult IncomingPacketFeedbackVector(
      const TransportPacketsFeedback& report,
      std::optional<DataRate> acknowledged_bitrate);

 private:
  DataRate min_bitrate_ = DataRate::KilobitsPerSec(20);
  DataRate max_bitrate_ = DataRate::PlusInfinity();
  
  // The sovereign state of the ECN controller
  DataRate current_target_rate_ = DataRate::KilobitsPerSec(300); 
  double alpha_ = 0.0;
  
  Timestamp last_md_time_ = Timestamp::MinusInfinity();
  Timestamp last_update_time_ = Timestamp::MinusInfinity();
  TimeDelta current_rtt_ = TimeDelta::Millis(50);
};

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_GOOG_CC_ECN_BASED_BWE_H_