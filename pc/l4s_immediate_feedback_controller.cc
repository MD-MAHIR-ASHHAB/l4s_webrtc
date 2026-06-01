/*
 *  Copyright (c) 2024 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "pc/l4s_immediate_feedback_controller.h"

#include "modules/remote_bitrate_estimator/congestion_control_feedback_generator.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

namespace webrtc {

L4sImmediateFeedbackController::L4sImmediateFeedbackController(
    CongestionControlFeedbackGenerator* feedback_generator)
    : feedback_generator_(feedback_generator) {
  RTC_DCHECK(feedback_generator_);
  RTC_LOG(LS_INFO) << "L4S: Immediate Feedback Controller initialized";
}

void L4sImmediateFeedbackController::OnCePacketReceived(
    Timestamp timestamp, uint32_t ssrc, uint16_t sequence_number) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  
  RTC_LOG(LS_INFO) << "L4S: CE packet received - SSRC=" << ssrc 
                   << " seq=" << sequence_number 
                   << " mode=" << (current_mode_ == FeedbackMode::kBatchMode ? "BATCH" : "IMMEDIATE");

  if (current_mode_ == FeedbackMode::kBatchMode) {
    // First CE packet: flush current batch and switch to immediate mode
    RTC_LOG(LS_INFO) << "L4S: Switching from BATCH to IMMEDIATE mode - flushing accumulated batch";
    FlushBatchAndSwitchToImmediate();
    ++batch_flushes_;
  } else {
    // Already in immediate mode: send immediate feedback for this packet
    RTC_LOG(LS_INFO) << "L4S: Sending immediate feedback for CE packet in IMMEDIATE mode";
    SendImmediateFeedbackForPacket();
    ++ce_packets_in_immediate_mode_;
  }

  last_mode_switch_ = timestamp;
}

void L4sImmediateFeedbackController::OnNonCePacketReceived(
    Timestamp timestamp, uint32_t ssrc, uint16_t sequence_number) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);

  if (current_mode_ == FeedbackMode::kImmediateMode) {
    RTC_LOG(LS_INFO) << "L4S: Non-CE packet received - switching back to BATCH mode"
                     << " (handled " << ce_packets_in_immediate_mode_ << " CE packets in immediate mode)";
    SwitchToBatchMode();
    last_mode_switch_ = timestamp;
  }
  
  // In batch mode, we don't need to do anything special for non-CE packets
  // The normal batching mechanism will handle them
}

void L4sImmediateFeedbackController::FlushBatchAndSwitchToImmediate() {
  RTC_DCHECK_RUN_ON(&sequence_checker_);

  // Immediate RTCP is disabled here; CE packets only update controller state
  // and periodic RFC 8888 feedback will carry the congestion signal.
  current_mode_ = FeedbackMode::kImmediateMode;
  ce_packets_in_immediate_mode_ = 1;
}

void L4sImmediateFeedbackController::SendImmediateFeedbackForPacket() {
  RTC_DCHECK_RUN_ON(&sequence_checker_);

  RTC_LOG(LS_VERBOSE)
      << "L4S: Immediate RTCP disabled; waiting for periodic RFC 8888 feedback";
}

void L4sImmediateFeedbackController::SwitchToBatchMode() {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  
  current_mode_ = FeedbackMode::kBatchMode;
  ce_packets_in_immediate_mode_ = 0;
  
  RTC_LOG(LS_INFO) << "L4S: Switched to BATCH mode after " << batch_flushes_ << " batch flushes";
}

}  // namespace webrtc
