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
    // First CE packet: flush current batch, switch to immediate mode, and start accumulating CE packets
    RTC_LOG(LS_INFO) << "L4S: RFC 8888 COMPLIANT - Switching from BATCH to IMMEDIATE mode. "
                     << "Flushing accumulated batch, then collecting CE packets for batched RTCP.";
    FlushBatchAndSwitchToImmediate();
    ++batch_flushes_;
    
    // Start accumulating CE packets
    first_ce_sequence_ = sequence_number;
    ce_packet_count_ = 1;
    
    RTC_LOG(LS_INFO) << "L4S: RFC 8888 - Started CE accumulation: seq=" << first_ce_sequence_ 
                     << ", count=" << ce_packet_count_;
  } else {
    // Already in immediate mode: accumulate this CE packet (don't send yet!)
    if (ce_packet_count_ == 0) {
      first_ce_sequence_ = sequence_number;
    }
    ce_packet_count_++;
    
    RTC_LOG(LS_VERBOSE) << "L4S: RFC 8888 - Accumulated CE packet: seq=" << sequence_number 
                        << ", total_count=" << ce_packet_count_;
  }

  last_mode_switch_ = timestamp;
}

void L4sImmediateFeedbackController::OnNonCePacketReceived(
    Timestamp timestamp, uint32_t ssrc, uint16_t sequence_number) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);

  if (current_mode_ == FeedbackMode::kImmediateMode) {
    RTC_LOG(LS_INFO) << "L4S: RFC 8888 - Non-CE packet received. Sending batch of " 
                     << ce_packet_count_ << " accumulated CE packets and switching back to BATCH mode";
    SendCeBatchAndSwitchToBatch();
    last_mode_switch_ = timestamp;
  }
  
  // In batch mode, normal batching mechanism handles non-CE packets
}

void L4sImmediateFeedbackController::FlushBatchAndSwitchToImmediate() {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  
  // Send immediate feedback to flush any accumulated batch + current CE packet
  feedback_generator_->SendImmediateFeedback();
  
  // Switch to immediate mode
  current_mode_ = FeedbackMode::kImmediateMode;
  ce_packets_in_immediate_mode_ = 1; // Count the current CE packet
}

void L4sImmediateFeedbackController::SendCeBatchAndSwitchToBatch() {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  
  if (ce_packet_count_ > 0) {
    RTC_LOG(LS_INFO) << "L4S: RFC 8888 COMPLIANT - Sending ONE RTCP batch for " 
                     << ce_packet_count_ << " CE packets (seq " << first_ce_sequence_ 
                     << " + " << (ce_packet_count_ - 1) << " more)";
    // Send immediate feedback once with all accumulated CE packets
    feedback_generator_->SendImmediateFeedback();
    ++ce_batches_sent_;
  }
  
  ce_packet_count_ = 0;
  first_ce_sequence_ = 0;
  SwitchToBatchMode();
}

void L4sImmediateFeedbackController::SwitchToBatchMode() {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  
  current_mode_ = FeedbackMode::kBatchMode;
  ce_packets_in_immediate_mode_ = 0;
  
  RTC_LOG(LS_INFO) << "L4S: Switched to BATCH mode after " << batch_flushes_ << " batch flushes";
}

}  // namespace webrtc
