/*
 *  Copyright (c) 2024 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef PC_L4S_IMMEDIATE_FEEDBACK_CONTROLLER_H_
#define PC_L4S_IMMEDIATE_FEEDBACK_CONTROLLER_H_

#include <memory>

#include "api/sequence_checker.h"
#include "api/units/timestamp.h"
#include "rtc_base/logging.h"
#include "rtc_base/thread_annotations.h"

namespace webrtc {

class CongestionControlFeedbackGenerator;

// L4S Immediate Feedback Controller
// 
// Implements the hybrid feedback strategy for L4S:
// - Normal mode: Standard batch-based feedback timing
// - Immediate mode: CE-triggered immediate feedback
//
// State Transitions:
// BATCH_MODE -> IMMEDIATE_MODE: On first CE packet (flush current batch)
// IMMEDIATE_MODE -> IMMEDIATE_MODE: On subsequent CE packets (immediate single packet)
// IMMEDIATE_MODE -> BATCH_MODE: On non-CE packet arrival
class L4sImmediateFeedbackController {
 public:
  enum class FeedbackMode {
    kBatchMode,     // Normal batching behavior
    kImmediateMode  // Immediate feedback for each CE packet
  };

  explicit L4sImmediateFeedbackController(
      CongestionControlFeedbackGenerator* feedback_generator);
  ~L4sImmediateFeedbackController() = default;

  // Called when a CE-marked packet is received
  // timestamp: arrival time of the CE packet
  // ssrc: SSRC of the stream
  // sequence_number: RTP sequence number
  void OnCePacketReceived(Timestamp timestamp, uint32_t ssrc, uint16_t sequence_number);

  // Called when a non-CE packet is received  
  // Used to transition back to batch mode
  void OnNonCePacketReceived(Timestamp timestamp, uint32_t ssrc, uint16_t sequence_number);

  // Get current feedback mode
  FeedbackMode GetCurrentMode() const RTC_LOCKS_EXCLUDED(sequence_checker_) { 
    RTC_DCHECK_RUN_ON(&sequence_checker_);
    return current_mode_; 
  }

  // For testing: force mode change
  void SetModeForTesting(FeedbackMode mode) RTC_LOCKS_EXCLUDED(sequence_checker_) { 
    RTC_DCHECK_RUN_ON(&sequence_checker_);
    current_mode_ = mode; 
  }

 private:
  // Flush current batch and switch to immediate mode
  void FlushBatchAndSwitchToImmediate() RTC_NO_THREAD_SAFETY_ANALYSIS;

  // Send immediate feedback for single packet
  void SendImmediateFeedbackForPacket() RTC_NO_THREAD_SAFETY_ANALYSIS;

  // Switch back to batch mode
  void SwitchToBatchMode();

  SequenceChecker sequence_checker_;
  CongestionControlFeedbackGenerator* const feedback_generator_ RTC_GUARDED_BY(sequence_checker_);
  
  FeedbackMode current_mode_ RTC_GUARDED_BY(sequence_checker_) = FeedbackMode::kBatchMode;
  
  // Statistics
  int ce_packets_in_immediate_mode_ RTC_GUARDED_BY(sequence_checker_) = 0;
  int batch_flushes_ RTC_GUARDED_BY(sequence_checker_) = 0;
  Timestamp last_mode_switch_ RTC_GUARDED_BY(sequence_checker_) = Timestamp::MinusInfinity();
};

}  // namespace webrtc

#endif  // PC_L4S_IMMEDIATE_FEEDBACK_CONTROLLER_H_
