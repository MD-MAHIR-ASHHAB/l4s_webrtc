/*
 * Copyright 2024 The WebRTC Project Authors. All rights reserved.
 *
 * Use of this source code is governed by a BSD-style license
 * that can be found in the LICENSE file in the root of the source
 * tree. An additional intellectual property rights grant can be found
 * in the file PATENTS.  All contributing project authors may
 * be found in the AUTHORS file in the root of the source tree.
 */

#include "pc/l4s_ecn_feedback_adapter.h"

#include "rtc_base/logging.h"
#include "rtc_base/synchronization/mutex.h"

namespace webrtc {

L4sEcnFeedbackAdapter::L4sEcnFeedbackAdapter(
    ReceiveSideCongestionController* congestion_controller)
    : congestion_controller_(congestion_controller) {
  RTC_LOG(LS_INFO) << "L4S: ECN feedback adapter created";
}

L4sEcnFeedbackAdapter::~L4sEcnFeedbackAdapter() {
  Reset();
  RTC_LOG(LS_INFO) << "L4S: ECN feedback adapter destroyed";
}

void L4sEcnFeedbackAdapter::OnCongestionMarkingReceived(Timestamp timestamp,
                                                        uint32_t ssrc, 
                                                        uint16_t sequence_number) {
  MutexLock lock(&mutex_);
  
  ++ce_packets_detected_;
  
  RTC_LOG(LS_INFO) << "L4S: CE marking detected on SSRC " << ssrc 
                   << ", sequence " << sequence_number 
                   << ", timestamp " << timestamp.us()
                   << " (total CE packets: " << ce_packets_detected_ << ")";
  
  if (congestion_controller_) {
    // Trigger immediate RTCP feedback for L4S
    // Note: This call must be made on the correct sequence checker thread
    // The ReceiveSideCongestionController will handle the thread safety
    congestion_controller_->SendImmediateCongestionFeedback();
    ++immediate_feedback_sent_;
    
    RTC_LOG(LS_INFO) << "L4S: Immediate feedback triggered "
                     << "(total immediate feedbacks: " << immediate_feedback_sent_ << ")";
  } else {
    RTC_LOG(LS_WARNING) << "L4S: CE detected but congestion controller unavailable";
  }
}

void L4sEcnFeedbackAdapter::Reset() {
  MutexLock lock(&mutex_);
  
  if (congestion_controller_) {
    RTC_LOG(LS_INFO) << "L4S: Resetting ECN feedback adapter - "
                     << "CE packets: " << ce_packets_detected_ 
                     << ", immediate feedbacks: " << immediate_feedback_sent_;
  }
  
  congestion_controller_ = nullptr;
}

}  // namespace webrtc
