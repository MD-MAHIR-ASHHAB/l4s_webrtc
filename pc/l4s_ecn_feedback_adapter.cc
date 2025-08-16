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
    ReceiveSideCongestionController* congestion_controller,
    TaskQueueBase* task_queue)
    : congestion_controller_(congestion_controller), task_queue_(task_queue) {
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
  
  if (congestion_controller_ && task_queue_) {
    // Post the immediate feedback call to the congestion controller's task queue
    // This ensures the call happens on the correct thread (sequence checker)
    ++immediate_feedback_sent_;
    
    // Capture needed variables for the async call
    ReceiveSideCongestionController* controller = congestion_controller_;
    uint64_t feedback_count = immediate_feedback_sent_;
    
    task_queue_->PostTask([controller, feedback_count]() {
      RTC_LOG(LS_INFO) << "L4S: Triggering immediate feedback "
                       << "(feedback #" << feedback_count << ")";
      controller->SendImmediateCongestionFeedback();
    });
    
    RTC_LOG(LS_INFO) << "L4S: Immediate feedback posted to task queue "
                     << "(total immediate feedbacks: " << immediate_feedback_sent_ << ")";
  } else {
    RTC_LOG(LS_WARNING) << "L4S: CE detected but congestion controller or task queue unavailable";
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
  task_queue_ = nullptr;
}

}  // namespace webrtc
