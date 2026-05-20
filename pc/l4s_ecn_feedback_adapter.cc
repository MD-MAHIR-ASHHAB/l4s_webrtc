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

#include "pc/l4s_immediate_feedback_controller.h"
#include "rtc_base/logging.h"
#include "rtc_base/synchronization/mutex.h"
#include "rtc_base/thread_annotations.h"

namespace webrtc {

L4sEcnFeedbackAdapter::L4sEcnFeedbackAdapter(
    std::unique_ptr<L4sImmediateFeedbackController> l4s_controller,
    TaskQueueBase* task_queue)
    : l4s_controller_(std::move(l4s_controller)), task_queue_(task_queue) {
  RTC_LOG(LS_INFO) << "L4S: ECN feedback adapter created with L4S controller";
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
  
  if (l4s_controller_ && task_queue_) {
    // Post the L4S CE callback to the correct task queue
    // This ensures the call happens on the L4S controller's sequence checker thread
    ++l4s_callbacks_sent_;
    
    // Capture needed variables for the async call
    L4sImmediateFeedbackController* controller = l4s_controller_.get();
    uint64_t callback_count = l4s_callbacks_sent_;
    
    task_queue_->PostTask([controller, timestamp, ssrc, sequence_number, callback_count]() {
      RTC_LOG(LS_INFO) << "L4S: Processing CE packet in L4S controller "
                       << "(callback #" << callback_count << ")";
      controller->OnCePacketReceived(timestamp, ssrc, sequence_number);
    });
    
    RTC_LOG(LS_INFO) << "L4S: CE callback posted to task queue "
                     << "(total L4S callbacks: " << l4s_callbacks_sent_ << ")";
  } else {
    RTC_LOG(LS_WARNING) << "L4S: CE detected but L4S controller or task queue unavailable";
  }
}

void L4sEcnFeedbackAdapter::OnNonCePacketReceived(Timestamp timestamp,
                                                  uint32_t ssrc, 
                                                  uint16_t sequence_number) {
  MutexLock lock(&mutex_);
  
  ++non_ce_packets_detected_;
  
  if (l4s_controller_ && task_queue_) {
    // Post the L4S non-CE callback to the correct task queue
    ++l4s_callbacks_sent_;
    
    // Capture needed variables for the async call
    L4sImmediateFeedbackController* controller = l4s_controller_.get();
    
    task_queue_->PostTask([controller, timestamp, ssrc, sequence_number]() {
      controller->OnNonCePacketReceived(timestamp, ssrc, sequence_number);
    });
  }
}

void L4sEcnFeedbackAdapter::Reset() {
  MutexLock lock(&mutex_);
  
  if (l4s_controller_) {
    RTC_LOG(LS_INFO) << "L4S: Resetting ECN feedback adapter - "
                     << "CE packets: " << ce_packets_detected_ 
                     << ", non-CE packets: " << non_ce_packets_detected_
                     << ", L4S callbacks: " << l4s_callbacks_sent_;
  }
  
  l4s_controller_.reset();
  task_queue_ = nullptr;
}

}  // namespace webrtc
