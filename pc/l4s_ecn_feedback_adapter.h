/*
 * Copyright 2024 The WebRTC Project Authors. All rights reserved.
 *
 * Use of this source code is governed by a BSD-style license
 * that can be found in the LICENSE file in the root of the source
 * tree. An additional intellectual property rights grant can be found
 * in the file PATENTS.  All contributing project authors may
 * be found in the AUTHORS file in the root of the source tree.
 */

#ifndef PC_L4S_ECN_FEEDBACK_ADAPTER_H_
#define PC_L4S_ECN_FEEDBACK_ADAPTER_H_

#include "pc/rtp_transport.h"
#include "pc/l4s_immediate_feedback_controller.h"
#include "modules/congestion_controller/include/receive_side_congestion_controller.h"
#include "api/sequence_checker.h"
#include "api/task_queue/task_queue_base.h"
#include "api/units/timestamp.h"
#include "rtc_base/synchronization/mutex.h"

namespace webrtc {

// L4S ECN Feedback Adapter
// 
// This class acts as a bridge between RtpTransport's ECN detection 
// and the L4S immediate feedback controller. When CE-marked packets
// are detected in the transport layer, this adapter triggers the
// L4S state machine for hybrid batch/immediate feedback control.
//
// Thread Safety:
// - Callbacks from RtpTransport (OnCongestionMarkingReceived) occur on 
//   the network thread
// - L4S controller and feedback generator methods must be called on the 
//   correct sequence checker thread
// - This adapter handles the thread transitions safely
class L4sEcnFeedbackAdapter : public EcnFeedbackObserver {
 public:
  explicit L4sEcnFeedbackAdapter(
      std::unique_ptr<L4sImmediateFeedbackController> l4s_controller,
      TaskQueueBase* task_queue);
  ~L4sEcnFeedbackAdapter() override;

  // EcnFeedbackObserver implementation
  void OnCongestionMarkingReceived(Timestamp timestamp,
                                   uint32_t ssrc, 
                                   uint16_t sequence_number) override;

  // EcnFeedbackObserver implementation for non-CE packets
  void OnNonCePacketReceived(Timestamp timestamp,
                             uint32_t ssrc, 
                             uint16_t sequence_number) override;

  // Called when the L4S controller is being destroyed
  void Reset();

 private:
  // Thread-safe L4S controller ownership
  mutable Mutex mutex_;
  std::unique_ptr<L4sImmediateFeedbackController> l4s_controller_ RTC_GUARDED_BY(mutex_);
  TaskQueueBase* task_queue_ RTC_GUARDED_BY(mutex_);
  
  // Statistics
  uint64_t ce_packets_detected_ RTC_GUARDED_BY(mutex_) = 0;
  uint64_t non_ce_packets_detected_ RTC_GUARDED_BY(mutex_) = 0;
  uint64_t l4s_callbacks_sent_ RTC_GUARDED_BY(mutex_) = 0;
};

}  // namespace webrtc

#endif  // PC_L4S_ECN_FEEDBACK_ADAPTER_H_
