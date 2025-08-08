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
#include "modules/congestion_controller/include/receive_side_congestion_controller.h"
#include "rtc_base/sequence_checker.h"
#include "rtc_base/synchronization/mutex.h"

namespace webrtc {

// L4S ECN Feedback Adapter
// 
// This class acts as a bridge between RtpTransport's ECN detection 
// and the congestion control feedback system. When CE-marked packets
// are detected in the transport layer, this adapter triggers immediate
// RTCP feedback according to L4S requirements.
//
// Thread Safety:
// - Callbacks from RtpTransport (OnCongestionMarkingReceived) occur on 
//   the network thread
// - ReceiveSideCongestionController methods must be called on the 
//   sequence checker thread
// - This adapter handles the thread transitions safely
class L4sEcnFeedbackAdapter : public RtpTransport::EcnFeedbackObserver {
 public:
  explicit L4sEcnFeedbackAdapter(
      ReceiveSideCongestionController* congestion_controller);
  ~L4sEcnFeedbackAdapter() override;

  // RtpTransport::EcnFeedbackObserver implementation
  void OnCongestionMarkingReceived(uint32_t ssrc, 
                                   uint16_t sequence_number) override;

  // Called when congestion controller is being destroyed
  void Reset();

 private:
  // Thread-safe pointer to congestion controller
  mutable Mutex mutex_;
  ReceiveSideCongestionController* congestion_controller_ RTC_GUARDED_BY(mutex_);
  
  // Statistics
  uint64_t ce_packets_detected_ RTC_GUARDED_BY(mutex_) = 0;
  uint64_t immediate_feedback_sent_ RTC_GUARDED_BY(mutex_) = 0;
};

}  // namespace webrtc

#endif  // PC_L4S_ECN_FEEDBACK_ADAPTER_H_
