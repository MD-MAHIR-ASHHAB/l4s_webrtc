/*
 *  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/include/receive_side_congestion_controller.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>

#include "api/environment/environment.h"
#include "api/media_types.h"
#include "api/sequence_checker.h"
#include "api/units/data_rate.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/congestion_controller/remb_throttler.h"
#include "modules/remote_bitrate_estimator/congestion_control_feedback_generator.h"
#include "modules/remote_bitrate_estimator/remote_bitrate_estimator_abs_send_time.h"
#include "modules/remote_bitrate_estimator/remote_bitrate_estimator_single_stream.h"
#include "modules/remote_bitrate_estimator/transport_sequence_number_feedback_generator.h"
#include "modules/rtp_rtcp/source/rtp_header_extensions.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "rtc_base/experiments/field_trial_parser.h"
#include "rtc_base/logging.h"
#include "rtc_base/synchronization/mutex.h"
#include "rtc_base/thread_annotations.h"

namespace webrtc {

namespace {
static const uint32_t kTimeOffsetSwitchThreshold = 30;
}  // namespace

void ReceiveSideCongestionController::OnRttUpdate(int64_t avg_rtt_ms,
                                                  int64_t max_rtt_ms) {
  MutexLock lock(&mutex_);
  rbe_->OnRttUpdate(avg_rtt_ms, max_rtt_ms);
}

void ReceiveSideCongestionController::RemoveStream(uint32_t ssrc) {
  MutexLock lock(&mutex_);
  rbe_->RemoveStream(ssrc);
}

DataRate ReceiveSideCongestionController::LatestReceiveSideEstimate() const {
  MutexLock lock(&mutex_);
  return rbe_->LatestEstimate();
}

void ReceiveSideCongestionController::PickEstimator(
    bool has_absolute_send_time) {
  if (has_absolute_send_time) {
    // If we see AST in header, switch RBE strategy immediately.
    if (!using_absolute_send_time_) {
      RTC_LOG(LS_INFO)
          << "WrappingBitrateEstimator: Switching to absolute send time RBE.";
      using_absolute_send_time_ = true;
      rbe_ = std::make_unique<RemoteBitrateEstimatorAbsSendTime>(
          env_, &remb_throttler_);
    }
    packets_since_absolute_send_time_ = 0;
  } else {
    // When we don't see AST, wait for a few packets before going back to TOF.
    if (using_absolute_send_time_) {
      ++packets_since_absolute_send_time_;
      if (packets_since_absolute_send_time_ >= kTimeOffsetSwitchThreshold) {
        RTC_LOG(LS_INFO)
            << "WrappingBitrateEstimator: Switching to transmission "
               "time offset RBE.";
        using_absolute_send_time_ = false;
        rbe_ = std::make_unique<RemoteBitrateEstimatorSingleStream>(
            env_, &remb_throttler_);
      }
    }
  }
}

ReceiveSideCongestionController::ReceiveSideCongestionController(
    const Environment& env,
    TransportSequenceNumberFeedbackGenenerator::RtcpSender feedback_sender,
    RembThrottler::RembSender remb_sender)
    : env_(env),
      remb_throttler_(std::move(remb_sender), &env_.clock()),
      transport_sequence_number_feedback_generator_(feedback_sender),
      congestion_control_feedback_generator_(env, feedback_sender),
      rbe_(std::make_unique<RemoteBitrateEstimatorSingleStream>(
          env_,
          &remb_throttler_)),
      using_absolute_send_time_(false),
      packets_since_absolute_send_time_(0) {
  // Always enable RFC 8888 congestion control feedback by default
  EnableSendCongestionControlFeedbackAccordingToRfc8888();
}

void ReceiveSideCongestionController::
    EnableSendCongestionControlFeedbackAccordingToRfc8888() {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  send_rfc8888_congestion_feedback_ = true;
}

void ReceiveSideCongestionController::SendImmediateCongestionFeedback()
    RTC_NO_THREAD_SAFETY_ANALYSIS {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  
  if (send_rfc8888_congestion_feedback_) {
    RTC_LOG(LS_VERBOSE) << "L4S: Triggering immediate congestion control feedback";
    congestion_control_feedback_generator_.SendImmediateFeedback();
  } else {
    RTC_LOG(LS_WARNING) << "L4S: Immediate feedback requested but RFC 8888 feedback not enabled";
  }
}

CongestionControlFeedbackGenerator* 
ReceiveSideCongestionController::GetCongestionControlFeedbackGenerator()
    RTC_NO_THREAD_SAFETY_ANALYSIS {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  return &congestion_control_feedback_generator_;
}

void ReceiveSideCongestionController::OnReceivedPacket(
    const RtpPacketReceived& packet,
    MediaType media_type) {
  bool has_transport_sequence_number =
      packet.HasExtension<TransportSequenceNumber>() ||
      packet.HasExtension<TransportSequenceNumberV2>();
  
// RTC_LOG(LS_INFO) << "Received packet with media type: "
//                  << MediaTypeToString(media_type)
//                  << ", sequence number: "
//                  << (packet.sequence_number() < 0 ? "N/A" : std::to_string(packet.sequence_number()))
//                  << ", packet size: "
//                  << packet.size() << " bytes, SSRC: "
//                  << packet.Ssrc()
//                  << ", sequence number: "
//                  << packet.SequenceNumber()
//                  << ", arrival time: "
//                  << packet.arrival_time().us() << " us"
//                  << ", ECN marking: "
//                  << (packet.ecn() == EcnMarking::kNotEct ? "Not ECT" :
//                      packet.ecn() == EcnMarking::kEct0 ? "ECT(0)" :
//                      packet.ecn() == EcnMarking::kEct1 ? "ECT(1)" : "CE");


  // Only use RFC 8888 feedback
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  congestion_control_feedback_generator_.OnReceivedPacket(packet);
}

void ReceiveSideCongestionController::OnBitrateChanged(int bitrate_bps) {
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  DataRate send_bandwidth_estimate = DataRate::BitsPerSec(bitrate_bps);
  // transport_sequence_number_feedback_generator_.OnSendBandwidthEstimateChanged(send_bandwidth_estimate); // Disabled: RFC 8888 only
  congestion_control_feedback_generator_.OnSendBandwidthEstimateChanged(
      send_bandwidth_estimate);
}

TimeDelta ReceiveSideCongestionController::MaybeProcess() {
  Timestamp now = env_.clock().CurrentTime();
  // Only process RFC 8888 feedback
  RTC_DCHECK_RUN_ON(&sequence_checker_);
  TimeDelta time_until_cc_rep = congestion_control_feedback_generator_.Process(now);
  return std::max(time_until_cc_rep, TimeDelta::Zero());
}

void ReceiveSideCongestionController::SetMaxDesiredReceiveBitrate(
    DataRate bitrate) {
  remb_throttler_.SetMaxDesiredReceiveBitrate(bitrate);
}

}  // namespace webrtc
