#ifndef MODULES_CONGESTION_CONTROLLER_GOOG_CC_ECN_BASED_BWE_H_
#define MODULES_CONGESTION_CONTROLLER_GOOG_CC_ECN_BASED_BWE_H_

#include <cstdint>
#include <optional>
#include "api/units/data_rate.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/congestion_controller/rtp/transport_feedback_adapter.h"

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
      std::optional<DataRate> acknowledged_bitrate,
      std::optional<DataRate> probe_bitrate,
      std::optional<NetworkStateEstimate> estimate, bool in_alr);
 private:
  DataRate min_bitrate_ = DataRate::KilobitsPerSec(20);
  DataRate max_bitrate_ = DataRate::PlusInfinity();
  DataRate current_target_rate_ = DataRate::KilobitsPerSec(300);
  double alpha_ = 0.0;
  Timestamp last_md_time_ = Timestamp::MinusInfinity();
  Timestamp window_start_ = Timestamp::MinusInfinity();
  TimeDelta current_rtt_ = TimeDelta::Millis(50);
  TimeDelta window_duration_ = TimeDelta::Millis(50);
  int64_t window_total_ = 0;
  int64_t window_ce_ = 0;
};
}  // namespace webrtc
#endif
