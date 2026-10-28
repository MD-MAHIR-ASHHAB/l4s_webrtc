#include "modules/congestion_controller/goog_cc/ecn_based_bwe.h"
#include <algorithm>
#include "rtc_base/logging.h"

namespace webrtc {
EcnBasedBwe::EcnBasedBwe() = default;

void EcnBasedBwe::SetMinMaxBitrate(DataRate min_bitrate, DataRate max_bitrate) {
  if (min_bitrate.IsFinite()) min_bitrate_ = min_bitrate;
  if (max_bitrate.IsFinite())
    max_bitrate_ = std::max(min_bitrate_, max_bitrate);
  current_target_rate_ = std::clamp(current_target_rate_, min_bitrate_, max_bitrate_);
}
void EcnBasedBwe::SetTargetBitrate(DataRate rate) {
  if (rate.IsFinite() && rate > DataRate::Zero())
    current_target_rate_ = std::clamp(rate, min_bitrate_, max_bitrate_);
}
void EcnBasedBwe::UpdateRtt(TimeDelta rtt) {
  if (rtt.IsFinite() && rtt > TimeDelta::Zero()) current_rtt_ = rtt;
}
EcnBasedBwe::ECNResult EcnBasedBwe::IncomingPacketFeedbackVector(
    const TransportPacketsFeedback& report,
    std::optional<DataRate> /*acknowledged_bitrate*/,
    std::optional<DataRate> /*probe_bitrate*/,
    std::optional<NetworkStateEstimate> /*estimate*/, bool /*in_alr*/) {
  ECNResult result;
  const Timestamp now = report.feedback_time;
  if (!now.IsFinite()) return result;

  // Close the previous window before admitting the current feedback batch.
  // Never manufacture clean windows during feedback silence.
  if (window_start_.IsFinite() && now >= window_start_ + window_duration_) {
    if (window_total_ > 0) {
      const double fraction = static_cast<double>(window_ce_) / window_total_;
      constexpr double g = 1.0 / 16.0;
      alpha_ = (1.0 - g) * alpha_ + g * fraction;
      // A clean window decays alpha but does not request a fresh reduction.
      if (window_ce_ > 0 &&
          (last_md_time_.IsInfinite() || now - last_md_time_ >= window_duration_)) {
        // Proportional DCTCP-inspired response. No arbitrary minimum 10% cut.
        const double factor = std::clamp(1.0 - alpha_ / 2.0, 0.5, 1.0);
        const DataRate previous = current_target_rate_;
        current_target_rate_ = std::clamp(previous * factor, min_bitrate_, max_bitrate_);
        last_md_time_ = now;
        result.updated = current_target_rate_ < previous;
        result.target_bitrate = current_target_rate_;
        RTC_LOG(LS_VERBOSE) << "[ECN BWE] window packets=" << window_total_
                           << " ce=" << window_ce_ << " fraction=" << fraction
                           << " alpha=" << alpha_ << " factor=" << factor
                           << " target_kbps=" << current_target_rate_.kbps();
      }
    }
    window_total_ = window_ce_ = 0;
    window_start_ = Timestamp::MinusInfinity();
  }

  // Adapter semantics: ect_count includes ECT(0), ECT(1), AND CE.
  // Counts must represent newly acknowledged received packets.
  if (report.packet_feedbacks.empty() || report.ect_count <= 0 ||
      report.ce_count < 0 || report.ce_count > report.ect_count)
    return result;
  if (window_start_.IsInfinite()) {
    window_start_ = now;
    window_duration_ = std::max(current_rtt_, TimeDelta::Millis(20));
  }
  if (now < window_start_) return result;
  window_total_ += report.ect_count;
  window_ce_ += report.ce_count;
  return result;
}
}  // namespace webrtc
