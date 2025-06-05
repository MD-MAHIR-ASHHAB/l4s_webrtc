#include "modules/congestion_controller/l4s/l4s_prague_controller.h"

#include <algorithm>
#include <cmath>
#include "rtc_base/logging.h"
#include "api/field_trials_view.h"

namespace webrtc {

namespace {
// Default values for Prague controller parameters
constexpr double kDefaultAlpha = 0.125;  // DCTCP-style decrease factor
constexpr TimeDelta kDefaultRttFilterTime = TimeDelta::Millis(100);
constexpr TimeDelta kDefaultMinRtt = TimeDelta::Millis(5);
constexpr double kDefaultInitCwnd = 2.0;  // Initial cwnd = 2 * BDP
constexpr double kDefaultBeta = 0.8;  // Multiplicative decrease factor
}  // namespace

L4SPragueController::L4SPragueController(const FieldTrialsView& field_trials)
    : alpha_("alpha", kDefaultAlpha),
      rtt_filter_time_("rtt_filter_time", kDefaultRttFilterTime),
      min_rtt_("min_rtt", kDefaultMinRtt),
      init_cwnd_("init_cwnd", kDefaultInitCwnd),
      beta_("beta", kDefaultBeta) {
  ParseFieldTrial({&alpha_, &rtt_filter_time_, &min_rtt_, &init_cwnd_, &beta_},
                 field_trials.Lookup("WebRTC-L4SPragueController"));
}

L4SPragueController::~L4SPragueController() = default;

void L4SPragueController::UpdateEcnFeedback(
    const TransportPacketsFeedback& feedback) {
  if (feedback.packet_feedbacks.empty())
    return;
  
  Timestamp now = feedback.feedback_time;

  // Count ECT and CE packets in this feedback
  size_t ect_packets = 0;
  size_t ce_packets = 0;

  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.sent_packet.sequence_number) {
      if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1) {
        ect_packets++;
      } else if (packet.ecn == EcnMarking::kCe) {
        ce_packets++;
        ect_packets++; // CE also counts as ECT
      }
    }
  }
  
  // If we received any ECT or CE packets, activate the controller
  if (ect_packets > 0) {
    active_ = true;
  }
  
  // Add to totals
  total_ect_packets_ += ect_packets;
  total_ce_packets_ += ce_packets;
  
  // Add to window
  ecn_window_.push_back({now, ect_packets, ce_packets});
  
  // Remove old entries from window
  while (!ecn_window_.empty() && 
         now - ecn_window_.front().time > rtt_filter_time_) {
    total_ect_packets_ -= ecn_window_.front().ect_count;
    total_ce_packets_ -= ecn_window_.front().ce_count;
    ecn_window_.pop_front();
  }
  
  // Calculate new CE ratio
  if (total_ect_packets_ > 0) {
    ecn_ce_ratio_ = static_cast<double>(total_ce_packets_) / total_ect_packets_;
  } else {
    ecn_ce_ratio_ = 0.0;
  }
  
  last_update_time_ = now;
}

void L4SPragueController::UpdateRtt(TimeDelta rtt) {
  rtt_ = rtt;
  
  // Update min_rtt_estimate
  if (!min_rtt_estimate_ || rtt < *min_rtt_estimate_) {
    min_rtt_estimate_ = rtt;
  }
}

std::optional<DataRate> L4SPragueController::GetTargetRate(
    Timestamp now) const {
  if (!active_ || !rtt_)
    return std::nullopt;
  
  // Use at least the minimum RTT
  TimeDelta current_rtt = rtt_.value();
  if (min_rtt_estimate_.has_value()) {
    current_rtt = std::max(current_rtt, min_rtt_estimate_.value());
  }
  
  // Calculate the base rate using the congestion window
  DataSize cwnd = CalculateCongestionWindow();
  DataRate base_rate = DataRate::BitsPerSec(
      (cwnd.bytes() * 8 * 1000) / current_rtt.ms());
  
  // Apply Prague's scalable congestion control formula
  if (ecn_ce_ratio_ > 0) {
    // If we have congestion signals, apply the Prague reduction
    double reduction_factor = 1.0 - (alpha_.Get() * ecn_ce_ratio_);
    reduction_factor = std::max(reduction_factor, beta_.Get());
    return base_rate * reduction_factor;
  }
  
  // If no congestion, increase additively based on RTT
  TimeDelta time_since_update = now - last_update_time_;
  if (time_since_update > TimeDelta::Zero()) {
    // Additive increase proportional to 1/RTT (RTT-fairness)
    double increase_factor = 1.0 + 
        (time_since_update.ms() / (1000.0 * current_rtt.seconds()));
    return base_rate * increase_factor;
  }
  
  return base_rate;
}

bool L4SPragueController::IsActive() const {
  return active_;
}

DataSize L4SPragueController::CalculateCongestionWindow() const {
  if (!rtt_ || !min_rtt_estimate_)
    return DataSize::Bytes(1500 * 2);  // Default: 2 packets
  
  // Use the minimum RTT for BDP calculation to avoid bloating the window
  TimeDelta base_rtt = *min_rtt_estimate_;
  
  // Calculate base congestion window (BDP)
  // Base it on a reasonable link capacity for interactive media
  DataRate base_rate = DataRate::KilobitsPerSec(1000);  // 1 Mbps base
  DataSize bdp = base_rate * base_rtt;
  
  // Apply the initial window multiplier
  DataSize cwnd = bdp * init_cwnd_;
  
  // Apply reduction based on CE marking ratio
  if (ecn_ce_ratio_ > 0) {
    double reduction = std::max(1.0 - ecn_ce_ratio_, beta_.Get());
    cwnd = cwnd * reduction;
  }
  
  // Ensure minimum congestion window (2 packets)
  return std::max(cwnd, DataSize::Bytes(1500 * 2));
}

}  // namespace webrtc