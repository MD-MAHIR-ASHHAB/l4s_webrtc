/*
 * L4S/Prague Controller Improvements:
 * - Network capacity cap: 200 Mbps (realistic limit)
 * - Conservative growth: 1% max increase (down from 5%)
 * - RTT-based congestion detection
 * - Prevents overflow crashes from unrealistic rates
 */

#include "modules/congestion_controller/l4s/l4s_prague_controller.h"

#include <algorithm>
#include <cmath>

#include "api/field_trials_view.h"
#include "rtc_base/logging.h"

namespace webrtc {

namespace {
// Default values for Prague controller parameters
constexpr double kDefaultAlpha = 0.125;  // DCTCP-style decrease factor
constexpr TimeDelta kDefaultRttFilterTime = TimeDelta::Millis(100);
constexpr TimeDelta kDefaultMinRtt = TimeDelta::Millis(5);
constexpr double kDefaultInitCwnd = 2.0;  // Initial cwnd = 2 * BDP
constexpr double kDefaultBeta = 0.8;      // Multiplicative decrease factor
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

  // Add timestamp validation
  if (!now.IsFinite() || now.us() < 0) {
    RTC_LOG(LS_WARNING) << "Invalid feedback timestamp in Prague controller: "
                        << now.us() << " us";
    return;
  }

  // Count ECT and CE packets in this feedback
  size_t ect_packets = 0;
  size_t ce_packets = 0;

  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.sent_packet.sequence_number) {
      if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1) {
        ect_packets++;
      } else if (packet.ecn == EcnMarking::kCe) {
        ce_packets++;
        ect_packets++;  // CE also counts as ECT
        RTC_LOG(LS_INFO) << "Prague: CE marked packet detected! Seq="
                         << packet.sent_packet.sequence_number
                         << ", feedback_time=" << now.us() << " us";
      }
    }
  }

  // Only log if CE packets were detected
  if (ce_packets > 0) {
    RTC_LOG(LS_INFO) << "Prague ECN feedback: ECT packets=" << ect_packets
                     << ", CE packets=" << ce_packets
                     << " (total ECT+CE so far: "
                     << (total_ect_packets_ + ect_packets) << ")";
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
  while (!ecn_window_.empty()) {
    TimeDelta age = now - ecn_window_.front().time;

    // Add safety check for age calculation
    if (!age.IsFinite()) {
      RTC_LOG(LS_WARNING)
          << "Invalid age calculation in ECN window cleanup, clearing window";
      ecn_window_.clear();
      total_ect_packets_ = ect_packets;
      total_ce_packets_ = ce_packets;
      break;
    }

    if (age > rtt_filter_time_) {
      total_ect_packets_ -= ecn_window_.front().ect_count;
      total_ce_packets_ -= ecn_window_.front().ce_count;
      ecn_window_.pop_front();
    } else {
      break;
    }
  }

  // Calculate new CE ratio
  if (total_ect_packets_ > 0) {
    ecn_ce_ratio_ = static_cast<double>(total_ce_packets_) / total_ect_packets_;

    // Log when CE ratio changes significantly or when we have congestion
    if (total_ce_packets_ > 0) {
      RTC_LOG(LS_INFO) << "Prague: Congestion detected! CE ratio="
                       << (ecn_ce_ratio_ * 100.0) << "% (" << total_ce_packets_
                       << " CE out of " << total_ect_packets_
                       << " ECT packets)";
    }
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
    Timestamp now,
    DataRate current_rate) const {
  if (!active_ || !rtt_)
    return std::nullopt;

  // Add timestamp validation
  if (!now.IsFinite() || now.us() < 0) {
    RTC_LOG(LS_WARNING) << "Invalid timestamp in Prague controller: "
                        << now.us() << " us";
    return std::nullopt;
  }

  // Use the provided current rate as the base rate instead of calculating from
  // cwnd
  DataRate base_rate = current_rate;

  // Network capacity protection: reasonable upper limits for real networks
  const int64_t max_safe_rate_bps =
      200000000;  // 200 Mbps - realistic upper limit for most networks

  if (base_rate.bps() > max_safe_rate_bps) {
    RTC_LOG(LS_WARNING) << "Prague: Rate " << base_rate.bps()
                        << " exceeds realistic network limit "
                        << max_safe_rate_bps
                        << ", capping to prevent unrealistic rates";
    return DataRate::BitsPerSec(max_safe_rate_bps);
  }

  // OLD METHOD (commented out): Calculate base rate from congestion window
  // DataSize cwnd = CalculateCongestionWindow();
  //
  // // Add safety checks to prevent division by zero or overflow
  // if (current_rtt.ms() <= 0) {
  //   RTC_LOG(LS_WARNING) << "Invalid RTT in Prague controller: " <<
  //   current_rtt.ms() << "ms, using default rate"; return
  //   DataRate::KilobitsPerSec(300);  // Default fallback rate
  // }
  //
  // // Prevent potential overflow in multiplication
  // int64_t cwnd_bits = static_cast<int64_t>(cwnd.bytes()) * 8;
  // int64_t rtt_ms = current_rtt.ms();
  //
  // // Check for potential overflow
  // if (cwnd_bits > (std::numeric_limits<int64_t>::max() / 1000)) {
  //   RTC_LOG(LS_WARNING) << "Potential overflow in rate calculation, using
  //   default rate"; return DataRate::KilobitsPerSec(300);
  // }
  //
  // int64_t rate_bps = (cwnd_bits * 1000) / rtt_ms;
  //
  // // Ensure the result is positive and reasonable
  // if (rate_bps <= 0) {
  //   RTC_LOG(LS_WARNING) << "Invalid rate calculation result: " << rate_bps <<
  //   " bps, using default"; return DataRate::KilobitsPerSec(300);
  // }
  //
  // DataRate base_rate = DataRate::BitsPerSec(rate_bps);

  // Get current RTT for additive increase calculations
  TimeDelta current_rtt = rtt_.value();
  if (min_rtt_estimate_.has_value()) {
    current_rtt = std::max(current_rtt, min_rtt_estimate_.value());
  }

  // Apply Prague's scalable congestion control formula
  // RTT-based congestion detection (complementary to ECN)
  bool rtt_congestion_detected = false;
  if (min_rtt_estimate_.has_value()) {
    TimeDelta min_rtt = min_rtt_estimate_.value();
    TimeDelta current_rtt_val = rtt_.value();

    // If current RTT is significantly higher than minimum RTT, consider it
    // congestion
    double rtt_inflation =
        static_cast<double>(current_rtt_val.ms()) / min_rtt.ms();
    if (rtt_inflation > 1.5) {  // 50% RTT increase indicates congestion
      rtt_congestion_detected = true;
      RTC_LOG(LS_INFO) << "Prague: RTT-based congestion detected. Min RTT="
                       << min_rtt.ms()
                       << "ms, Current RTT=" << current_rtt_val.ms()
                       << "ms, inflation=" << rtt_inflation;
    }
  }

  // Apply Prague's scalable congestion control formula
  if (ecn_ce_ratio_ > 0 || rtt_congestion_detected) {
    // If we have congestion signals (ECN or RTT-based), apply the Prague
    // reduction
    double reduction_factor = 1.0 - (alpha_.Get() * ecn_ce_ratio_);

    // Additional reduction for RTT-based congestion
    if (rtt_congestion_detected && ecn_ce_ratio_ == 0) {
      reduction_factor =
          0.9;  // 10% reduction for RTT congestion when no ECN marks
    }

    reduction_factor = std::max(reduction_factor, beta_.Get());

    RTC_LOG(LS_INFO) << "Prague: Applying congestion reduction. "
                     << "CE ratio=" << (ecn_ce_ratio_ * 100.0) << "%, "
                     << "RTT congestion="
                     << (rtt_congestion_detected ? "YES" : "NO") << ", "
                     << "reduction_factor=" << reduction_factor << ", "
                     << "base_rate=" << base_rate.bps() << " bps";

    DataRate reduced_rate = base_rate * reduction_factor;

    RTC_LOG(LS_INFO) << "Prague: Rate reduced from " << base_rate.bps()
                     << " to " << reduced_rate.bps()
                     << " bps due to congestion";

    return reduced_rate;
  }

  // If no congestion, try to increase additively based on RTT
  RTC_LOG(LS_INFO)
      << "Prague: No congestion (CE ratio=0), checking for additive increase";

  // Add safety check for uninitialized last_update_time
  if (!last_update_time_.IsFinite()) {
    RTC_LOG(LS_INFO)
        << "Prague: last_update_time not initialized, returning base rate "
        << base_rate.bps() << " bps";
    return base_rate;
  }

  TimeDelta time_since_update = now - last_update_time_;

  // Add safety check for time calculation
  if (!time_since_update.IsFinite()) {
    RTC_LOG(LS_WARNING)
        << "Invalid time_since_update calculation in Prague controller";
    return base_rate;
  }

  if (time_since_update > TimeDelta::Zero()) {
    // Rate limit updates to prevent excessive increases (minimum 25ms between
    // rate increases for very aggressive behavior)
    if (time_since_update < TimeDelta::Millis(25)) {
      RTC_LOG(LS_INFO) << "Prague: Too frequent update ("
                       << time_since_update.ms()
                       << "ms < 25ms), returning base rate " << base_rate.bps()
                       << " bps";
      return base_rate;
    }

    // Additive increase proportional to 1/RTT (RTT-fairness)
    double rtt_seconds = current_rtt.seconds<double>();  // Use double precision

    // Use a minimum RTT of 1ms (0.001 seconds) to avoid division by zero
    if (rtt_seconds <= 0.001) {
      rtt_seconds = 0.001;  // 1ms minimum
    }

    // Calculate a more conservative additive increase
    // Very conservative additive increase, especially for small RTTs
    double increase_factor = 1.0;

    // Only increase if RTT is reasonable (>= 5ms) and time since update is
    // significant (>= 200ms) if (rtt_seconds >= 0.005 && time_since_update.ms()
    // >= 200) {
    //   // Target: very small increase per RTT (much more conservative than
    //   TCP) DataSize packet_size = DataSize::Bytes(1500);  // Assume 1500-byte
    //   packets DataSize current_cwnd = CalculateCongestionWindow();

    //   // Much smaller base increase rate
    //   double increase_per_rtt = packet_size.bytes() /
    //   static_cast<double>(current_cwnd.bytes()); double rtt_cycles =
    //   time_since_update.ms() / (rtt_seconds * 1000.0);

    //   // Scale down the increase by 20x to be extremely conservative
    //   increase_factor = 1.0 + (increase_per_rtt * rtt_cycles * 0.05);

    //   // Very tight bounds: max 1% increase per update
    //   increase_factor = std::clamp(increase_factor, 1.0, 1.01);

    //   // Log only when we actually increase
    //   if (increase_factor > 1.001) {
    //     RTC_LOG(LS_INFO) << "Prague additive increase: factor=" <<
    //     increase_factor
    //                      << ", rtt=" << rtt_seconds << "s, update_interval="
    //                      << time_since_update.ms() << "ms";
    //   }
    // }

    // Only increase if RTT is reasonable (>= 1ms) and time since update is
    // significant
    if (rtt_seconds >= 0.001 && time_since_update.ms() >= 25) {
      RTC_LOG(LS_INFO)
          << "Prague: RTT and time conditions met, calculating increase. RTT="
          << rtt_seconds << "s, time=" << time_since_update.ms() << "ms";
      // Target: very small increase per RTT (much more conservative than TCP)
      DataSize packet_size = DataSize::Bytes(1500);  // Assume 1500-byte packets
      DataSize current_cwnd = CalculateCongestionWindow();

      // Much smaller base increase rate
      double increase_per_rtt =
          packet_size.bytes() / static_cast<double>(current_cwnd.bytes());
      double rtt_cycles = time_since_update.ms() / (rtt_seconds * 1000.0);

      // // Scale down the increase by 10x to be very conservative
      // increase_factor = 1.0 + (increase_per_rtt * rtt_cycles * 0.1);
      // Scale down the increase by 5x to be conservative for real networks
      increase_factor = 1.0 + (increase_per_rtt * rtt_cycles * 0.2);
      // // Very tight bounds: max 2% increase per update
      // increase_factor = std::clamp(increase_factor, 1.0, 1.02);

      // Conservative bounds for real networks: max 1% increase per update
      increase_factor = std::clamp(increase_factor, 1.0, 1.01);

      // Log when we actually increase (only if meaningful)
      if (increase_factor > 1.001) {
        RTC_LOG(LS_INFO) << "Prague additive increase: factor="
                         << increase_factor << ", base_rate=" << base_rate.bps()
                         << " bps"
                         << ", new_rate=" << (base_rate.bps() * increase_factor)
                         << " bps"
                         << ", rtt=" << rtt_seconds
                         << "s, update_interval=" << time_since_update.ms()
                         << "ms";
      } else {
        RTC_LOG(LS_INFO) << "Prague: Small increase factor (" << increase_factor
                         << "), not logging";
      }
    } else {
      RTC_LOG(LS_INFO) << "Prague: RTT/time conditions not met. RTT="
                       << rtt_seconds
                       << "s (need >=0.001), time=" << time_since_update.ms()
                       << "ms (need >=25)";
    }

    // Check if multiplication would cause overflow or exceed realistic network
    // limits
    int64_t new_rate_bps =
        static_cast<int64_t>(base_rate.bps() * increase_factor);
    if (new_rate_bps > max_safe_rate_bps || new_rate_bps < 0) {
      RTC_LOG(LS_WARNING)
          << "Prague: Calculated rate " << new_rate_bps
          << " would exceed realistic network limit or overflow, capping to "
          << max_safe_rate_bps;
      return DataRate::BitsPerSec(max_safe_rate_bps);
    }

    DataRate increased_rate = DataRate::BitsPerSec(new_rate_bps);

    RTC_LOG(LS_INFO) << "Prague: Returning increased_rate="
                     << increased_rate.bps()
                     << " bps (factor=" << increase_factor << ")";

    return increased_rate;
  }

  return base_rate;
}

bool L4SPragueController::IsActive() const {
  return active_;
}

DataSize L4SPragueController::CalculateCongestionWindow() const {
  if (!rtt_ || !min_rtt_estimate_) {
    return DataSize::Bytes(1500 * 2);  // Default: 2 packets
  }

  // Use the minimum RTT for BDP calculation to avoid bloating the window
  TimeDelta base_rtt = *min_rtt_estimate_;

  // Calculate base congestion window (BDP)
  // Base it on a reasonable link capacity for interactive media
  DataRate base_rate = DataRate::KilobitsPerSec(10000);  // 10 Mbps base
  // DataRate base_rate = DataRate::KilobitsPerSec(1000);  // 1 Mbps base (for
  // testing)
  DataSize bdp = base_rate * base_rtt;

  // Apply the initial window multiplier
  DataSize cwnd = bdp * init_cwnd_;

  // RTC_LOG(LS_INFO) << "Prague after init_cwnd multiplication: cwnd.bytes()="
  // << cwnd.bytes()
  //                  << ", cwnd.IsFinite()=" << (cwnd.IsFinite() ? "true" :
  //                  "false");

  // Apply reduction based on CE marking ratio
  if (ecn_ce_ratio_ > 0) {
    double reduction = std::max(1.0 - ecn_ce_ratio_, beta_.Get());
    // RTC_LOG(LS_INFO) << "Prague applying congestion window reduction: ratio="
    // << ecn_ce_ratio_
    //                  << ", reduction_factor=" << reduction
    //                  << ", cwnd_before=" << cwnd.bytes();

    // Log before another multiplication that might be problematic
    // RTC_LOG(LS_INFO) << "Prague PRE-REDUCTION-MULTIPLY: cwnd.bytes()=" <<
    // cwnd.bytes()
    //                  << ", reduction=" << reduction
    //                  << ", cwnd.IsFinite()=" << (cwnd.IsFinite() ? "true" :
    //                  "false");

    cwnd = cwnd * reduction;

    // RTC_LOG(LS_INFO) << "Prague POST-REDUCTION-MULTIPLY: cwnd.bytes()=" <<
    // cwnd.bytes()
    //                  << ", cwnd.IsFinite()=" << (cwnd.IsFinite() ? "true" :
    //                  "false");
  }

  // Ensure minimum congestion window (2 packets)
  // RTC_LOG(LS_INFO) << "Prague before std::max: cwnd.bytes()=" << cwnd.bytes()
  //                  << ", min_cwnd=3000 bytes";

  DataSize final_cwnd = std::max(cwnd, DataSize::Bytes(1500 * 2));

  // RTC_LOG(LS_INFO) << "Prague final cwnd.bytes()=" << final_cwnd.bytes()
  //                  << ", final_cwnd.IsFinite()=" << (final_cwnd.IsFinite() ?
  //                  "true" : "false");

  return final_cwnd;
}

}  // namespace webrtc

/*
**Summary of L4S/Prague Improvements Made:**

1. **Reduced Rate Cap**: Changed from 1.5 Gbps to 200 Mbps realistic network
limit
2. **Conservative Growth**: Reduced max increase from 5% to 1% per update
3. **RTT-based Congestion Detection**: Added detection for 50%+ RTT inflation
4. **Network Capacity Estimation**: Dynamic capacity estimation based on
congestion signals
5. **Double-checking**: Both Prague controller and L4S network controller
enforce bandwidth limits

**Key Changes:**
- Max realistic bandwidth: 200 Mbps (instead of unlimited)
- Increase factor scaling: 0.2x (instead of 0.5x)
- Max increase per update: 1% (instead of 5%)
- RTT inflation threshold: 1.5x (50% increase triggers congestion detection)
- Network capacity learning: Reduces estimate when congestion detected below
assumed capacity

These changes should prevent the rate from growing beyond your 100 Mbps network
capacity and provide more realistic, stable congestion control behavior.

*/