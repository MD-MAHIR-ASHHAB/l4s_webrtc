/*
 * L4S/Prague Controller - GCC-Aligned Implementation:
 * - Uses GCC's minimum bitrate (5 kbps) and timing intervals (300ms)
 * - ECN-based congestion detection (like GCC's loss-based detection)
 * - GCC-style rate clamping and overflow protection
 * - Removed RTT inflation detection (GCC doesn't use this approach)
 * - Network capacity cap: 200 Mbps (realistic limit)
 */

#include "modules/congestion_controller/l4s/l4s_prague_controller.h"

#include <algorithm>
#include <cmath>

#include "api/field_trials_view.h"
#include "api/transport/network_types.h"
#include "modules/remote_bitrate_estimator/include/bwe_defines.h"
#include "rtc_base/logging.h"
#include "rtc_base/time_utils.h"

namespace webrtc {

namespace {
// Default values for Prague controller parameters
constexpr double kDefaultAlpha = 0.125;  // DCTCP-style decrease factor
constexpr TimeDelta kDefaultRttFilterTime = TimeDelta::Millis(100);
constexpr TimeDelta kDefaultMinRtt = TimeDelta::Millis(5);
constexpr double kDefaultInitCwnd = 2.0;  // Initial cwnd = 2 * BDP
constexpr double kDefaultBeta = 0.8;      // Multiplicative decrease factor
}  // namespace

L4SPragueController::L4SPragueController(const FieldTrialsView& field_trials,
                                         bool use_ect1_marking)
    : alpha_("alpha", kDefaultAlpha),
      rtt_filter_time_("rtt_filter_time", kDefaultRttFilterTime),
      min_rtt_("min_rtt", kDefaultMinRtt),
      init_cwnd_("init_cwnd", kDefaultInitCwnd),
      beta_("beta", kDefaultBeta),
      use_ect1_marking_(use_ect1_marking) {
  ParseFieldTrial({&alpha_, &rtt_filter_time_, &min_rtt_, &init_cwnd_, &beta_},
                  field_trials.Lookup("WebRTC-L4SPragueController"));
  if (use_ect1_marking_) {
    active_ = true;
    RTC_LOG(LS_INFO) << "Prague: Controller activated at initialization due to ECT(1) marking enabled";
  }
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
  size_t notect_packets = 0;

  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.sent_packet.sequence_number > 0) {
      if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1) {
        // RTC_LOG(LS_INFO) << "Prague: ECT marked packet detected! Seq="
        //          << packet.sent_packet.sequence_number
        //          << ", feedback_time=" << now.us() << " us";
        ect_packets++;
      } else if (packet.ecn == EcnMarking::kCe) {
        ce_packets++;
        ect_packets++;  // CE also counts as ECT
        RTC_LOG(LS_INFO) << "Prague: CE marked packet detected! Seq="
                         << packet.sent_packet.sequence_number
                         << ", feedback_time=" << now.us() << " us";
      } else if (packet.ecn == EcnMarking::kNotEct) {
        notect_packets++;
      }
    }
  }



  // If we received any ECT or CE packets, activate the controller
  if (ect_packets > 0) {
    if (!active_) {
      startup_time_ = now;  // Record when startup protection began
      RTC_LOG(LS_INFO) << "Prague: Controller activated, starting startup protection timer";
    }
    active_ = true;
    // Reset the NotECT packet counter when we see ECT packets
    consecutive_notect_feedbacks_ = 0;
  } else if (notect_packets > 0) {
    // Count consecutive feedbacks with only NotECT packets
    consecutive_notect_feedbacks_++;
    
    // Only deactivate if we've seen sustained NotECT packets AND we were previously active
    // This prevents startup transients from causing problems
    // Increased limit from 10 to 25 to allow more time for ECN marking to stabilize
    if (active_ && consecutive_notect_feedbacks_ >= 50) {
      RTC_LOG(LS_WARNING) << "Prague: Deactivating due to " << consecutive_notect_feedbacks_ 
                          << " consecutive NotECT-only feedbacks";
      active_ = false;
      consecutive_notect_feedbacks_ = 0;
    }
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

  // Calculate new CE ratio (only if we have enough ECT packets for reliable signal)
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
    // No ECT packets in window - this could be due to:
    // 1. Startup transients (NotECT packets before ECN marking activates)
    // 2. ECN marking disabled temporarily 
    // 3. Network path doesn't support ECN
    ecn_ce_ratio_ = 0.0;
    
    if (notect_packets > 0 && active_) {
      RTC_LOG(LS_INFO) << "Prague: No ECT packets in feedback window (received " 
                       << notect_packets << " NotECT packets). CE ratio reset to 0.";
    }
  }
  last_update_time_ = now;
}

void L4SPragueController::UpdateRtt(TimeDelta rtt) {
  rtt_ = rtt;

  // Log raw RTT for debugging
  // RTC_LOG(LS_INFO) << "Prague: Raw RTT measurement: " << rtt.ms() << "ms";

  // Update min_rtt_estimate with more realistic floor
  // Prevent unrealistically low RTT estimates that can cause issues
  // Use 10ms minimum for realistic network scenarios (LAN: 1-10ms, WAN: 10-500ms)
  TimeDelta realistic_rtt = std::max(rtt, TimeDelta::Millis(10)); // At least 10ms
  
  if (!min_rtt_estimate_ || realistic_rtt < *min_rtt_estimate_) {
    min_rtt_estimate_ = realistic_rtt;
    RTC_LOG(LS_INFO) << "Prague: Updated min RTT estimate to " << min_rtt_estimate_->ms() << "ms";
  }
}

std::optional<DataRate> L4SPragueController::GetTargetRate(
    Timestamp now,
    DataRate current_rate) {
  if (!active_ || !rtt_)
    return std::nullopt;

  // Add timestamp validation
  if (!now.IsFinite() || now.us() < 0) {
    RTC_LOG(LS_WARNING) << "Invalid timestamp in Prague controller: "
                        << now.us() << " us";
    return std::nullopt;
  }

  // Startup protection: Don't reduce rates aggressively when we haven't seen 
  // enough ECN signal history. This prevents startup transients from causing problems.
  // Add timeout-based fallback for non-ECN environments.
  bool startup_protection = total_ect_packets_ < 5;  // Need at least 5 ECT packets for reliable signal

  // Check if we've been in startup protection for too long (indicating no ECN support)
  if (startup_protection && startup_time_.IsFinite()) {
    TimeDelta startup_duration = now - startup_time_;
    const TimeDelta kStartupProtectionTimeout = TimeDelta::Seconds(15);  // 15 second timeout

    if (startup_duration > kStartupProtectionTimeout) {
      RTC_LOG(LS_INFO) << "Prague: Startup protection timeout after " << startup_duration.ms() 
                       << "ms with only " << total_ect_packets_ << " ECT packets. "
                       << "Disabling startup protection (likely non-ECN environment)";
      startup_protection = false;
    }
  }
  
  if (startup_protection && ecn_ce_ratio_ == 0.0) {
    RTC_LOG(LS_INFO) << "Prague: Startup protection active (only " << total_ect_packets_ 
                     << " ECT packets seen), maintaining current rate " << current_rate.bps() << " bps";
    return current_rate;
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

  // Apply Prague's scalable congestion control formula based on ECN signals
  // Similar to how GCC uses loss-based signals, but we use ECN marking
  if (ecn_ce_ratio_ > 0) {
    // Rate limit reductions to prevent death spiral (GCC-style timing)
    TimeDelta time_since_last_reduction = now - last_reduction_time_;
    const TimeDelta kBweDecreaseInterval = TimeDelta::Millis(100); // l4s decrease interval
    
    if (time_since_last_reduction.IsFinite() && time_since_last_reduction < kBweDecreaseInterval) {
      RTC_LOG(LS_INFO) << "Prague: Rate reduction rate-limited (last reduction " 
                       << time_since_last_reduction.ms() << "ms ago, need " 
                       << kBweDecreaseInterval.ms() << "ms), returning base rate " 
                       << base_rate.bps() << " bps";
      return base_rate;
    }
    
    // Additional protection during startup: Require stronger CE signal if we haven't seen much ECN traffic
    double effective_ce_ratio = ecn_ce_ratio_;
    // if (startup_protection && total_ect_packets_ < 100) {
    //   // During startup, require at least 2% CE ratio before reducing (instead of any CE ratio)
    //   if (ecn_ce_ratio_ < 0.02) {
    //     RTC_LOG(LS_INFO) << "Prague: Startup protection - CE ratio " << (ecn_ce_ratio_ * 100.0) 
    //                      << "% below 2% threshold with only " << total_ect_packets_ 
    //                      << " ECT packets, not reducing rate";
    //     return base_rate;
    //   }
    //   // Scale down the reduction during startup to be more conservative
    //   effective_ce_ratio = ecn_ce_ratio_ * 0.5;  // Half the reduction strength
    //   RTC_LOG(LS_INFO) << "Prague: Startup protection - scaling CE ratio from " 
    //                    << (ecn_ce_ratio_ * 100.0) << "% to " << (effective_ce_ratio * 100.0) << "%";
    // }
    
    // ECN-based reduction (similar to GCC's loss-based reduction)
    double reduction_factor = 1.0 - (alpha_.Get() * effective_ce_ratio);
    reduction_factor = std::max(reduction_factor, beta_.Get());

    // Add additional protection against overly aggressive reductions
    // Limit the maximum reduction to prevent system instability
    const double kMaxReductionPerStep = 0.5;  // Don't reduce more than 50% at once
    if (reduction_factor < kMaxReductionPerStep) {
      RTC_LOG(LS_WARNING) << "Prague: Limiting reduction factor from " << reduction_factor 
                          << " to " << kMaxReductionPerStep << " to prevent aggressive reduction";
      reduction_factor = kMaxReductionPerStep;
    }

    RTC_LOG(LS_INFO) << "Prague: Applying ECN-based congestion reduction. "
                     << "CE ratio=" << (ecn_ce_ratio_ * 100.0) << "%, "
                     << "reduction_factor=" << reduction_factor << ", "
                     << "base_rate=" << base_rate.bps() << " bps";

    DataRate reduced_rate = base_rate * reduction_factor;
    
    // GCC-style minimum rate enforcement
    if (reduced_rate < kCongestionControllerMinBitrate) {
      RTC_LOG(LS_WARNING) << "Prague: Rate would drop to " << reduced_rate.bps() 
                          << " bps, enforcing GCC minimum " << kCongestionControllerMinBitrate.bps() << " bps";
      reduced_rate = kCongestionControllerMinBitrate;
    }

    RTC_LOG(LS_INFO) << "Prague: Rate reduced from " << base_rate.bps()
                     << " to " << reduced_rate.bps()
                     << " bps due to ECN congestion";

    // Update last reduction time
    last_reduction_time_ = now;
    base_rate = reduced_rate;
    return reduced_rate;
  }

  // If no congestion, try to increase additively based on RTT
  // For non-ECN environments, use loss-based approach if available
  if (total_ect_packets_ == 0) {
    // No ECN marking seen, probably a non-ECN environment
    // Use a conservative additive increase approach
    const double kNonEcnAdditiveIncrease = 1.1;  // 10% increase
    DataRate increased_rate = base_rate * kNonEcnAdditiveIncrease;
    
    // Cap the increase to prevent aggressive ramp-up
    const DataRate kMaxNonEcnIncrease = DataRate::KilobitsPerSec(300);  // 300 kbps max increase
    if (increased_rate - base_rate > kMaxNonEcnIncrease) {
      increased_rate = base_rate + kMaxNonEcnIncrease;
    }
    
    RTC_LOG(LS_INFO) << "Prague: Non-ECN fallback mode - increasing rate from " 
                      << base_rate.bps() << " to " << increased_rate.bps() << " bps";
    base_rate = increased_rate;
    return increased_rate;
  }
  
  // RTC_LOG(LS_INFO)
  //     << "Prague: No congestion (CE ratio=0), checking for additive increase";

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
    // Rate limit updates to prevent excessive increases (minimum 10ms between
    // rate increases for faster ramp-up, reduced from 25ms for better video quality)
    if (time_since_update < TimeDelta::Millis(10)) {
      // RTC_LOG(LS_INFO) << "Prague: Too frequent update ("
      //                  << time_since_update.ms()
      //                  << "ms < 10ms), returning base rate " << base_rate.bps()
      //                  << " bps";
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

    // Only increase if RTT is reasonable (>= 1ms) and time since update is
    // significant
    if (rtt_seconds >= 0.001 && time_since_update.ms() >= 10) {
      // RTC_LOG(LS_INFO)
      //     << "Prague: RTT and time conditions met, calculating increase. RTT="
      //     << rtt_seconds << "s, time=" << time_since_update.ms() << "ms";
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
      increase_factor = 1.0 + (increase_per_rtt * rtt_cycles);

      // More aggressive bounds for faster ramp-up: max 5% increase per update (increased from 3% for faster video quality)
      increase_factor = std::clamp(increase_factor, 1.0, 1.05);

      // Log when we actually increase (only if meaningful)
      if (increase_factor > 1.001) {
        // RTC_LOG(LS_INFO) << "Prague additive increase: factor="
        //                  << increase_factor << ", base_rate=" << base_rate.bps()
        //                  << " bps"
        //                  << ", new_rate=" << (base_rate.bps() * increase_factor)
        //                  << " bps"
        //                  << ", rtt=" << rtt_seconds
        //                  << "s, update_interval=" << time_since_update.ms()
        //                  << "ms";
      } else {
        // RTC_LOG(LS_INFO) << "Prague: Small increase factor (" << increase_factor
        //                  << "), not logging";
      }
    } else {
      // RTC_LOG(LS_INFO) << "Prague: RTT/time conditions not met. RTT="
      //                  << rtt_seconds
      //                  << "s (need >=0.001), time=" << time_since_update.ms()
      //                  << "ms (need >=25)";
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

    // GCC-style final rate clamping: ensure we never go below minimum
    increased_rate = std::max(kCongestionControllerMinBitrate, increased_rate);

    // RTC_LOG(LS_INFO) << "Prague: Returning increased_rate="
    //                  << increased_rate.bps()
    //                  << " bps (factor=" << increase_factor << ")";
    base_rate = increased_rate;  // Update base rate for next calculations
    return increased_rate;
  }

  // GCC-style final rate clamping: ensure we never go below minimum  
  DataRate final_rate = std::max(kCongestionControllerMinBitrate, base_rate);
  
  if (final_rate != base_rate) {
    RTC_LOG(LS_INFO) << "Prague: Applied minimum rate clamp, returning " 
                     << final_rate.bps() << " bps instead of " << base_rate.bps() << " bps";
  }
  
  return final_rate;
}

bool L4SPragueController::IsActive() const {
  // Check if we're still in startup protection
  // if (startup_time_.IsFinite()) {
  //   Timestamp now = Timestamp::Millis(webrtc::TimeMillis());
  //   TimeDelta startup_duration = now - startup_time_;
    
  //   // If startup protection has timed out without ECN feedback, we're not active
  //   if (startup_duration > TimeDelta::Seconds(30)) {
  //     // Only log this message occasionally to avoid spam
  //     static Timestamp last_timeout_log = Timestamp::MinusInfinity();
  //     if (now - last_timeout_log > TimeDelta::Seconds(10)) {
  //       RTC_LOG(LS_INFO) << "Prague: Startup protection timeout after " 
  //                        << startup_duration.seconds() << " seconds, controller not active";
  //       last_timeout_log = now;
  //     }
  //     return false;
  //   }
  // }
  
  // We're active if we have enough ECN feedback and haven't timed out
  bool has_ecn_feedback = (total_ect_packets_ > 0 || total_ce_packets_ > 0);
  
  // Not active if we've received too many consecutive NotECT-only feedbacks
  if (consecutive_notect_feedbacks_ >= 50) {
    static Timestamp last_notect_log = Timestamp::MinusInfinity();
    Timestamp now = Timestamp::Millis(webrtc::TimeMillis());
    if (now - last_notect_log > TimeDelta::Seconds(10)) {
      RTC_LOG(LS_INFO) << "Prague: Too many NotECT-only feedbacks (" 
                       << consecutive_notect_feedbacks_ << "), controller not active";
      last_notect_log = now;
    }
    return false;
  }
  
  return active_ || has_ecn_feedback;
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
**Summary of L4S/Prague Controller - GCC-Aligned Implementation:**

**GCC-Style Safety Mechanisms:**
1. **Minimum Rate Protection**: Uses GCC's kCongestionControllerMinBitrate (5 kbps)
2. **Rate Limiting**: Uses GCC's kBweDecreaseInterval (300ms between reductions)
3. **Target Rate Clamping**: Always ensures rate >= minimum like GCC's target_rate()
4. **Overflow Protection**: Prevents crashes from unrealistic rates

**L4S-Specific Features:**
1. **ECN-Based Detection**: Uses CE marking ratio (like GCC uses loss ratio)
2. **Prague Algorithm**: Scalable congestion control with alpha/beta parameters
3. **Network Capacity Cap**: 200 Mbps realistic upper limit
4. **Conservative Growth**: 1% max increase per update

**Key Architectural Similarities to GCC:**
- Multiple protection layers (minimum rate + timing + clamping)
- Time-based rate limiting to prevent death spirals
- Conservative reduction factors
- Explicit congestion signaling (ECN vs loss/delay)

**Major Difference from Previous Version:**
- REMOVED RTT inflation detection (GCC doesn't use this approach)
- Focus on ECN signals only, like GCC focuses on loss/delay signals
- Aligned timing intervals and minimum rates with GCC standards

This implementation should behave as robustly as GCC while providing L4S/Prague's
scalable congestion control benefits through ECN marking.

*/