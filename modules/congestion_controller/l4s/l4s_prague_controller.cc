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
  
  // Add timestamp validation
  if (!now.IsFinite() || now.us() < 0) {
    RTC_LOG(LS_WARNING) << "Invalid feedback timestamp in Prague controller: " << now.us() << " us";
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
        ect_packets++; // CE also counts as ECT
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
                     << " (total ECT+CE so far: " << (total_ect_packets_ + ect_packets) << ")";
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
      RTC_LOG(LS_WARNING) << "Invalid age calculation in ECN window cleanup, clearing window";
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
                       << (ecn_ce_ratio_ * 100.0) << "% (" 
                       << total_ce_packets_ << " CE out of " 
                       << total_ect_packets_ << " ECT packets)";
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
    Timestamp now) const {
  if (!active_ || !rtt_)
    return std::nullopt;
  
  // Add timestamp validation
  if (!now.IsFinite() || now.us() < 0) {
    RTC_LOG(LS_WARNING) << "Invalid timestamp in Prague controller: " << now.us() << " us";
    return std::nullopt;
  }
  

  
  // Use at least the minimum RTT
  TimeDelta current_rtt = rtt_.value();
  if (min_rtt_estimate_.has_value()) {
    current_rtt = std::max(current_rtt, min_rtt_estimate_.value());
  }
  
  // Calculate the base rate using the congestion window
  DataSize cwnd = CalculateCongestionWindow();
  
  // Add safety checks to prevent division by zero or overflow
  if (current_rtt.ms() <= 0) {
    RTC_LOG(LS_WARNING) << "Invalid RTT in Prague controller: " << current_rtt.ms() << "ms, using default rate";
    return DataRate::KilobitsPerSec(300);  // Default fallback rate
  }
  
  // Prevent potential overflow in multiplication
  int64_t cwnd_bits = static_cast<int64_t>(cwnd.bytes()) * 8;
  int64_t rtt_ms = current_rtt.ms();
  
  // Check for potential overflow
  if (cwnd_bits > (std::numeric_limits<int64_t>::max() / 1000)) {
    RTC_LOG(LS_WARNING) << "Potential overflow in rate calculation, using default rate";
    return DataRate::KilobitsPerSec(300);
  }
  
  int64_t rate_bps = (cwnd_bits * 1000) / rtt_ms;
  
  // Ensure the result is positive and reasonable
  if (rate_bps <= 0) {
    RTC_LOG(LS_WARNING) << "Invalid rate calculation result: " << rate_bps << " bps, using default";
    return DataRate::KilobitsPerSec(300);
  }
  
  DataRate base_rate = DataRate::BitsPerSec(rate_bps);
  
  // Apply Prague's scalable congestion control formula
  if (ecn_ce_ratio_ > 0) {
    // If we have congestion signals, apply the Prague reduction
    double reduction_factor = 1.0 - (alpha_.Get() * ecn_ce_ratio_);
    reduction_factor = std::max(reduction_factor, beta_.Get());
    
    RTC_LOG(LS_INFO) << "Prague: Applying congestion reduction due to CE marks. "
                     << "CE ratio=" << (ecn_ce_ratio_ * 100.0) << "%, "
                     << "reduction_factor=" << reduction_factor << ", "
                     << "base_rate=" << base_rate.bps() << " bps";
    
    DataRate reduced_rate = base_rate * reduction_factor;
    
    RTC_LOG(LS_INFO) << "Prague: Rate reduced from " << base_rate.bps() 
                     << " to " << reduced_rate.bps() << " bps due to congestion";
    
    return reduced_rate;
  }
  
  // If no congestion, increase additively based on RTT
  // Add safety check for uninitialized last_update_time
  if (!last_update_time_.IsFinite()) {
    return base_rate;
  }
  
  TimeDelta time_since_update = now - last_update_time_;
  
  // Add safety check for time calculation
  if (!time_since_update.IsFinite()) {
    RTC_LOG(LS_WARNING) << "Invalid time_since_update calculation in Prague controller";
    return base_rate;
  }
  
  if (time_since_update > TimeDelta::Zero()) {

    // // Rate limit updates to prevent excessive increases (minimum 200ms between rate increases)
    // if (time_since_update < TimeDelta::Millis(50)) {
      // Too frequent updates, return current rate without increase
    //   return base_rate;
    // Rate limit updates to prevent excessive increases (minimum 100ms between rate increases)
    if (time_since_update < TimeDelta::Millis(100)) {
      // Too frequent updates, return current rate without increase
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
    
    // Only increase if RTT is reasonable (>= 5ms) and time since update is significant (>= 200ms)
    // if (rtt_seconds >= 0.005 && time_since_update.ms() >= 200) {
    //   // Target: very small increase per RTT (much more conservative than TCP)
    //   DataSize packet_size = DataSize::Bytes(1500);  // Assume 1500-byte packets
    //   DataSize current_cwnd = CalculateCongestionWindow();
      
    //   // Much smaller base increase rate
    //   double increase_per_rtt = packet_size.bytes() / static_cast<double>(current_cwnd.bytes());
    //   double rtt_cycles = time_since_update.ms() / (rtt_seconds * 1000.0);
      
    //   // Scale down the increase by 20x to be extremely conservative
    //   increase_factor = 1.0 + (increase_per_rtt * rtt_cycles * 0.05);
      
    //   // Very tight bounds: max 1% increase per update
    //   increase_factor = std::clamp(increase_factor, 1.0, 1.01);
      
    //   // Log only when we actually increase
    //   if (increase_factor > 1.001) {
    //     RTC_LOG(LS_INFO) << "Prague additive increase: factor=" << increase_factor 
    //                      << ", rtt=" << rtt_seconds << "s, update_interval=" << time_since_update.ms() << "ms";
    //   }
    // }
    
    // Only increase if RTT is reasonable (>= 5ms) and time since update is significant
    if (rtt_seconds >= 0.005 && time_since_update.ms() >= 100) {
      // Target: very small increase per RTT (much more conservative than TCP)
      DataSize packet_size = DataSize::Bytes(1500);  // Assume 1500-byte packets
      DataSize current_cwnd = CalculateCongestionWindow();
      
      // Much smaller base increase rate
      double increase_per_rtt = packet_size.bytes() / static_cast<double>(current_cwnd.bytes());
      double rtt_cycles = time_since_update.ms() / (rtt_seconds * 1000.0);
      
      // Scale down the increase by 10x to be very conservative
      increase_factor = 1.0 + (increase_per_rtt * rtt_cycles * 0.1);
      // Scale down the increase by 10x to be little bit conservative
      increase_factor = 1.0 + (increase_per_rtt * rtt_cycles * 0.5);
      // Very tight bounds: max 2% increase per update
      increase_factor = std::clamp(increase_factor, 1.0, 1.02);
      
      // Not Very tight bounds: max 5% increase per update
      increase_factor = std::clamp(increase_factor, 1.0, 1.05);
    
    }
    
    DataRate increased_rate = base_rate * increase_factor;
    
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
  // DataRate base_rate = DataRate::KilobitsPerSec(1000);  // 1 Mbps base (for testing)
  DataSize bdp = base_rate * base_rtt;
  
  // Apply the initial window multiplier
  DataSize cwnd = bdp * init_cwnd_;
  
  // RTC_LOG(LS_INFO) << "Prague after init_cwnd multiplication: cwnd.bytes()=" << cwnd.bytes()
  //                  << ", cwnd.IsFinite()=" << (cwnd.IsFinite() ? "true" : "false");
  
  // Apply reduction based on CE marking ratio
  if (ecn_ce_ratio_ > 0) {
    double reduction = std::max(1.0 - ecn_ce_ratio_, beta_.Get());
    // RTC_LOG(LS_INFO) << "Prague applying congestion window reduction: ratio=" << ecn_ce_ratio_
    //                  << ", reduction_factor=" << reduction
    //                  << ", cwnd_before=" << cwnd.bytes();
    
    // Log before another multiplication that might be problematic
    // RTC_LOG(LS_INFO) << "Prague PRE-REDUCTION-MULTIPLY: cwnd.bytes()=" << cwnd.bytes()
    //                  << ", reduction=" << reduction
    //                  << ", cwnd.IsFinite()=" << (cwnd.IsFinite() ? "true" : "false");
    
    cwnd = cwnd * reduction;
    
    // RTC_LOG(LS_INFO) << "Prague POST-REDUCTION-MULTIPLY: cwnd.bytes()=" << cwnd.bytes()
    //                  << ", cwnd.IsFinite()=" << (cwnd.IsFinite() ? "true" : "false");
  }
  
  // Ensure minimum congestion window (2 packets)
  // RTC_LOG(LS_INFO) << "Prague before std::max: cwnd.bytes()=" << cwnd.bytes()
  //                  << ", min_cwnd=3000 bytes";
  
  DataSize final_cwnd = std::max(cwnd, DataSize::Bytes(1500 * 2));
  
  // RTC_LOG(LS_INFO) << "Prague final cwnd.bytes()=" << final_cwnd.bytes()
  //                  << ", final_cwnd.IsFinite()=" << (final_cwnd.IsFinite() ? "true" : "false");
  
  return final_cwnd;
}

}  // namespace webrtc