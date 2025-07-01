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
  
  // Log timestamp values before processing - with safety check for MinusInfinity
  if (last_update_time_.IsFinite()) {
    RTC_LOG(LS_INFO) << "Prague UpdateEcnFeedback: now=" << now.us() 
                     << " us, last_update_time=" << last_update_time_.us() << " us";
  } else {
    RTC_LOG(LS_INFO) << "Prague UpdateEcnFeedback: now=" << now.us() 
                     << " us, last_update_time=MinusInfinity (not initialized)";
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
  while (!ecn_window_.empty()) {
    TimeDelta age = now - ecn_window_.front().time;
    
    // Log values before age calculation check
    RTC_LOG(LS_INFO) << "Prague ECN window cleanup: now=" << now.us() 
                     << " us, front_time=" << ecn_window_.front().time.us() 
                     << " us, age=" << (age.IsFinite() ? std::to_string(age.ms()) + "ms" : "non-finite");
    
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
  
  // Log timestamp values before calculations - with safety check for MinusInfinity
  if (last_update_time_.IsFinite()) {
    RTC_LOG(LS_INFO) << "Prague GetTargetRate: now=" << now.us() 
                     << " us, last_update_time=" << last_update_time_.us() << " us";
  } else {
    RTC_LOG(LS_INFO) << "Prague GetTargetRate: now=" << now.us() 
                     << " us, last_update_time=MinusInfinity (not initialized)";
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
  
  // Log values before calculation to debug potential unit_base.h assertion
  RTC_LOG(LS_INFO) << "Prague controller rate calculation: cwnd=" << cwnd.bytes() 
                   << " bytes, cwnd_bits=" << cwnd_bits 
                   << ", rtt_ms=" << rtt_ms;
  
  // Check for potential overflow
  if (cwnd_bits > (std::numeric_limits<int64_t>::max() / 1000)) {
    RTC_LOG(LS_WARNING) << "Potential overflow in rate calculation, using default rate";
    return DataRate::KilobitsPerSec(300);
  }
  
  int64_t rate_bps = (cwnd_bits * 1000) / rtt_ms;
  
  // Log the calculated rate before creating DataRate object
  RTC_LOG(LS_INFO) << "Prague controller calculated rate_bps=" << rate_bps;
  
  // Ensure the result is positive and reasonable
  if (rate_bps <= 0) {
    RTC_LOG(LS_WARNING) << "Invalid rate calculation result: " << rate_bps << " bps, using default";
    return DataRate::KilobitsPerSec(300);
  }
  
  DataRate base_rate = DataRate::BitsPerSec(rate_bps);
  
  // Log the created DataRate value and its internal representation
  RTC_LOG(LS_INFO) << "Prague controller created DataRate: bps=" << base_rate.bps()
                   << ", IsFinite=" << (base_rate.IsFinite() ? "true" : "false");
  
  // Apply Prague's scalable congestion control formula
  if (ecn_ce_ratio_ > 0) {
    // If we have congestion signals, apply the Prague reduction
    double reduction_factor = 1.0 - (alpha_.Get() * ecn_ce_ratio_);
    reduction_factor = std::max(reduction_factor, beta_.Get());
    
    // Log values before applying reduction
    RTC_LOG(LS_INFO) << "Prague applying congestion reduction: base_rate_bps=" << base_rate.bps()
                     << ", reduction_factor=" << reduction_factor
                     << ", alpha=" << alpha_.Get()
                     << ", ecn_ce_ratio=" << ecn_ce_ratio_
                     << ", base_rate.IsFinite()=" << (base_rate.IsFinite() ? "true" : "false");
    
    // Log before the multiplication that might trigger unit_base.h assertion
    RTC_LOG(LS_INFO) << "Prague PRE-REDUCTION-RATE-MULTIPLY: base_rate.bps()=" << base_rate.bps()
                     << ", reduction_factor=" << reduction_factor;
    
    DataRate reduced_rate = base_rate * reduction_factor;
    
    RTC_LOG(LS_INFO) << "Prague POST-REDUCTION-RATE-MULTIPLY: reduced_rate.bps()=" << reduced_rate.bps()
                     << ", IsFinite=" << (reduced_rate.IsFinite() ? "true" : "false");
    
    return reduced_rate;
  }
  
  // If no congestion, increase additively based on RTT
  // Add safety check for uninitialized last_update_time
  if (!last_update_time_.IsFinite()) {
    RTC_LOG(LS_INFO) << "Prague controller: last_update_time not initialized, returning base rate";
    return base_rate;
  }
  
  TimeDelta time_since_update = now - last_update_time_;
  
  // Add safety check for time calculation
  if (!time_since_update.IsFinite()) {
    RTC_LOG(LS_WARNING) << "Invalid time_since_update calculation in Prague controller";
    return base_rate;
  }
  
  if (time_since_update > TimeDelta::Zero()) {
    // Additive increase proportional to 1/RTT (RTT-fairness)
    double rtt_seconds = current_rtt.seconds();
    
    // Log values before calculations
    RTC_LOG(LS_INFO) << "Prague controller time calculation: time_since_update=" 
                     << time_since_update.ms() << "ms, rtt_seconds=" << rtt_seconds;
    
    if (rtt_seconds <= 0) {
      RTC_LOG(LS_WARNING) << "Invalid RTT seconds: " << rtt_seconds;
      return base_rate;
    }
    
    double increase_factor = 1.0 + 
        (time_since_update.ms() / (1000.0 * rtt_seconds));
    
    // Log the calculated increase factor
    RTC_LOG(LS_INFO) << "Prague controller increase_factor=" << increase_factor;
    
    // Ensure increase factor is reasonable (prevent extreme values)
    if (increase_factor < 0.5 || increase_factor > 2.0) {
      RTC_LOG(LS_WARNING) << "Extreme increase factor: " << increase_factor << ", clamping";
      increase_factor = std::clamp(increase_factor, 0.5, 2.0);
    }
    
    // Log before applying the increase factor
    RTC_LOG(LS_INFO) << "Prague controller applying increase: base_rate=" 
                     << base_rate.bps() << " bps, factor=" << increase_factor
                     << ", base_rate.IsFinite()=" << (base_rate.IsFinite() ? "true" : "false");
    
    // Log before another multiplication that might trigger unit_base.h assertion
    RTC_LOG(LS_INFO) << "Prague PRE-INCREASE-RATE-MULTIPLY: base_rate.bps()=" << base_rate.bps()
                     << ", increase_factor=" << increase_factor;
    
    DataRate increased_rate = base_rate * increase_factor;
    
    RTC_LOG(LS_INFO) << "Prague POST-INCREASE-RATE-MULTIPLY: increased_rate.bps()=" << increased_rate.bps()
                     << ", IsFinite=" << (increased_rate.IsFinite() ? "true" : "false");
    
    return increased_rate;
  }
  
  // Log the final base_rate before returning
  RTC_LOG(LS_INFO) << "Prague returning base_rate_bps=" << base_rate.bps()
                   << ", IsFinite=" << (base_rate.IsFinite() ? "true" : "false");
  
  return base_rate;
}

bool L4SPragueController::IsActive() const {
  return active_;
}

DataSize L4SPragueController::CalculateCongestionWindow() const {
  if (!rtt_ || !min_rtt_estimate_) {
    RTC_LOG(LS_INFO) << "Prague CalculateCongestionWindow: using default 2 packets (no RTT data)";
    return DataSize::Bytes(1500 * 2);  // Default: 2 packets
  }
  
  // Use the minimum RTT for BDP calculation to avoid bloating the window
  TimeDelta base_rtt = *min_rtt_estimate_;
  
  // Log RTT values before calculations
  RTC_LOG(LS_INFO) << "Prague CalculateCongestionWindow: base_rtt_ms=" << base_rtt.ms()
                   << ", current_rtt_ms=" << (rtt_ ? rtt_->ms() : -1);
  
  // Calculate base congestion window (BDP)
  // Base it on a reasonable link capacity for interactive media
  DataRate base_rate = DataRate::KilobitsPerSec(1000);  // 1 Mbps base
  
  // Log before multiplication that might trigger unit_base.h assertion
  RTC_LOG(LS_INFO) << "Prague BDP calculation PRE-MULTIPLY: base_rate.bps()=" << base_rate.bps()
                   << ", base_rtt.us()=" << base_rtt.us()
                   << ", base_rtt.ms()=" << base_rtt.ms()
                   << ", base_rate.IsFinite()=" << base_rate.IsFinite()
                   << ", base_rtt.IsFinite()=" << base_rtt.IsFinite();
  
  DataSize bdp = base_rate * base_rtt;
  
  // Log after potential problematic multiplication
  RTC_LOG(LS_INFO) << "Prague BDP calculation POST-MULTIPLY: bdp.bytes()=" << bdp.bytes()
                   << ", bdp.IsFinite()=" << (bdp.IsFinite() ? "true" : "false");
  
  // Log intermediate calculations
  RTC_LOG(LS_INFO) << "Prague BDP calculation: base_rate_bps=" << base_rate.bps()
                   << ", bdp_bytes=" << bdp.bytes()
                   << ", init_cwnd_factor=" << init_cwnd_.Get();
  
  // Apply the initial window multiplier
  // Log before another potential problematic multiplication
  RTC_LOG(LS_INFO) << "Prague before init_cwnd multiplication: bdp.bytes()=" << bdp.bytes()
                   << ", init_cwnd=" << init_cwnd_.Get()
                   << ", bdp.IsFinite()=" << (bdp.IsFinite() ? "true" : "false");
  
  DataSize cwnd = bdp * init_cwnd_;
  
  RTC_LOG(LS_INFO) << "Prague after init_cwnd multiplication: cwnd.bytes()=" << cwnd.bytes()
                   << ", cwnd.IsFinite()=" << (cwnd.IsFinite() ? "true" : "false");
  
  // Apply reduction based on CE marking ratio
  if (ecn_ce_ratio_ > 0) {
    double reduction = std::max(1.0 - ecn_ce_ratio_, beta_.Get());
    RTC_LOG(LS_INFO) << "Prague applying congestion window reduction: ratio=" << ecn_ce_ratio_
                     << ", reduction_factor=" << reduction
                     << ", cwnd_before=" << cwnd.bytes();
    
    // Log before another multiplication that might be problematic
    RTC_LOG(LS_INFO) << "Prague PRE-REDUCTION-MULTIPLY: cwnd.bytes()=" << cwnd.bytes()
                     << ", reduction=" << reduction
                     << ", cwnd.IsFinite()=" << (cwnd.IsFinite() ? "true" : "false");
    
    cwnd = cwnd * reduction;
    
    RTC_LOG(LS_INFO) << "Prague POST-REDUCTION-MULTIPLY: cwnd.bytes()=" << cwnd.bytes()
                     << ", cwnd.IsFinite()=" << (cwnd.IsFinite() ? "true" : "false");
  }
  
  // Ensure minimum congestion window (2 packets)
  RTC_LOG(LS_INFO) << "Prague before std::max: cwnd.bytes()=" << cwnd.bytes()
                   << ", min_cwnd=3000 bytes";
  
  DataSize final_cwnd = std::max(cwnd, DataSize::Bytes(1500 * 2));
  
  RTC_LOG(LS_INFO) << "Prague final cwnd.bytes()=" << final_cwnd.bytes()
                   << ", final_cwnd.IsFinite()=" << (final_cwnd.IsFinite() ? "true" : "false");
  
  return final_cwnd;
}

}  // namespace webrtc