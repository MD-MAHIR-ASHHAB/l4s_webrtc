#include "modules/congestion_controller/l4s/l4s_network_controller.h"

#include <algorithm>
#include <memory>
#include <numeric>
#include <utility>

#include "absl/strings/match.h"
#include "api/field_trials_view.h"
#include "api/rtc_event_log/rtc_event_log.h"
#include "api/transport/bandwidth_usage.h"
#include "api/transport/network_types.h"
#include "api/units/data_rate.h"
#include "api/units/data_size.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "logging/rtc_event_log/events/rtc_event_probe_cluster_created.h"
#include "rtc_base/checks.h"
#include "rtc_base/experiments/field_trial_parser.h"
#include "rtc_base/logging.h"
#include "system_wrappers/include/metrics.h"

// Metrics collection
#include "api/test/metrics/global_metrics_logger_and_exporter.h"
#include "api/test/metrics/metrics_logger.h"

// Real probe infrastructure
#include "modules/congestion_controller/goog_cc/probe_bitrate_estimator.h"

namespace webrtc {

// =============================================================================
// PragueCapacityEstimator Implementation
// =============================================================================

PragueCapacityEstimator::PragueCapacityEstimator(DataRate starting_rate, DataRate min_rate, DataRate max_rate)
    : congestion_based_estimate_(starting_rate),
      min_target_rate_(min_rate),
      max_target_rate_(max_rate),
      current_rtt_(TimeDelta::Millis(50)),
      last_update_time_(Timestamp::MinusInfinity()),
      last_congestion_signal_(Timestamp::MinusInfinity()),
      last_ecn_feedback_(Timestamp::MinusInfinity()),
      direction_flag_(1),
      non_ce_packet_count_(0),
      discovery_mode_active_(true),
      first_ce_mark_detected_(false) {
  
  // Clamp initial estimate to bounds
  if (congestion_based_estimate_ < min_target_rate_) {
    congestion_based_estimate_ = min_target_rate_;
  }
  if (congestion_based_estimate_ > max_target_rate_) {
    congestion_based_estimate_ = max_target_rate_;
  }
}

PragueCapacityEstimator::~PragueCapacityEstimator() = default;

void webrtc::PragueCapacityEstimator::UpdateFromCongestionSignal(DataRate current_rate, double ce_ratio, Timestamp current_time) {
  constexpr int kDefaultMssBytes = 1440;  // Typical Ethernet MSS
  
  TimeDelta rtt = current_rtt_.IsFinite() && !current_rtt_.IsZero() ? current_rtt_ : TimeDelta::Millis(1);
  double rtt_seconds = rtt.seconds<double>();
  if (rtt_seconds <= 0.0) {
    rtt_seconds = 0.001;  // Fallback RTT (1ms for VM testbed)
  }

  // Prague DCTCP-style rate adaptation with state machine (RFC 9330)
  if (ce_ratio > 0.0) {  // CE-marked packets detected
    // Reset non-CE packet count when we see CE marks
    non_ce_packet_count_ = 0;
    
    // Immediately exit discovery mode on first CE mark (highest priority)
    if (discovery_mode_active_ && !first_ce_mark_detected_) {
      discovery_mode_active_ = false;
      first_ce_mark_detected_ = true;
      RTC_LOG(LS_INFO) << "Prague: Exiting discovery mode - first CE mark detected (ce_ratio=" 
                       << ce_ratio << ")";
    }
    
    // Always update alpha, even in reduction mode (proper DCTCP behavior)
    constexpr double g = 1.0 / 16.0;  // RFC 9330 standard gain
    alpha_ = (1.0 - g) * alpha_ + g * ce_ratio;
    
    // Only switch to reduction mode if we're in increasing mode
    if (direction_flag_ == 1) {

      // Switch to reduction mode
      direction_flag_ = -1;

      // Proportional decrease (much gentler than 50% reduction)
      double reduction_factor = 1.0 - alpha_ / 2.0;
      DataRate reduced = std::max(current_rate * reduction_factor, min_target_rate_);
      // Enforce absolute minimum of 20 kbps to prevent pacer crashes
      reduced = std::max(reduced, DataRate::KilobitsPerSec(20));
      congestion_based_estimate_ = reduced;
      last_md_time_ = current_time;  // Record when this MD fired
      
      RTC_LOG(LS_INFO) << "Prague: Switched to reduction mode (alpha=" << alpha_
                       << ", ce_ratio=" << ce_ratio 
                       << ", reduction_factor=" << reduction_factor
                       << "), new rate=" << congestion_based_estimate_.bps() << " bps";
    } else {
      // Already in reduction mode. RFC 9330 §4.3: MD is applied at most once
      // per RTT.  Alpha keeps accumulating so sustained congestion is captured,
      // but the rate is only reduced again when a full RTT has elapsed since
      // the last reduction.  This prevents the cascade where 10-20 CE batches
      // each landing within one RTT collectively push the rate to the floor.
      TimeDelta gate_rtt = current_rtt_.IsFinite() && !current_rtt_.IsZero()
                               ? current_rtt_
                               : TimeDelta::Millis(100);
      bool gate_open = last_md_time_.IsInfinite() ||
                       (current_time - last_md_time_ >= gate_rtt);
      if (gate_open) {
        double additional_reduction = 1.0 - alpha_ / 4.0;  // Gentler than initial
        DataRate further_reduced = std::max(
            congestion_based_estimate_ * additional_reduction, min_target_rate_);
        further_reduced = std::max(further_reduced, DataRate::KilobitsPerSec(20));
        congestion_based_estimate_ = further_reduced;
        last_md_time_ = current_time;
        RTC_LOG(LS_VERBOSE) << "Prague: Additional reduction (once-per-RTT gate open, alpha="
                         << alpha_ << ", ce_ratio=" << ce_ratio
                         << ", additional_reduction=" << additional_reduction
                         << "), new rate=" << congestion_based_estimate_.bps() << " bps";
      } else {
        RTC_LOG(LS_VERBOSE) << "Prague: MD gate closed - last MD "
                         << (current_time - last_md_time_).ms()
                         << " ms ago (rtt=" << gate_rtt.ms()
                         << " ms), alpha accumulating to " << alpha_;
      }
    }
    
    // Update congestion signal timestamp
    last_congestion_signal_ = current_time;
                     
  } else {  // No CE marks in this batch
    // Increment non-CE packet count
    non_ce_packet_count_++;
    
    // Check if we should switch from reduction mode to additive mode
    if (direction_flag_ == -1 && non_ce_packet_count_ >= kNonCeThreshold) {
      direction_flag_ = 1;
      non_ce_packet_count_ = 0;  // Reset counter
      RTC_LOG(LS_INFO) << "Prague: Switched to additive mode after " << kNonCeThreshold 
                       << " consecutive non-CE packets";
    }
    
    // Only perform additive increase if we're in increasing mode (flag = 1)
    if (direction_flag_ == 1) {  // Always increase when in additive mode
    if (!additive_hold_until_.IsInfinite() && current_time < additive_hold_until_) {
      RTC_LOG(LS_VERBOSE) << "Prague: Additive increase paused during probe hold window";
      last_update_time_ = current_time;
      return;
    }
    
    // Prague DCTCP additive increase: +1 MSS per RTT period
    // This is the fundamental L4S congestion control behavior
    int64_t bits_per_rtt = kDefaultMssBytes * 8;  // 1440 * 8 = 11520 bits
    
    // Calculate the theoretical AI rate: 1 MSS worth of extra bits per RTT period
    int64_t theoretical_ai_bps = static_cast<int64_t>(bits_per_rtt / rtt_seconds);
    
    int64_t ai_step_bps;
    
    // Discovery Mode: Fast startup for rates below 5 Mbps when no congestion detected
    bool in_discovery_mode = discovery_mode_active_ && 
                            !first_ce_mark_detected_ && 
                            current_rate.bps() < 5000000;  // 5 Mbps threshold
    
    if (in_discovery_mode) {
      // Moderate aggressive growth: 5x normal AI step
      ai_step_bps = theoretical_ai_bps * 5;
      
      // Safety cap for discovery mode
      ai_step_bps = std::min(ai_step_bps, static_cast<int64_t>(2000000));  // Cap at 2 Mbps/RTT
      
      RTC_LOG(LS_VERBOSE) << "Prague: Discovery mode active - aggressive AI step: " 
                       << ai_step_bps << " bps (5x theoretical: " << theoretical_ai_bps << " bps)";
    } else {
      // Normal Prague mode: Context-aware AI step calculation
      ai_step_bps = CalculateContextAwareAiStep(theoretical_ai_bps, current_rate, current_time);
      
      // Note: Discovery mode exit is now handled by L4SNetworkController
      // based on probe-Prague convergence or fallback threshold
    }
    
    // Safety check to ensure valid step size
    if (ai_step_bps <= 0 || ai_step_bps > 1000000000) {  // Cap at 1 Gbps for safety
      RTC_LOG(LS_WARNING) << "Prague: Invalid AI step " << ai_step_bps << " bps, using fallback";
      ai_step_bps = std::min(theoretical_ai_bps, static_cast<int64_t>(current_rate.bps() * 0.1));
    }
    
    DataRate increased = current_rate + DataRate::BitsPerSec(ai_step_bps);
    
    // Don't exceed maximum rate
    if (max_target_rate_ > DataRate::Zero()) {
        increased = std::min(increased, max_target_rate_);
    }
    
    // Apply probe constraint during discovery mode
    if (discovery_mode_active_ && probe_constraint_ > DataRate::Zero() && probe_constraint_confidence_ > 0.7) {
      DataRate max_allowed = probe_constraint_ * 0.95;  // 95% of probe estimate
      if (increased > max_allowed) {
        increased = max_allowed;
        RTC_LOG(LS_VERBOSE) << "Prague: Rate limited by probe constraint to " << increased.bps() 
                         << " bps (probe: " << probe_constraint_.bps() << " bps)";
      }
    }
    
    congestion_based_estimate_ = increased;
    
    RTC_LOG(LS_VERBOSE) << "Prague: DCTCP additive increase (+1.0 MSS/RTT, rtt=" << rtt.ms() << " ms, "
                     << "step=" << ai_step_bps << " bps), new rate=" 
                     << congestion_based_estimate_.bps() << " bps (input rate: " << current_rate.bps() << "), non_ce_count=" << non_ce_packet_count_;
    } else if (direction_flag_ == -1) {
      RTC_LOG(LS_VERBOSE) << "Prague: Skipping AI, in reduction mode (need " 
                          << (kNonCeThreshold - non_ce_packet_count_) 
                          << " more non-CE packets to switch)";
    }
  }
  
  last_update_time_ = current_time;
}

void webrtc::PragueCapacityEstimator::UpdateEcnActivity(Timestamp current_time) {
  // Track any ECN activity (ECT or CE packets) to maintain confidence
  last_ecn_feedback_ = current_time;
}

void webrtc::PragueCapacityEstimator::UpdateFromRtt(TimeDelta rtt) {
  if (rtt.IsFinite() && !rtt.IsZero()) {
    current_rtt_ = rtt;
  }
}

void webrtc::PragueCapacityEstimator::OnPacketLoss(DataRate current_rate, Timestamp current_time) {
  // Multiplicative decrease for packet loss (fallback mechanism)
  DataRate reduced = std::max(current_rate * 0.5, min_target_rate_);
  // Enforce absolute minimum of 20 kbps to prevent pacer crashes
  reduced = std::max(reduced, DataRate::KilobitsPerSec(20));
  congestion_based_estimate_ = reduced;
  
  // Switch to reduction mode and reset non-CE counter
  direction_flag_ = -1;
  non_ce_packet_count_ = 0;
  
  RTC_LOG(LS_WARNING) << "Prague: Packet loss detected, halving estimate to "
                      << congestion_based_estimate_.bps() << " bps, switched to reduction mode";
  
  last_update_time_ = current_time;
}

void webrtc::PragueCapacityEstimator::OnTimeUpdate(Timestamp current_time) {
  if (last_update_time_.IsInfinite()) {
    last_update_time_ = current_time;
    return;
  }
  
  TimeDelta elapsed = current_time - last_update_time_;
  if (elapsed >= kDecayInterval) {
    // Gradually decay estimates if not reinforced
    congestion_based_estimate_ = std::max(congestion_based_estimate_ * 0.95, min_target_rate_);
    // Enforce absolute minimum of 20 kbps to prevent pacer crashes
    congestion_based_estimate_ = std::max(congestion_based_estimate_, DataRate::KilobitsPerSec(20));
    last_update_time_ = current_time;
  }
  
  // Re-enable discovery mode after 30 seconds without congestion signals
  if (first_ce_mark_detected_ && !discovery_mode_active_) {
    TimeDelta since_congestion = current_time - last_congestion_signal_;
    if (since_congestion > TimeDelta::Seconds(30)) {
      discovery_mode_active_ = true;
      first_ce_mark_detected_ = false;  // Reset CE flag to allow rediscovery
      RTC_LOG(LS_INFO) << "Prague: Re-enabling discovery mode after 30s without congestion (CE flag reset)";
    }
  }
}

webrtc::DataRate webrtc::PragueCapacityEstimator::GetCurrentEstimate() const {
  return congestion_based_estimate_;
}

void webrtc::PragueCapacityEstimator::SetCurrentEstimate(DataRate rate) {
  congestion_based_estimate_ = std::max(rate, min_target_rate_);
  congestion_based_estimate_ = std::max(congestion_based_estimate_,
                                        DataRate::KilobitsPerSec(20));
}

double webrtc::PragueCapacityEstimator::GetConfidence(Timestamp now) const {
  // Check for any ECN activity (ECT or CE packets)
  if (last_ecn_feedback_.IsInfinite()) {
    return 0.3;  // Low confidence without any ECN feedback
  }
  
  TimeDelta since_ecn_activity = now - last_ecn_feedback_;
  if (since_ecn_activity < TimeDelta::Seconds(2)) {
    return 0.9;  // Very confident with recent ECN activity
  } else if (since_ecn_activity < TimeDelta::Seconds(5)) {
    return 0.7;  // Moderately confident
  }
  return 0.4;  // Lower confidence with stale ECN feedback
}

int64_t webrtc::PragueCapacityEstimator::CalculateContextAwareAiStep(int64_t theoretical_ai_bps, DataRate current_rate, Timestamp current_time) {
  // Context-aware AI step calculation that adapts to network conditions
  
  // Safety checks for input parameters
  if (theoretical_ai_bps <= 0 || !current_rate.IsFinite() || current_time.IsInfinite()) {
    RTC_LOG(LS_WARNING) << "Prague: Invalid input parameters to CalculateContextAwareAiStep";
    return 100000;  // Fallback: 100 Kbps step
  }
  
  // 1. Base multiplier starts at 1.0 (full DCTCP behavior)
  double context_multiplier = 1.0;
  
  // 2. Consider time since last congestion signal
  TimeDelta since_congestion = current_time - last_congestion_signal_;
  if (!last_congestion_signal_.IsInfinite()) {
    if (since_congestion < TimeDelta::Seconds(1)) {
      // Very recent congestion - be more conservative
      context_multiplier *= 0.3;
    } else if (since_congestion < TimeDelta::Seconds(5)) {
      // Recent congestion - moderate caution
      context_multiplier *= 0.6;
    } else if (since_congestion > TimeDelta::Seconds(10)) {
      // No recent congestion - can be more aggressive
      context_multiplier *= 1.5;
    }
    // Between 5-10 seconds: use base multiplier (1.0)
  }
  
  // 3. Consider current alpha value (congestion severity)
  if (alpha_ > 0.5) {
    // High congestion memory - be very conservative
    context_multiplier *= 0.2;
  } else if (alpha_ > 0.1) {
    // Moderate congestion memory - be somewhat conservative
    context_multiplier *= 0.5;
  } else if (alpha_ < 0.01) {
    // Very low congestion memory - can be more aggressive
    context_multiplier *= 1.3;
  }
  
  // 4. Consider rate magnitude (avoid explosive growth at high rates)
  int64_t current_bps = current_rate.bps();
  if (current_bps > 50000000) {  // > 50 Mbps
    // At very high rates, cap growth to prevent network overload
    context_multiplier *= 0.1;
  } else if (current_bps > 10000000) {  // > 10 Mbps
    // At high rates, be very conservative
    context_multiplier *= 0.2;
  } else if (current_bps > 5000000) {  // > 5 Mbps
    // At medium-high rates, be conservative
    context_multiplier *= 0.3;
  } else if (current_bps > 1000000) {  // > 1 Mbps
    // At medium rates, moderate increases
    context_multiplier *= 0.6;
  } else if (current_bps < 300000) {  // < 300 Kbps
    // At very low rates, allow more aggressive growth
    context_multiplier *= 1.5;
  }
  
  // 5. Consider direction flag stability
  if (direction_flag_ == 1 && non_ce_packet_count_ > kNonCeThreshold * 2) {
    // Been in additive mode for a while - can be more aggressive
    context_multiplier *= 1.2;
  }
  
  // 6. Apply RTT-based scaling (better responsiveness for high RTT)
  double rtt_seconds = current_rtt_.IsFinite() ? current_rtt_.seconds<double>() : 0.05;
  if (rtt_seconds < 0.01) {  // < 10ms RTT - very low latency scenario
    // For very low RTT, dramatically reduce AI steps to prevent explosive growth
    context_multiplier *= 0.1;
    RTC_LOG(LS_VERBOSE) << "Prague: Very low RTT (" << (rtt_seconds * 1000) << " ms), applying aggressive dampening";
  } else if (rtt_seconds < 0.05) {  // < 50ms RTT - low latency
    // For low RTT, moderate reduction to prevent excessive growth
    context_multiplier *= 0.3;
  } else if (rtt_seconds > 0.1) {  // > 100ms RTT
    // High RTT networks need more aggressive AI to maintain fairness
    context_multiplier *= std::min(2.0, rtt_seconds / 0.05);  // Scale with RTT, cap at 2x
  }
  
  // 7. Calculate context-aware step
  int64_t context_ai_bps = static_cast<int64_t>(theoretical_ai_bps * context_multiplier);
  
  // 8. Apply reasonable bounds to prevent pathological behavior
  int64_t min_step_bps = theoretical_ai_bps / 20;  // At least 5% of DCTCP standard
  
  // Much more aggressive rate-based capping for low RTT scenarios
  int64_t rate_based_max_step = std::max(
      static_cast<int64_t>(current_bps * 0.1),   // 10% of current rate
      static_cast<int64_t>(100000)               // Minimum 100 Kbps step
  );
  
  int64_t max_step_bps = std::min(
      theoretical_ai_bps * 2,     // At most 2x DCTCP standard
      rate_based_max_step         // But respect rate-based limit
  );
  
  context_ai_bps = std::max(min_step_bps, std::min(context_ai_bps, max_step_bps));
  
  // Final safety check to ensure result is valid
  if (context_ai_bps <= 0 || context_ai_bps > 1000000000 || !std::isfinite(context_ai_bps)) {
    RTC_LOG(LS_WARNING) << "Prague: Invalid context AI step calculated: " << context_ai_bps;
    context_ai_bps = 100000;  // Fallback: 100 Kbps
  }
  
  // Log the decision for debugging
  RTC_LOG(LS_VERBOSE) << "Prague: Context-aware AI calculation - "
                      << "theoretical=" << theoretical_ai_bps << " bps, "
                      << "multiplier=" << context_multiplier << ", "
                      << "context_step=" << context_ai_bps << " bps, "
                      << "alpha=" << alpha_ << ", "
                      << "since_congestion=" << (last_congestion_signal_.IsInfinite() ? -1.0 : since_congestion.ms<double>()) << " ms, "
                      << "current_rate=" << current_bps << " bps";
  
  return context_ai_bps;
}

void webrtc::PragueCapacityEstimator::SetProbeConstraint(DataRate probe_estimate, double probe_confidence) {
  probe_constraint_ = probe_estimate;
  probe_constraint_confidence_ = probe_confidence;
  
  RTC_LOG(LS_VERBOSE) << "Prague: Setting probe constraint to " << probe_estimate.bps() 
                   << " bps with confidence " << probe_confidence;
}

void webrtc::PragueCapacityEstimator::ClearProbeConstraint() {
  probe_constraint_ = DataRate::Zero();
  probe_constraint_confidence_ = 0.0;
  
  RTC_LOG(LS_VERBOSE) << "Prague: Cleared probe constraint";
}

void webrtc::PragueCapacityEstimator::SetAdditiveHoldUntil(Timestamp hold_until) {
  additive_hold_until_ = hold_until;
}

void webrtc::PragueCapacityEstimator::ExitDiscoveryMode(const std::string& reason) {
  if (discovery_mode_active_) {
    discovery_mode_active_ = false;
    ClearProbeConstraint();  // Clear any probe constraints when exiting discovery
    RTC_LOG(LS_INFO) << "Prague: Exiting discovery mode - " << reason;
  }
}

// =============================================================================
// L4SBandwidthFusion Implementation
// =============================================================================

webrtc::L4SBandwidthFusion::L4SBandwidthFusion(const L4SControllerConfig& config) : config_(config) {}

webrtc::L4SBandwidthFusion::~L4SBandwidthFusion() = default;

void webrtc::L4SBandwidthFusion::UpdateEcnEstimate(DataRate estimate, double confidence, Timestamp now) {
  // Enforce absolute minimum of 20 kbps to prevent pacer crashes
  DataRate clamped_estimate = std::max(estimate, DataRate::KilobitsPerSec(20));
  RTC_LOG(LS_VERBOSE) << "L4S: Updating ECN estimate to " << clamped_estimate.bps() << " bps with confidence " << confidence;
  sources_.ecn_estimate = clamped_estimate;
  sources_.ecn_confidence = confidence;
  sources_.last_ecn_update = now;
}

void webrtc::L4SBandwidthFusion::UpdateDelayEstimate(DataRate estimate, double confidence, Timestamp now) {
  RTC_LOG(LS_VERBOSE) << "L4S: Updating delay estimate to " << estimate.bps() << " bps with confidence " << confidence;
  sources_.delay_estimate = estimate;
  sources_.delay_confidence = confidence;
  sources_.last_delay_update = now;
}

void webrtc::L4SBandwidthFusion::UpdateProbeEstimate(DataRate estimate, double confidence, Timestamp now) {
  RTC_LOG(LS_VERBOSE) << "L4S: Updating probe estimate to " << estimate.bps() << " bps with confidence " << confidence;
  sources_.probe_estimate = estimate;
  sources_.probe_confidence = confidence;
  sources_.last_probe_update = now;
}

void webrtc::L4SBandwidthFusion::UpdateAckedEstimate(DataRate estimate, double confidence, Timestamp now) {
  RTC_LOG(LS_VERBOSE) << "L4S: Updating acked estimate to " << estimate.bps() << " bps with confidence " << confidence;
  sources_.acked_estimate = estimate;
  sources_.acked_confidence = confidence;
  sources_.last_acked_update = now;
}

webrtc::DataRate webrtc::L4SBandwidthFusion::GetFusedEstimate(Timestamp now) const {
  return GetFusedEstimateWithMode(now, false, false);
}

webrtc::DataRate webrtc::L4SBandwidthFusion::GetFusedEstimateWithMode(Timestamp now, bool discovery_mode, bool recovery_mode) const {
  // L4S Fusion: Prague ECN provides congestion control authority,
  // other estimators provide capacity discovery insights
  
  // During discovery or recovery mode, use probe-weighted fusion
  if (discovery_mode || recovery_mode) {
    return GetDiscoveryModeFusedEstimate(now, recovery_mode);
  }
  
  // 1. Prague ECN estimate has highest priority for congestion control
  // When Prague detects congestion, it overrides other estimates
  if (sources_.ecn_confidence > config_.ecn_confidence_threshold && 
      IsRecentlyUpdated(sources_.last_ecn_update, now)) {
    RTC_LOG(LS_VERBOSE) << "L4S: Using Prague ECN estimate: " << sources_.ecn_estimate.bps() << " bps";
    return sources_.ecn_estimate;
  }
  
  // 2. When no recent congestion, use probe results as capacity upper bound
  if (sources_.probe_confidence > config_.probe_confidence_threshold && 
      IsRecentlyUpdated(sources_.last_probe_update, now)) {
    DataRate capacity_estimate = ValidateWithOtherSources(sources_.probe_estimate, sources_);
    
    // But don't exceed Prague's current estimate if it's lower (recent congestion)
    if (sources_.ecn_confidence > 0.3) {
      capacity_estimate = std::min(capacity_estimate, sources_.ecn_estimate * 1.1);
    }
    
    RTC_LOG(LS_VERBOSE) << "L4S: Using probe-based capacity: " << capacity_estimate.bps() << " bps";
    return capacity_estimate;
  }
  
  // 3. Fallback to conservative combination of delay and acked estimates
  double total_weight = sources_.delay_confidence + sources_.acked_confidence;
  if (total_weight > 0.1) {
    DataRate weighted_estimate = 
        (sources_.delay_estimate * sources_.delay_confidence + 
         sources_.acked_estimate * sources_.acked_confidence) / total_weight;
    
    // Always respect Prague's congestion authority
    if (sources_.ecn_confidence > 0.3) {
      weighted_estimate = std::min(weighted_estimate, sources_.ecn_estimate);
    }
    
    RTC_LOG(LS_VERBOSE) << "L4S: Using weighted delay+acked estimate: " << weighted_estimate.bps() << " bps";
    return weighted_estimate;
  }
  
  // 4. Final fallback to most confident single estimate
  RTC_LOG(LS_VERBOSE) << "L4S: Using fallback estimate";
  return GetMostConfidentEstimate(now);
}

webrtc::DataRate webrtc::L4SBandwidthFusion::GetMostConfidentEstimate(Timestamp now) const {
  DataRate best_estimate = DataRate::KilobitsPerSec(300);  // Fallback
  double best_confidence = 0.0;
  
  if (sources_.ecn_confidence > best_confidence && IsRecentlyUpdated(sources_.last_ecn_update, now)) {
    best_estimate = sources_.ecn_estimate;
    best_confidence = sources_.ecn_confidence;
  }
  
  if (sources_.delay_confidence > best_confidence && IsRecentlyUpdated(sources_.last_delay_update, now)) {
    best_estimate = sources_.delay_estimate;
    best_confidence = sources_.delay_confidence;
  }
  
  if (sources_.probe_confidence > best_confidence && IsRecentlyUpdated(sources_.last_probe_update, now)) {
    best_estimate = sources_.probe_estimate;
    best_confidence = sources_.probe_confidence;
  }
  
  if (sources_.acked_confidence > best_confidence && IsRecentlyUpdated(sources_.last_acked_update, now)) {
    best_estimate = sources_.acked_estimate;
    best_confidence = sources_.acked_confidence;
  }
  
  return best_estimate;
}

webrtc::DataRate webrtc::L4SBandwidthFusion::ValidateWithOtherSources(DataRate primary_estimate, const BandwidthSources& sources) const {
  // Don't allow probe results that are dramatically higher than other estimates
  DataRate max_alternative = DataRate::Zero();
  
  if (sources.ecn_confidence > 0.3) {
    max_alternative = std::max(max_alternative, sources.ecn_estimate);
  }
  if (sources.delay_confidence > 0.3) {
    max_alternative = std::max(max_alternative, sources.delay_estimate);
  }
  if (sources.acked_confidence > 0.3) {
    max_alternative = std::max(max_alternative, sources.acked_estimate);
  }
  
  if (max_alternative > DataRate::Zero() && primary_estimate > max_alternative * 2.0) {
    // Probe result seems too optimistic, cap it
    return max_alternative * 1.5;
  }
  
  return primary_estimate;
}

webrtc::DataRate webrtc::L4SBandwidthFusion::GetDiscoveryModeFusedEstimate(Timestamp now, bool recovery_mode) const {
  // During discovery/recovery mode, give probe controller higher weight (80%)
  // and other estimators lower weight (20%)
  
  DataRate probe_weighted = DataRate::Zero();
  DataRate other_weighted = DataRate::Zero();
  double probe_weight = 0.0;
  double other_weight = 0.0;
  
  // Probe estimate weight is intentionally conservative for CE-sensitive AQMs.
  if (sources_.probe_confidence > 0.5 && IsRecentlyUpdated(sources_.last_probe_update, now)) {
    probe_weighted = sources_.probe_estimate;
    probe_weight = recovery_mode ? 0.50 : 0.45;
  }
  
  // Combine other estimates for remaining weight
  double total_other_confidence = 0.0;
  DataRate combined_other = DataRate::Zero();
  
  // Combine ECN and Acked estimates (delay and ALR removed for L4S simplicity)
  if (sources_.ecn_confidence > 0.3 && IsRecentlyUpdated(sources_.last_ecn_update, now)) {
    combined_other = combined_other + sources_.ecn_estimate * sources_.ecn_confidence;
    total_other_confidence += sources_.ecn_confidence;
  }
  
  if (sources_.acked_confidence > 0.3 && IsRecentlyUpdated(sources_.last_acked_update, now)) {
    combined_other = combined_other + sources_.acked_estimate * sources_.acked_confidence;
    total_other_confidence += sources_.acked_confidence;
  }
  
  if (total_other_confidence > 0.0) {
    other_weighted = combined_other / total_other_confidence;
    other_weight = 1.0 - probe_weight;
  }
  
  // Calculate weighted fusion
  DataRate fused_estimate;
  if (probe_weight > 0.0 && other_weight > 0.0) {
    fused_estimate = probe_weighted * probe_weight + other_weighted * other_weight;
    RTC_LOG(LS_VERBOSE) << "L4S: Discovery/recovery fusion - Probe: " << probe_weighted.bps() 
                     << " bps (" << (probe_weight * 100) << "%), Other: " << other_weighted.bps() 
                     << " bps (" << (other_weight * 100) << "%), Fused: " << fused_estimate.bps() << " bps";
  } else if (probe_weight > 0.0) {
    fused_estimate = probe_weighted;
    RTC_LOG(LS_VERBOSE) << "L4S: Discovery/recovery fusion - Using probe only: " << fused_estimate.bps() << " bps";
  } else if (other_weight > 0.0) {
    fused_estimate = other_weighted;
    RTC_LOG(LS_VERBOSE) << "L4S: Discovery/recovery fusion - Using other estimates: " << fused_estimate.bps() << " bps";
  } else {
    // Fallback to most confident estimate
    fused_estimate = GetMostConfidentEstimate(now);
    RTC_LOG(LS_VERBOSE) << "L4S: Discovery/recovery fusion fallback: " << fused_estimate.bps() << " bps";
  }
  
  return fused_estimate;
}

bool webrtc::L4SBandwidthFusion::IsRecentlyUpdated(Timestamp last_update, Timestamp now) const {
  // Check both timestamps for infinity before arithmetic to prevent crash
  if (last_update.IsInfinite() || now.IsInfinite()) {
    return false;
  }
  return (now - last_update) < TimeDelta::Seconds(10);
}

// =============================================================================
// L4SMetricsCollector Implementation
// =============================================================================

webrtc::L4SMetricsCollector::L4SMetricsCollector(
    test::MetricsLogger* logger,
    const std::string& test_case_name)
    : logger_(logger), test_case_name_(test_case_name) {
  RTC_CHECK(logger_);
  RTC_LOG(LS_INFO) << "L4SMetricsCollector initialized for test case: " << test_case_name_;
}

webrtc::L4SMetricsCollector::~L4SMetricsCollector() = default;

void webrtc::L4SMetricsCollector::LogBandwidthMetrics(
    Timestamp at_time,
    DataRate target_bitrate,
    DataRate actual_bitrate,
    std::optional<DataRate> acked_bitrate) {
  if (at_time - last_bandwidth_log_ < kBandwidthLogInterval) {
    return;
  }
  
  last_bandwidth_log_ = at_time;
  // Shared GCC/L4S contract: throughput for comparison is acked rate.
  DataRate rate_to_log = acked_bitrate.value_or(DataRate::Zero());
  UpdateThroughputStats(rate_to_log);

  logger_->LogSingleValueMetric("acked_rate_mbps", test_case_name_,
                                rate_to_log.bps() / 1e6,
                                webrtc::test::Unit::kUnitless,
                                webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});

  (void)target_bitrate;
  (void)actual_bitrate;
}

void webrtc::L4SMetricsCollector::LogDelayMetrics(Timestamp at_time, TimeDelta rtt, TimeDelta one_way_delay, TimeDelta jitter) {
  if (at_time - last_delay_log_ < kDelayLogInterval) {
    return;
  }
  
  last_delay_log_ = at_time;
  UpdateDelayStats(rtt, one_way_delay);
  
  logger_->LogSingleValueMetric("rtt_ms", test_case_name_, rtt.ms(),
                                webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  
  if (one_way_delay.IsFinite()) {
    logger_->LogSingleValueMetric("one_way_delay_ms", test_case_name_, one_way_delay.ms(),
                                  webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"timestamp_ms", std::to_string(at_time.ms())}});
  }
}

void webrtc::L4SMetricsCollector::LogLossMetrics(Timestamp at_time, double loss_fraction, int packets_lost) {
  if (at_time - last_loss_log_ < kLossLogInterval) {
    return;
  }
  
  last_loss_log_ = at_time;
  UpdateLossStats(loss_fraction);

  logger_->LogSingleValueMetric("packet_loss_fraction", test_case_name_, loss_fraction,
                                webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  
  logger_->LogSingleValueMetric("packets_lost_count", test_case_name_, packets_lost,
                                webrtc::test::Unit::kCount, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
}

void webrtc::L4SMetricsCollector::LogCongestionMetrics(Timestamp at_time, int ce_count, int ect_count, double congestion_ratio) {
  logger_->LogSingleValueMetric("congestion_ce_count", test_case_name_, ce_count, 
                                webrtc::test::Unit::kCount, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
  
  logger_->LogSingleValueMetric("congestion_ratio", test_case_name_, congestion_ratio, 
                                webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
}

// void L4SMetricsCollector::LogFusionMetrics(Timestamp at_time, const L4SBandwidthFusion::BandwidthSources& sources, DataRate fused_rate) {
//   logger_->LogSingleValueMetric("fusion_ecn_estimate_mbps", test_case_name_, sources.ecn_estimate.bps() / 1e6, 
//                                 webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
  
//   logger_->LogSingleValueMetric("fusion_delay_estimate_mbps", test_case_name_, sources.delay_estimate.bps() / 1e6, 
//                                 webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
  
//   logger_->LogSingleValueMetric("fusion_probe_estimate_mbps", test_case_name_, sources.probe_estimate.bps() / 1e6, 
//                                 webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
  
//   logger_->LogSingleValueMetric("fusion_fused_rate_mbps", test_case_name_, fused_rate.bps() / 1e6, 
//                                 webrtc::test::Unit::kKilobitsPerSecond, webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});
// }

void webrtc::L4SMetricsCollector::LogPeriodicSummary(Timestamp at_time) {
  if (at_time - last_summary_log_ < kSummaryLogInterval) {
    return;
  }
  
  last_summary_log_ = at_time;
  
  if (throughput_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric("acked_rate_avg_mbps", test_case_name_,
                                  throughput_stats_.GetAverage() / 1e6,
                                  webrtc::test::Unit::kUnitless,
                                  webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "acked_rate"}});
    logger_->LogSingleValueMetric("acked_rate_std_mbps", test_case_name_,
                                  throughput_stats_.GetStandardDeviation() / 1e6,
                                  webrtc::test::Unit::kUnitless,
                                  webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "std_dev"}, {"metric", "acked_rate"}});
  }

  if (rtt_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric("rtt_avg_ms", test_case_name_, rtt_stats_.GetAverage(),
                                  webrtc::test::Unit::kMilliseconds,
                                  webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "rtt"}});
    logger_->LogSingleValueMetric("rtt_std_ms", test_case_name_, rtt_stats_.GetStandardDeviation(),
                                  webrtc::test::Unit::kMilliseconds,
                                  webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "std_dev"}, {"metric", "rtt"}});
  }
  
  if (delay_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric("delay_avg_ms", test_case_name_, delay_stats_.GetAverage(),
                                  webrtc::test::Unit::kMilliseconds, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "delay"}});
    logger_->LogSingleValueMetric("delay_std_ms", test_case_name_, delay_stats_.GetStandardDeviation(),
                                  webrtc::test::Unit::kMilliseconds,
                                  webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "std_dev"}, {"metric", "delay"}});
  }

  if (loss_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric("packet_loss_avg_fraction", test_case_name_,
                                  loss_stats_.GetAverage(),
                                  webrtc::test::Unit::kUnitless, webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "average"}, {"metric", "packet_loss"}});
    logger_->LogSingleValueMetric("packet_loss_std_fraction", test_case_name_,
                                  loss_stats_.GetStandardDeviation(),
                                  webrtc::test::Unit::kUnitless,
                                  webrtc::test::ImprovementDirection::kSmallerIsBetter,
                                  {{"stat_type", "std_dev"}, {"metric", "packet_loss"}});
  }

  ExportToJsonFile("l4s_test_1.json");
}

void webrtc::L4SMetricsCollector::ExportToJsonFile(const std::string& filename) {
  if (!logger_) return;
  auto metrics = logger_->GetCollectedMetrics();
  FILE* f = fopen(filename.c_str(), "w");
  if (!f) return;
  
  fprintf(f, "[\n");
  for (size_t i = 0; i < metrics.size(); ++i) {
    const auto& m = metrics[i];
    fprintf(f, "  {\n");
    fprintf(f, "    \"name\": \"%s\",\n", m.name.c_str());
    fprintf(f, "    \"samples\": [");
    for (size_t j = 0; j < m.time_series.samples.size(); ++j) {
      const auto& s = m.time_series.samples[j];
      fprintf(f, "%s{\"timestamp_ms\": %lld, \"value\": %f}",
        (j > 0 ? ", " : ""), static_cast<long long>(s.timestamp.ms()), s.value);
    }
    fprintf(f, "]\n  }%s\n", (i + 1 < metrics.size()) ? "," : "");
  }
  fprintf(f, "]\n");
  fclose(f);
}

void webrtc::L4SMetricsCollector::UpdateThroughputStats(DataRate actual_bitrate) {
  throughput_stats_.AddSample(actual_bitrate.bps());
}

void webrtc::L4SMetricsCollector::UpdateDelayStats(TimeDelta rtt,
                                                   TimeDelta one_way_delay) {
  if (rtt.IsFinite()) {
    rtt_stats_.AddSample(rtt.ms());
  }
  if (one_way_delay.IsFinite()) {
    delay_stats_.AddSample(one_way_delay.ms());
  }
}

void webrtc::L4SMetricsCollector::UpdateLossStats(double loss_fraction) {
  loss_stats_.AddSample(loss_fraction);
}

// =============================================================================
// L4SNetworkController Implementation
// =============================================================================

webrtc::L4SNetworkController::L4SNetworkController(NetworkControllerConfig config,
                                          L4SControllerConfig l4s_config,
                                          test::MetricsLogger* metrics_logger)
    : env_(config.env), config_(l4s_config) {
  
  // Initialize Prague capacity estimator
  DataRate starting_rate = config.constraints.starting_rate.value_or(DataRate::KilobitsPerSec(300));
  DataRate min_rate = config.constraints.min_data_rate.value_or(DataRate::KilobitsPerSec(30));
  DataRate max_rate = config.constraints.max_data_rate.value_or(DataRate::KilobitsPerSec(100000));
  
  prague_estimator_ = std::make_unique<PragueCapacityEstimator>(starting_rate, min_rate, max_rate);
  
  // Initialize bandwidth fusion engine
  bandwidth_fusion_ = std::make_unique<L4SBandwidthFusion>(config_);

  // Set initial rate constraints BEFORE initializing estimators so that
  // InitializeBandwidthEstimators() can read them if needed.
  starting_rate_ = config.constraints.starting_rate;
  min_target_rate_ = config.constraints.min_data_rate;
  max_target_rate_ = config.constraints.max_data_rate;
  target_rate_ = starting_rate;

  // Initialize bandwidth estimation components
  InitializeBandwidthEstimators();
  
  // Initialize metrics collector
  if (config_.enable_metrics_collection) {
    using webrtc::test::GetGlobalMetricsLogger;
    test::MetricsLogger* logger_to_use = metrics_logger;
    if (!logger_to_use) {
      logger_to_use = GetGlobalMetricsLogger();
    }
    metrics_collector_ = std::make_unique<L4SMetricsCollector>(
        logger_to_use, config_.test_case_name);
  }

  AdvanceStateMachine(Timestamp::Millis(env_.clock().TimeInMilliseconds()));
  
  RTC_LOG(LS_INFO) << "L4SNetworkController created with starting rate: " 
                   << starting_rate.bps() << " bps";
}

webrtc::L4SNetworkController::~L4SNetworkController() {
  if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->ExportToJsonFile("l4s_network_metrics.json");
    RTC_LOG(LS_INFO) << "L4S: Exported metrics to l4s_network_metrics.json";
  }
}

void webrtc::L4SNetworkController::InitializeBandwidthEstimators() {
  if (config_.enable_probing) {
    probe_controller_ = std::make_unique<ProbeController>(&env_.field_trials(), &env_.event_log());
    probe_bitrate_estimator_ = std::make_unique<ProbeBitrateEstimator>(&env_.event_log());
    // SetBitrates is intentionally deferred to the first OnProcessInterval call
    // so we have a valid network timestamp and can return probe clusters to the
    // caller.  Calling it here would (a) use a stale clock timestamp and (b)
    // silently discard the returned ProbeClusterConfig vector.
  }
  
  if (config_.enable_acked_estimation) {
    acked_estimator_ = std::make_unique<AcknowledgedBitrateEstimator>(&env_.field_trials());
  }
  
  if (config_.enable_alr_detection) {
    alr_detector_ = std::make_unique<AlrDetector>(&env_.field_trials());
  }
  
  RTC_LOG(LS_INFO) << "L4S: Initialized bandwidth estimators - "
                   << "Probe: " << (probe_controller_ ? "enabled" : "disabled")
                   << ", Acked: " << (acked_estimator_ ? "enabled" : "disabled")
                   << ", ALR: " << (alr_detector_ ? "enabled" : "disabled");
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnNetworkAvailability(NetworkAvailability msg) {
  NetworkControlUpdate update;
  if (probe_controller_) {
    auto avail_probes = probe_controller_->OnNetworkAvailability(msg);
    // Global probe-rate limiter: never emit probes more frequently than
    // config_.probe_interval, regardless of source. NetworkAvailability
    // has no timestamp, so use the current env_ clock.
    Timestamp now = Timestamp::Millis(env_.clock().TimeInMilliseconds());
    for (const auto& probe : avail_probes) {
      TimeDelta since_last_probe =
          last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity()
                                        : (now - last_probe_time_);
      if (since_last_probe >= config_.probe_interval) {
        update.probe_cluster_configs.push_back(probe);
        last_probe_time_ = now;
        StartProbeHold(now);
      } else {
        RTC_LOG(LS_VERBOSE)
            << "L4S: Dropping availability probe due to global interval gate";
      }
    }
  }
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnNetworkRouteChange(NetworkRouteChange msg) {
  NetworkControlUpdate update;
  
  RTC_LOG(LS_INFO) << "L4S: OnNetworkRouteChange called";
  
  // Reset ECN support detection on network change
  ecn_supported_ = false;
  last_congestion_signal_ = Timestamp::MinusInfinity();
  
  // Update rate constraints
  if (msg.constraints.starting_rate) {
    starting_rate_ = msg.constraints.starting_rate;
    target_rate_ = starting_rate_;
  }
  min_target_rate_ = msg.constraints.min_data_rate;
  max_target_rate_ = msg.constraints.max_data_rate;

  TransitionToState(ControllerState::kRouteReset,
                    TransitionReason::kRouteChange,
                    Timestamp::Millis(env_.clock().TimeInMilliseconds()));
  
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnProcessInterval(ProcessInterval msg) {
  NetworkControlUpdate update;
  
  // Log periodic metrics
  LogPeriodicMetrics(msg.at_time);

  // --- ProbeController integration (GCC-compatible) ---
  if (probe_controller_) {
    // First call: initialise ProbeController with properly clamped bitrates and
    // the real network timestamp.  We defer this from the constructor so that
    // (a) the at_time is valid and (b) we can return probe clusters to the
    // transport.
    if (!initial_probes_sent_) {
      initial_probes_sent_ = true;
      DataRate clamped_min =
          min_target_rate_.value_or(DataRate::KilobitsPerSec(30));
      DataRate clamped_start =
          starting_rate_.value_or(DataRate::KilobitsPerSec(300));
      // Guard against PlusInfinity being passed to ProbeController – that
      // causes internal probe targets to overflow to infinity.
      DataRate clamped_max =
          max_target_rate_.value_or(DataRate::KilobitsPerSec(100000));
      if (!clamped_max.IsFinite()) {
        clamped_max = DataRate::KilobitsPerSec(100000);  // 100 Mbps ceiling
      }
      // Ensure ordering invariant: min <= start <= max
      clamped_start = std::max(clamped_min, clamped_start);
      clamped_max   = std::max(clamped_start, clamped_max);

      auto init_probes = probe_controller_->SetBitrates(
          clamped_min, clamped_start, clamped_max, msg.at_time);
      update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                          init_probes.begin(), init_probes.end());
      if (!init_probes.empty()) {
        StartProbeHold(msg.at_time);
      }
      probe_controller_->EnablePeriodicAlrProbing(true);
      RTC_LOG(LS_INFO) << "L4S: ProbeController initialised - "
                       << "min=" << clamped_min.bps() << " bps, "
                       << "start=" << clamped_start.bps() << " bps, "
                       << "max=" << clamped_max.bps() << " bps, "
                       << "initial_probes=" << init_probes.size();
    }

    // Every interval: feed ALR state so the probe controller can trigger ALR
    // probes at the right time.
    if (alr_detector_) {
      probe_controller_->SetAlrStartTimeMs(
          alr_detector_->GetApplicationLimitedRegionStartTime());
    }

    // Let ProbeController emit any time-driven probes (ALR periodic, network
    // state probes, etc.).
    auto periodic_probes = probe_controller_->Process(msg.at_time);
    for (const auto& probe : periodic_probes) {
      TimeDelta since_last_probe =
          last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity()
                                        : (msg.at_time - last_probe_time_);
      if (since_last_probe >= config_.probe_interval) {
        update.probe_cluster_configs.push_back(probe);
        last_probe_time_ = msg.at_time;
        StartProbeHold(msg.at_time);
      } else {
        RTC_LOG(LS_VERBOSE)
            << "L4S: Dropping periodic ProbeController probe due to global interval gate";
      }
    }
  }
  // --- end ProbeController integration ---

  // State-owned probing policy
  ApplyStateProbingPolicy(msg.at_time, &update);
  
  // Update time-based decay in Prague estimator
  prague_estimator_->OnTimeUpdate(msg.at_time);
  
  // State-owned fusion policy
  DataRate fused_rate = ApplyStateFusionPolicy(msg.at_time);
  target_rate_ = fused_rate;
  
  // Create rate update
  MaybeTriggerOnNetworkChanged(&update, msg.at_time);

  AdvanceStateMachine(msg.at_time);
  
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnRemoteBitrateReport(RemoteBitrateReport msg) {
  NetworkControlUpdate update;
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnRoundTripTimeUpdate(RoundTripTimeUpdate msg) {
  NetworkControlUpdate update;

  // Follow GCC behavior: ignore explicitly smoothed RTT updates and consume
  // raw RTT samples from RTCP transport reports.
  if (msg.smoothed) {
    return update;
  }

  if (msg.round_trip_time.IsFinite() && !msg.round_trip_time.IsZero()) {
    prague_estimator_->UpdateFromRtt(msg.round_trip_time);
    last_rtt_ = msg.round_trip_time;
    last_estimated_round_trip_time_ = msg.round_trip_time;
  }

  // Log RTT metrics
  if (metrics_enabled_ && metrics_collector_ && msg.round_trip_time.IsFinite() &&
      !msg.round_trip_time.IsZero()) {
    metrics_collector_->LogDelayMetrics(
        Timestamp::Millis(env_.clock().TimeInMilliseconds()),
        msg.round_trip_time, TimeDelta::PlusInfinity(), TimeDelta::Zero());
  }

  RTC_LOG(LS_VERBOSE) << "L4S: RTT updated to " << msg.round_trip_time.ms()
                      << " ms";

  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnSentPacket(SentPacket msg) {
  NetworkControlUpdate update;
  // Feed ALR detector so it can track application-limited periods.
  if (alr_detector_) {
    alr_detector_->OnBytesSent(msg.size.bytes(), msg.send_time.ms());
    if (acked_estimator_) {
      acked_estimator_->SetAlr(
          alr_detector_->GetApplicationLimitedRegionStartTime().has_value());
    }
  }
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnReceivedPacket(ReceivedPacket msg) {
  NetworkControlUpdate update;
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnStreamsConfig(StreamsConfig msg) {
  NetworkControlUpdate update;
  if (probe_controller_) {
    if (msg.requests_alr_probing) {
      probe_controller_->EnablePeriodicAlrProbing(*msg.requests_alr_probing);
    }
    if (msg.max_total_allocated_bitrate) {
      auto probes = probe_controller_->OnMaxTotalAllocatedBitrate(
          *msg.max_total_allocated_bitrate, msg.at_time);
      update.probe_cluster_configs.insert(update.probe_cluster_configs.end(),
                                          probes.begin(), probes.end());
    }
  }
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnTargetRateConstraints(TargetRateConstraints msg) {
  NetworkControlUpdate update;
  
  // Update constraints
  min_target_rate_ = msg.min_data_rate;
  max_target_rate_ = msg.max_data_rate;
  
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnTransportLossReport(TransportLossReport msg) {
  NetworkControlUpdate update;
  
  if (msg.packets_lost_delta > 0) {
    RTC_LOG(LS_INFO) << "L4S: Transport loss report - "
                     << "Lost: " << msg.packets_lost_delta
                     << ", Received: " << msg.packets_received_delta;
    
    DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
    prague_estimator_->OnPacketLoss(current_rate, msg.receive_time);
  }
  
  // Update loss metrics
  int total_packets = msg.packets_lost_delta + msg.packets_received_delta;
  if (total_packets > 0) {
    last_loss_fraction_ = static_cast<double>(msg.packets_lost_delta) / total_packets;
  } else {
    last_loss_fraction_ = 0.0;
  }
  last_packets_lost_ = static_cast<int>(msg.packets_lost_delta);
  
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnTransportPacketsFeedback(TransportPacketsFeedback msg) {
  NetworkControlUpdate update;

  // Track packet loss count from per-packet feedback for logging only.
  // CE remains the primary congestion-control signal.
  int packets_with_feedback = 0;
  int lost_packets_in_feedback = 0;
  for (const auto& packet_feedback : msg.PacketsWithFeedback()) {
    ++packets_with_feedback;
    if (!packet_feedback.IsReceived()) {
      ++lost_packets_in_feedback;
    }
  }
  if (packets_with_feedback > 0) {
    last_packets_lost_ = lost_packets_in_feedback;
  }
  
  // Update all bandwidth estimators
  UpdateAllBandwidthEstimators(msg);
  
  // State-owned probing policy
  ApplyStateProbingPolicy(msg.feedback_time, &update);
  
  // State-owned fusion policy
  DataRate fused_rate = ApplyStateFusionPolicy(msg.feedback_time);
  target_rate_ = fused_rate;
  
  // Update throughput calculation
  UpdateThroughputWindow(msg);
  
  // Create rate update
  MaybeTriggerOnNetworkChanged(&update, msg.feedback_time);

  AdvanceStateMachine(msg.feedback_time);
  
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnNetworkStateEstimate(NetworkStateEstimate msg) {
  NetworkControlUpdate update;
  return update;
}

void webrtc::L4SNetworkController::UpdateAllBandwidthEstimators(const TransportPacketsFeedback& feedback) {
  // Update ALR detector first
  UpdateAlrDetector(feedback);

  // GCC-style RTT update from transport feedback: derive RTT per received
  // packet and use the minimum sample in this feedback batch.
  std::vector<PacketResult> received_feedback = feedback.SortedByReceiveTime();
  if (!received_feedback.empty()) {
    const Timestamp max_recv_time = received_feedback.back().receive_time;
    TimeDelta feedback_min_rtt = TimeDelta::PlusInfinity();
    for (const auto& packet_feedback : received_feedback) {
      TimeDelta pending_time = max_recv_time - packet_feedback.receive_time;
      TimeDelta rtt = feedback.feedback_time - packet_feedback.sent_packet.send_time - pending_time;
      feedback_min_rtt = std::min(feedback_min_rtt, rtt);
    }
    if (feedback_min_rtt.IsFinite() && !feedback_min_rtt.IsZero()) {
      prague_estimator_->UpdateFromRtt(feedback_min_rtt);
      last_rtt_ = feedback_min_rtt;
      last_estimated_round_trip_time_ = feedback_min_rtt;
    }
  }
  
  // 1. Update non-ECN estimators first (acked, probe)
  if (acked_estimator_) {
    UpdateAckedBitrateEstimator(feedback);
  }
  
  if (probe_controller_) {
    ProcessRealProbeResults(feedback);
  }
  
  // 2. Get initial fused estimate (without ECN input)
  DataRate base_fused_rate = GetBaseFusedEstimate(feedback.feedback_time);
  
  // 3. State-owned ECN policy
  ApplyStateEcnPolicy(feedback, base_fused_rate);
}

void webrtc::L4SNetworkController::ApplyStateEcnPolicy(
    const TransportPacketsFeedback& feedback,
    DataRate base_fused_rate) {
  switch (controller_state_) {
    case ControllerState::kRouteReset:
    case ControllerState::kSlowStart:
    case ControllerState::kCongestionAvoidance:
    case ControllerState::kCongestionExperienced:
    case ControllerState::kCongestionRecovery:
      break;
  }

  if (!IsApplicationLimited()) {
    ProcessEcnFeedback(feedback, base_fused_rate);
  } else {
    RTC_LOG(LS_VERBOSE) << "L4S: Skipping ECN processing during ALR period";
  }
}

void webrtc::L4SNetworkController::ProcessEcnFeedback(const TransportPacketsFeedback& feedback, DataRate current_fused_rate) {
  if (feedback.packet_feedbacks.empty()) {
    return;
  }
  
  int new_ect_count = 0;
  int new_ce_count = 0;
  // Separate video-only counts for recovery detection.
  // Audio CE packets are excluded from recovery logic so that lightweight
  // audio CE marks (common on low-bandwidth audio streams) do not prevent
  // or prematurely terminate recovery of the dominant video path.
  // sent_packet.audio is set by TransportFeedbackAdapter from
  // RtpPacketMediaType::kAudio at send time.
  int new_video_ect_count = 0;
  int new_video_ce_count = 0;

  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1 || packet.ecn == EcnMarking::kCe) {
      new_ect_count++;
    }
    if (packet.ecn == EcnMarking::kCe) {
      new_ce_count++;
      last_congestion_signal_ = feedback.feedback_time;
      
      RTC_LOG(LS_VERBOSE) << "L4S: CE mark detected! Count=" << new_ce_count
                         << ", ECT count=" << new_ect_count;
    }
    // Video-only counts (audio=false covers video, padding, RTX)
    if (!packet.sent_packet.audio) {
      if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1 || packet.ecn == EcnMarking::kCe) {
        new_video_ect_count++;
      }
      if (packet.ecn == EcnMarking::kCe) {
        new_video_ce_count++;
      }
    }
  }
  
  // Update ECN support detection
  if (new_ect_count > 0 || new_ce_count > 0) {
    ecn_supported_ = true;
    
    // Track ECN activity for confidence calculation
    prague_estimator_->UpdateEcnActivity(feedback.feedback_time);
  }
  
  // Update Prague estimator with CE ratio using intelligent bottleneck detection
  if (new_ect_count + new_ce_count > 0) {
    double ce_ratio = static_cast<double>(new_ce_count) / (new_ect_count + new_ce_count);
    
    // During discovery mode, use Prague's own estimate to create positive feedback loop
    DataRate prague_input_rate;
    if (prague_estimator_->IsDiscoveryModeActive()) {
      prague_input_rate = prague_estimator_->GetCurrentEstimate();
      RTC_LOG(LS_VERBOSE) << "L4S: Discovery mode - using Prague's own estimate as input: " << prague_input_rate.bps() 
                       << " bps (bypassing constrained fused rate: " << current_fused_rate.bps() << ")";
    } else {
      prague_input_rate = DetermineBottleneckAwareTarget(current_fused_rate);
      RTC_LOG(LS_VERBOSE) << "L4S: Applying Prague AI/MD to bottleneck-aware rate: " << prague_input_rate.bps() 
                       << " bps (original fused: " << current_fused_rate.bps() << ", ce_ratio=" << ce_ratio << ")";
    }
    
    prague_estimator_->UpdateFromCongestionSignal(prague_input_rate, ce_ratio, feedback.feedback_time);
    
    // Actual-rate floor: the network is provably delivering last_actual_bitrate_,
    // so allow Prague to drop no lower than 50% of that observed throughput.
    // This is a safety net for edge cases where last_rtt_ is temporarily
    // invalid and the once-per-RTT gate cannot protect against cascading MDs.
    if (!last_actual_bitrate_.IsZero()) {
      DataRate floor = last_actual_bitrate_ * 0.5;
      DataRate prague_current = prague_estimator_->GetCurrentEstimate();
      if (prague_current < floor) {
        RTC_LOG(LS_VERBOSE) << "Prague: Actual-rate floor applied: " << prague_current.bps()
                           << " bps raised to " << floor.bps()
                           << " bps (50% of actual throughput " << last_actual_bitrate_.bps() << " bps)";
        // Re-seed the estimator at the floor so AI resumes from a sensible base
        prague_estimator_->SetCurrentEstimate(floor);
      }
    }
    
    // Update fusion engine with ECN estimate
    double ecn_confidence = prague_estimator_->GetConfidence(feedback.feedback_time);
    
    // Boost ECN confidence when Prague is in reduction mode to prevent other estimators from overriding
    if (prague_estimator_->GetDirectionFlag() == -1) {
      ecn_confidence = std::max(ecn_confidence, 0.95);  // Very high confidence during reduction
      RTC_LOG(LS_VERBOSE) << "L4S: Prague in reduction mode, boosting ECN confidence to " << ecn_confidence;
    }
    
    bandwidth_fusion_->UpdateEcnEstimate(prague_estimator_->GetCurrentEstimate(), ecn_confidence, feedback.feedback_time);
    
    // Log congestion metrics
    if (metrics_enabled_ && metrics_collector_) {
      metrics_collector_->LogCongestionMetrics(feedback.feedback_time, new_ce_count, new_ect_count, ce_ratio);
    }
  }
  
  // Handle packet-based recovery detection (video-only counts).
  // Passing video-only ECT/CE ensures that CE marks on the lightweight audio
  // stream do not prematurely terminate recovery of the video path.
  HandleRecoveryDetection(new_video_ect_count, new_video_ce_count, feedback.feedback_time);
}

webrtc::DataRate webrtc::L4SNetworkController::DetermineBottleneckAwareTarget(DataRate fused_rate) {
  // During discovery mode, bypass bottleneck constraints to allow aggressive growth
  if (prague_estimator_ && prague_estimator_->IsDiscoveryModeActive()) {
    RTC_LOG(LS_VERBOSE) << "L4S: Discovery mode active - bypassing bottleneck detection, using fused rate: " 
                     << fused_rate.bps() << " bps";
    return fused_rate;
  }
  
  // Intelligent bottleneck detection for Prague rate targeting
  auto sources = bandwidth_fusion_->GetCurrentSources();
  
  DataRate probe_capacity = sources.probe_estimate;
  DataRate acked_throughput = sources.acked_estimate;
  
  // Detect bottleneck type based on rate relationships
  double probe_vs_acked_ratio = 1.0;
  if (acked_throughput > DataRate::Zero() && probe_capacity > DataRate::Zero()) {
    probe_vs_acked_ratio = probe_capacity.bps() / static_cast<double>(acked_throughput.bps());
  }
  
  // Scenario 1: Large gap between probe and acked (receiver/app bottleneck)
  if (probe_vs_acked_ratio > 2.0) {
    if (IsApplicationLimited()) {
      // Application bottleneck - respect application demand
      RTC_LOG(LS_VERBOSE) << "L4S: Application bottleneck detected, targeting acked rate: " 
                       << acked_throughput.bps() << " bps";
      return acked_throughput;
    } else {
      // Receiver processing bottleneck - probe gently upward
      DataRate gentle_target = acked_throughput * 1.25;  // 25% increase
      gentle_target = std::min(gentle_target, fused_rate);  // Don't exceed fused rate
      RTC_LOG(LS_VERBOSE) << "L4S: Receiver bottleneck detected, gentle increase to: " 
                       << gentle_target.bps() << " bps (probe: " << probe_capacity.bps() 
                       << ", acked: " << acked_throughput.bps() << ")";
      return gentle_target;
    }
  }
  
  // Scenario 2: Network is the bottleneck - follow network-centric approach
  // Use fused rate which includes probe discoveries and safety constraints
  RTC_LOG(LS_VERBOSE) << "L4S: Network bottleneck detected, targeting fused rate: " 
                   << fused_rate.bps() << " bps (ratio: " << probe_vs_acked_ratio << ")";
  return fused_rate;
}

void webrtc::L4SNetworkController::UpdateAckedBitrateEstimator(const TransportPacketsFeedback& feedback) {
  if (feedback.packet_feedbacks.empty() || !acked_estimator_) {
    return;
  }

  // Match GCC behavior: compute delivery-rate signal from per-packet feedback.
  acked_estimator_->IncomingPacketFeedbackVector(feedback.SortedByReceiveTime());
  std::optional<DataRate> acked_bitrate = acked_estimator_->bitrate();
  if (!acked_bitrate.has_value()) {
    return;
  }

  last_acked_bitrate_ = acked_bitrate;
  double acked_confidence = CalculateAckedConfidence(feedback.feedback_time);
  bandwidth_fusion_->UpdateAckedEstimate(*acked_bitrate, acked_confidence,
                                         feedback.feedback_time);
}

void webrtc::L4SNetworkController::ProcessRealProbeResults(const TransportPacketsFeedback& feedback) {
  if (!probe_bitrate_estimator_) {
    return;
  }

  // Process each packet to detect probe clusters and measure real throughput
  for (const auto& packet_feedback : feedback.SortedByReceiveTime()) {
    if (packet_feedback.sent_packet.pacing_info.probe_cluster_id != PacedPacketInfo::kNotAProbe) {
      // This is a real probe packet - let ProbeBitrateEstimator measure it
      probe_bitrate_estimator_->HandleProbeAndEstimateBitrate(packet_feedback);
      
      RTC_LOG(LS_VERBOSE) << "L4S: Processing probe packet from cluster " 
                         << packet_feedback.sent_packet.pacing_info.probe_cluster_id
                         << ", size: " << packet_feedback.sent_packet.size.bytes() << " bytes";
    }
  }

  // Get the real measured probe result (if any)
  std::optional<DataRate> measured_probe_rate = GetLastProbeResult();
  if (measured_probe_rate) {
    // Layer 1 sanity guard: discard probe results that are physically impossible.
    // If the link is already delivering more actual throughput than the probe measured,
    // the probe result is an artifact (e.g. DualPI2 scheduling spreading the burst
    // across a long receive window at uncongested rates) and must not be used to cap
    // the rate. A real capacity measurement can never be below the already-observed
    // delivery rate.
    if (last_actual_bitrate_.IsZero() || *measured_probe_rate >= last_actual_bitrate_) {
      double probe_confidence = CalculateProbeConfidence(feedback.feedback_time);
      bandwidth_fusion_->UpdateProbeEstimate(*measured_probe_rate, probe_confidence, feedback.feedback_time);

      RTC_LOG(LS_VERBOSE) << "L4S: Real probe result accepted: " << measured_probe_rate->bps()
                          << " bps (actual throughput: " << last_actual_bitrate_.bps()
                          << " bps) with confidence " << probe_confidence;
    } else {
      RTC_LOG(LS_INFO) << "L4S: Probe result discarded (physically impossible): "
                       << measured_probe_rate->bps() << " bps < actual throughput "
                       << last_actual_bitrate_.bps() << " bps - likely AQM scheduling artifact";
    }
  }
}

std::optional<DataRate> webrtc::L4SNetworkController::GetLastProbeResult() {
  if (!probe_bitrate_estimator_) {
    return std::nullopt;
  }
  
  // Fetch the latest real probe measurement from the estimator
  return probe_bitrate_estimator_->FetchAndResetLastEstimatedBitrate();
}

void webrtc::L4SNetworkController::HandlePeriodicProbing(Timestamp now, NetworkControlUpdate* update) {
  if (!config_.enable_probing || !probe_controller_) {
    RTC_LOG(LS_WARNING) << "L4S: Probing disabled - enable_probing=" << config_.enable_probing 
                        << ", probe_controller=" << (probe_controller_ ? "available" : "null");
    return;
  }

  if (!probe_hold_until_.IsInfinite() && now < probe_hold_until_) {
    RTC_LOG(LS_VERBOSE) << "L4S: Probe hold active, skipping probe scheduling";
    return;
  }
  
  // Recovery probing has higher priority and frequency
  if (recovery_mode_active_) {
    TimeDelta since_last_probe = last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity() : (now - last_probe_time_);
    int64_t probe_ms = since_last_probe.IsInfinite() ? -1 : since_last_probe.ms();
    RTC_LOG(LS_VERBOSE) << "L4S: Recovery mode active, time since last probe: " << probe_ms << "ms";
    // Use a dedicated recovery interval, slower than before to avoid CE bursts.
    TimeDelta recovery_interval = GetRttScaledInterval();
    if (since_last_probe >= recovery_interval) {
      RTC_LOG(LS_INFO) << "L4S: Initiating recovery probe!";
      InitiateRecoveryProbing(now, update);
      last_probe_time_ = now;
    }
    return;
  }
  
  // Regular periodic probing
  bool interval_ok = last_probe_time_.IsInfinite() || (now - last_probe_time_) >= config_.probe_interval;
  bool probe_allowed = ShouldProbeNow(now);
  bool should_probe = interval_ok && probe_allowed;
  
  // More detailed debug logging with INFO level (safe timestamp handling)
  int64_t time_since_last_ms = last_probe_time_.IsInfinite() ? -1 : (now - last_probe_time_).ms();
  RTC_LOG(LS_VERBOSE) << "L4S: Probe decision - time_since_last=" << time_since_last_ms
                   << "ms, interval_req=" << config_.probe_interval.ms() 
                   << "ms, interval_ok=" << interval_ok 
                   << ", probe_allowed=" << probe_allowed 
                   << ", final_decision=" << should_probe;
  
  if (should_probe) {
    RTC_LOG(LS_INFO) << "L4S: Initiating periodic probe!";
    InitiateProbing(now, update);
    last_probe_time_ = now;
  }
}

void webrtc::L4SNetworkController::ApplyStateProbingPolicy(
    Timestamp now,
    NetworkControlUpdate* update) {
  switch (controller_state_) {
    case ControllerState::kRouteReset:
    case ControllerState::kSlowStart:
    case ControllerState::kCongestionAvoidance:
    case ControllerState::kCongestionExperienced:
    case ControllerState::kCongestionRecovery:
      HandlePeriodicProbing(now, update);
      return;
  }
}

TimeDelta webrtc::L4SNetworkController::GetRttScaledInterval() const {
  TimeDelta effective_rtt =
      last_rtt_.IsFinite() && !last_rtt_.IsZero()
          ? std::clamp(last_rtt_, TimeDelta::Millis(50), TimeDelta::Millis(300))
          : TimeDelta::Millis(100);
  TimeDelta rtt_scaled = effective_rtt * config_.recovery_probe_rtt_factor;
  return std::max(config_.min_rtt_scaled_interval, rtt_scaled);
}

void webrtc::L4SNetworkController::StartProbeHold(Timestamp now) {
  TimeDelta effective_rtt =
      last_rtt_.IsFinite() && !last_rtt_.IsZero()
          ? std::clamp(last_rtt_, TimeDelta::Millis(50), TimeDelta::Millis(300))
          : TimeDelta::Millis(100);
  TimeDelta hold = std::max(config_.min_rtt_scaled_interval,
                            effective_rtt * config_.probe_hold_rtt_factor);
  probe_hold_until_ = now + hold;
  if (prague_estimator_) {
    prague_estimator_->SetAdditiveHoldUntil(probe_hold_until_);
  }
}

bool webrtc::L4SNetworkController::ShouldProbeNow(Timestamp now) const {
  // Avoid periodic probing at very low rates where measurements are noisy.
  DataRate current_estimate = target_rate_.value_or(DataRate::Zero());
  if (current_estimate < config_.periodic_probe_min_rate) {
    RTC_LOG(LS_VERBOSE) << "L4S: Blocking probe due to low target rate: "
                        << current_estimate.bps() << " < "
                        << config_.periodic_probe_min_rate.bps();
    return false;
  }

  // Don't probe if we're experiencing heavy congestion
  if (HasRecentCongestionSignals(now)) {
    RTC_LOG(LS_VERBOSE) << "L4S: Blocking probe due to recent congestion signals";
    return false;
  }
  
  // Don't probe if ECN feedback is very fresh and confident (more permissive for discovery)
  double ecn_confidence = prague_estimator_->GetConfidence(now);
  bool is_discovery_mode = prague_estimator_->IsDiscoveryModeActive();
  double confidence_threshold = is_discovery_mode
                                    ? config_.discovery_probe_block_confidence
                                    : config_.steady_probe_block_confidence;
  
  if (IsEcnFeedbackFresh(now) && ecn_confidence > confidence_threshold) {
    RTC_LOG(LS_VERBOSE) << "L4S: Blocking probe due to high ECN confidence: " << ecn_confidence 
                        << " > " << confidence_threshold << " (mode: " << (is_discovery_mode ? "discovery" : "recovery") << ")";
    return false;
  }
  
  // Don't probe during high loss periods
  if (last_loss_fraction_ > 0.02) {  // 2% loss threshold
    RTC_LOG(LS_VERBOSE) << "L4S: Blocking probe due to high loss: " << last_loss_fraction_;
    return false;
  }
  
  RTC_LOG(LS_VERBOSE) << "L4S: Probe allowed - ECN confidence: " << ecn_confidence 
                      << ", loss: " << last_loss_fraction_;
  return true;
}

void webrtc::L4SNetworkController::InitiateProbing(Timestamp now, NetworkControlUpdate* update) {
  if (!probe_controller_) {
    return;
  }

  // Get current best estimate for probe rate calculation
  DataRate current_estimate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  
  // Use conservative probing by default and only a modest boost in ALR.
  double probe_multiplier = config_.probe_multiplier;
  if (IsApplicationLimited()) {
    probe_multiplier = config_.alr_probe_multiplier;
    RTC_LOG(LS_VERBOSE) << "L4S: Using ALR probe multiplier=" << probe_multiplier;
  }
  
  // Calculate probe rate
  DataRate probe_rate = current_estimate * probe_multiplier;
  if (max_target_rate_) {
    probe_rate = std::min(probe_rate, *max_target_rate_);
  }

  // RequestProbe handles the ALR-end + large-drop case.  Periodic ALR
  // probing is already driven by Process() in OnProcessInterval, so there is
  // nothing else to do here for the normal (non-ALR) steady-state path.
  auto probes = probe_controller_->RequestProbe(now);
  if (!probes.empty()) {
    update->probe_cluster_configs.insert(update->probe_cluster_configs.end(),
                                         probes.begin(), probes.end());
    StartProbeHold(now);
    RTC_LOG(LS_VERBOSE) << "L4S: RequestProbe returned " << probes.size()
                        << " probe clusters";
  }
  (void)probe_rate;  // calculated above, kept for ALR multiplier logic
}

double webrtc::L4SNetworkController::CalculateProbeConfidence(Timestamp now) const {
  TimeDelta since_probe = last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity() : (now - last_probe_time_);
  if (since_probe < TimeDelta::Seconds(1)) {
    return config_.probe_confidence_fresh;
  } else if (since_probe < TimeDelta::Seconds(10)) {
    return config_.probe_confidence_recent;
  }
  return 0.2;   // Low confidence in old probe results
}

double webrtc::L4SNetworkController::CalculateAckedConfidence(Timestamp now) const {
  if (!acked_estimator_) {
    return 0.0;
  }
  
  // Moderate confidence in acknowledged bitrate
  return 0.6;
}

webrtc::DataRate webrtc::L4SNetworkController::FuseBandwidthEstimates(Timestamp now) {
  bool discovery_signal = prague_estimator_ && prague_estimator_->IsDiscoveryModeActive();
  bool discovery_active =
      (controller_state_ == ControllerState::kSlowStart) && discovery_signal;

  // Bridge current and previous behavior during the phased refactor.
  if (!discovery_active && discovery_signal &&
      controller_state_ != ControllerState::kCongestionRecovery) {
    discovery_active = true;
  }

  bool recovery_active =
      (controller_state_ == ControllerState::kCongestionRecovery) ||
      recovery_mode_active_;
  
  // Check if we should exit discovery mode based on convergence
  if (discovery_active && ShouldExitDiscoveryMode(now)) {
    prague_estimator_->ExitDiscoveryMode("probe-Prague convergence or fallback threshold");
    discovery_active = false;
  }
  
  // Update Prague with probe constraints during discovery
  if (discovery_active) {
    auto sources = bandwidth_fusion_->GetCurrentSources();
    if (sources.probe_confidence > 0.7) {
      prague_estimator_->SetProbeConstraint(sources.probe_estimate, sources.probe_confidence);
    }
  }
  
  // Use mode-aware fusion (discovery/recovery modes use probe-weighted fusion)
  DataRate fused_rate =
      bandwidth_fusion_->GetFusedEstimateWithMode(now, discovery_active,
                                                  recovery_active);
  
  // During discovery mode, still use Prague's estimate as it incorporates probe constraints
  if (discovery_active) {
    DataRate prague_rate = prague_estimator_->GetCurrentEstimate();
    RTC_LOG(LS_VERBOSE) << "L4S: Discovery mode - Prague rate: " << prague_rate.bps() 
                     << " bps, Fused rate: " << fused_rate.bps() << " bps";
    // Use Prague rate if it's lower (respecting probe constraints)
    fused_rate = std::min(prague_rate, fused_rate);
  }
  
  // Apply rate constraints
  if (min_target_rate_ && fused_rate < *min_target_rate_) {
    fused_rate = *min_target_rate_;
  }
  // Final safety: enforce absolute minimum of 20 kbps to prevent pacer crashes
  fused_rate = std::max(fused_rate, DataRate::KilobitsPerSec(20));
  if (max_target_rate_ && fused_rate > *max_target_rate_) {
    fused_rate = *max_target_rate_;
  }
  
  // Log fusion metrics
  // if (metrics_enabled_ && metrics_collector_) {
  //   auto sources = bandwidth_fusion_->GetCurrentSources();
  //   metrics_collector_->LogFusionMetrics(now, sources, fused_rate);
  // }
  
  return fused_rate;
}

webrtc::DataRate webrtc::L4SNetworkController::ApplyStateFusionPolicy(
    Timestamp now) {
  switch (controller_state_) {
    case ControllerState::kRouteReset:
    case ControllerState::kSlowStart:
    case ControllerState::kCongestionAvoidance:
    case ControllerState::kCongestionExperienced:
    case ControllerState::kCongestionRecovery:
      return FuseBandwidthEstimates(now);
  }
  return FuseBandwidthEstimates(now);
}

webrtc::DataRate webrtc::L4SNetworkController::GetBaseFusedEstimate(Timestamp now) {
  // Get fused estimate from non-ECN sources only (delay, probe, acked)
  // This provides the base capacity estimate before Prague applies AI/MD
  
  L4SBandwidthFusion::BandwidthSources temp_sources = bandwidth_fusion_->GetCurrentSources();
  
  // Temporarily zero out ECN estimate for base fusion
  temp_sources.ecn_estimate = DataRate::Zero();
  temp_sources.ecn_confidence = 0.0;
  
  // Use weighted combination of delay, probe, and acked estimates as base
  double total_weight = temp_sources.delay_confidence + 
                       temp_sources.probe_confidence + 
                       temp_sources.acked_confidence;
  if (total_weight > 0.01) {  // Very low threshold - almost always use weighted combination
    DataRate weighted_estimate = 
        (temp_sources.delay_estimate * temp_sources.delay_confidence + 
         temp_sources.probe_estimate * temp_sources.probe_confidence +
         temp_sources.acked_estimate * temp_sources.acked_confidence) / total_weight;
    
    RTC_LOG(LS_VERBOSE) << "L4S: Base fused estimate (no ECN): " << weighted_estimate.bps() << " bps";
    return weighted_estimate;
  }
  
  // Fallback to most confident non-ECN estimate
  DataRate best_estimate = DataRate::KilobitsPerSec(300);  // Fallback
  double best_confidence = 0.0;
  
  if (temp_sources.delay_confidence > best_confidence) {
    best_estimate = temp_sources.delay_estimate;
    best_confidence = temp_sources.delay_confidence;
  }
  
  if (temp_sources.acked_confidence > best_confidence) {
    best_estimate = temp_sources.acked_estimate;
    best_confidence = temp_sources.acked_confidence;
  }
  
  if (temp_sources.probe_confidence > best_confidence) {
    best_estimate = temp_sources.probe_estimate;
    best_confidence = temp_sources.probe_confidence;
  }
  
  RTC_LOG(LS_VERBOSE) << "L4S: Fallback base estimate: " << best_estimate.bps() << " bps";
  return best_estimate;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::CreateRateUpdate(Timestamp at_time) const {
  NetworkControlUpdate update;
  
  if (!at_time.IsFinite()) {
    at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }
  
  DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  
  // Validate rate
  if (!current_rate.IsFinite() || current_rate.bps() <= 0) {
    current_rate = DataRate::KilobitsPerSec(300);
  }
  
  // Set target rate
  update.target_rate = TargetTransferRate();
  update.target_rate->at_time = at_time;
  update.target_rate->network_estimate.at_time = at_time;
  update.target_rate->network_estimate.bandwidth = current_rate;
  update.target_rate->network_estimate.loss_rate_ratio = static_cast<float>(last_loss_fraction_);
  update.target_rate->network_estimate.round_trip_time = last_estimated_round_trip_time_;
  update.target_rate->network_estimate.bwe_period = TimeDelta::Millis(500);
  update.target_rate->target_rate = current_rate;
  
  // Set pacer config
  update.pacer_config = PacerConfig();
  update.pacer_config->at_time = at_time;
  update.pacer_config->time_window = TimeDelta::Millis(10);
  update.pacer_config->data_window = current_rate * update.pacer_config->time_window;
  update.pacer_config->pad_window = DataSize::Zero();
  
  return update;
}

void webrtc::L4SNetworkController::MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update, Timestamp at_time) {
  if (!at_time.IsFinite()) {
    at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }
  
  NetworkControlUpdate rate_update = CreateRateUpdate(at_time);
  
  if (rate_update.pacer_config) {
    update->pacer_config = rate_update.pacer_config;
  }
  if (rate_update.target_rate) {
    update->target_rate = rate_update.target_rate;

    // Notify ProbeController of a meaningful bitrate change (>5% shift).
    // Calling SetEstimatedBitrate on every feedback batch would flood
    // probe_controller.cc's "Measured bitrate" log because that log fires
    // unconditionally while state == kWaitingForProbingResult.
    if (probe_controller_) {
      DataRate target_bitrate = rate_update.target_rate->target_rate;
      bool is_first_report = last_reported_bitrate_to_probe_controller_.IsZero();
      bool changed_significantly =
          is_first_report ||
          (std::abs(static_cast<int64_t>(target_bitrate.bps()) -
                    static_cast<int64_t>(
                        last_reported_bitrate_to_probe_controller_.bps())) >
           static_cast<int64_t>(
               0.05 * last_reported_bitrate_to_probe_controller_.bps()));
      if (changed_significantly) {
        // Use kLossLimitedBweIncreasing during recovery so ProbeController
        // records the cause correctly for future RequestProbe decisions.
        BandwidthLimitedCause cause =
            recovery_mode_active_
                ? BandwidthLimitedCause::kLossLimitedBweIncreasing
                : BandwidthLimitedCause::kDelayBasedLimited;
        auto probes = probe_controller_->SetEstimatedBitrate(
            target_bitrate, cause, at_time);
        last_reported_bitrate_to_probe_controller_ = target_bitrate;
        if (!probes.empty()) {
          for (const auto& probe : probes) {
            TimeDelta since_last_probe =
                last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity()
                                              : (at_time - last_probe_time_);
            if (since_last_probe >= config_.probe_interval) {
              update->probe_cluster_configs.push_back(probe);
              last_probe_time_ = at_time;
              StartProbeHold(at_time);
            } else {
              RTC_LOG(LS_VERBOSE)
                  << "L4S: Dropping bitrate-change probe due to global interval gate";
            }
          }
        }
      }
    }
  }
}

void webrtc::L4SNetworkController::AdvanceStateMachine(Timestamp now) {
  if (!now.IsFinite()) {
    now = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }
  if (state_entered_at_.IsInfinite()) {
    state_entered_at_ = now;
  }

  LogStateSnapshot(now);

  std::optional<TransitionDecision> decision = EvaluateStateTransition(now);
  if (decision.has_value()) {
    TransitionToState(decision->next_state, decision->reason, now);
  }
}

TimeDelta webrtc::L4SNetworkController::GetMinimumStateDwell() const {
  TimeDelta effective_rtt =
      last_rtt_.IsFinite() && !last_rtt_.IsZero()
          ? std::clamp(last_rtt_, TimeDelta::Millis(50), TimeDelta::Millis(300))
          : TimeDelta::Millis(100);
  return std::max(kMinimumStateDwellFloor, effective_rtt * 2.0);
}

bool webrtc::L4SNetworkController::CanEnterRecoveryState(Timestamp now) const {
  return recovery_cooldown_until_.IsInfinite() || now >= recovery_cooldown_until_;
}

void webrtc::L4SNetworkController::LogStateSnapshot(Timestamp now) {
  if (!now.IsFinite()) {
    now = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }

  if (!last_state_snapshot_log_.IsInfinite() &&
      (now - last_state_snapshot_log_) < kStateSnapshotLogInterval) {
    return;
  }
  last_state_snapshot_log_ = now;

  const bool discovery_active =
      prague_estimator_ && prague_estimator_->IsDiscoveryModeActive();
  const bool reduction_active =
      prague_estimator_ && prague_estimator_->GetDirectionFlag() == -1;
  TimeDelta dwell_time = state_entered_at_.IsInfinite()
                             ? TimeDelta::Zero()
                             : (now - state_entered_at_);
  TimeDelta min_dwell = GetMinimumStateDwell();
  TimeDelta cooldown_left =
      recovery_cooldown_until_.IsInfinite() || now >= recovery_cooldown_until_
          ? TimeDelta::Zero()
          : (recovery_cooldown_until_ - now);

  RTC_LOG(LS_VERBOSE)
      << "L4S: State snapshot state=" << StateToString(controller_state_)
      << ", discovery=" << discovery_active
      << ", reduction=" << reduction_active
      << ", recovery=" << recovery_mode_active_
      << ", dwell_ms=" << dwell_time.ms()
      << ", min_dwell_ms=" << min_dwell.ms()
      << ", cooldown_left_ms=" << cooldown_left.ms()
      << ", target_bps=" << target_rate_.value_or(DataRate::Zero()).bps()
      << ", acked_bps=" << last_acked_bitrate_.value_or(DataRate::Zero()).bps()
      << ", actual_bps=" << last_actual_bitrate_.bps()
      << ", loss=" << last_loss_fraction_
      << ", rtt_ms=" << (last_rtt_.IsFinite() ? last_rtt_.ms() : -1);
}

std::optional<webrtc::L4SNetworkController::TransitionDecision>
webrtc::L4SNetworkController::EvaluateStateTransition(Timestamp now) const {
  if (!prague_estimator_) {
    return std::nullopt;
  }

  if (!now.IsFinite()) {
    now = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }

  const bool discovery_active = prague_estimator_->IsDiscoveryModeActive();
  const bool reduction_active = prague_estimator_->GetDirectionFlag() == -1;
  const bool recovery_active = recovery_mode_active_;
  const bool dwell_ok =
      state_entered_at_.IsInfinite() ||
      (now - state_entered_at_ >= GetMinimumStateDwell());
  const bool recovery_cooldown_ok = CanEnterRecoveryState(now);

  switch (controller_state_) {
    case ControllerState::kRouteReset:
      if (discovery_active) {
        return TransitionDecision{ControllerState::kSlowStart,
                                  TransitionReason::kInitComplete};
      }
      if (recovery_active) {
        return TransitionDecision{ControllerState::kCongestionRecovery,
                                  TransitionReason::kRecoveryEntered};
      }
      if (reduction_active) {
        return TransitionDecision{ControllerState::kCongestionExperienced,
                                  TransitionReason::kPragueReduction};
      }
      return TransitionDecision{ControllerState::kCongestionAvoidance,
                                TransitionReason::kInitComplete};

    case ControllerState::kSlowStart:
      if (recovery_active && recovery_cooldown_ok && dwell_ok) {
        return TransitionDecision{ControllerState::kCongestionRecovery,
                                  TransitionReason::kRecoveryEntered};
      }
      if (!discovery_active && reduction_active) {
        return TransitionDecision{ControllerState::kCongestionExperienced,
                                  TransitionReason::kPragueReduction};
      }
      if (!discovery_active && dwell_ok) {
        return TransitionDecision{ControllerState::kCongestionAvoidance,
                                  TransitionReason::kDiscoveryExit};
      }
      return std::nullopt;

    case ControllerState::kCongestionAvoidance:
      if (reduction_active) {
        return TransitionDecision{ControllerState::kCongestionExperienced,
                                  TransitionReason::kPragueReduction};
      }
      if (recovery_active && recovery_cooldown_ok && dwell_ok) {
        return TransitionDecision{ControllerState::kCongestionRecovery,
                                  TransitionReason::kRecoveryEntered};
      }
      if (discovery_active && dwell_ok) {
        return TransitionDecision{ControllerState::kSlowStart,
                                  TransitionReason::kDiscoveryActive};
      }
      return std::nullopt;

    case ControllerState::kCongestionExperienced:
      if (recovery_active && recovery_cooldown_ok && dwell_ok) {
        return TransitionDecision{ControllerState::kCongestionRecovery,
                                  TransitionReason::kRecoveryEntered};
      }
      if (discovery_active && dwell_ok) {
        return TransitionDecision{ControllerState::kSlowStart,
                                  TransitionReason::kDiscoveryActive};
      }
      if (!reduction_active && dwell_ok) {
        return TransitionDecision{ControllerState::kCongestionAvoidance,
                                  TransitionReason::kPragueAdditive};
      }
      return std::nullopt;

    case ControllerState::kCongestionRecovery:
      if (!recovery_active && reduction_active) {
        return TransitionDecision{ControllerState::kCongestionExperienced,
                                  TransitionReason::kPragueReduction};
      }
      if (discovery_active && dwell_ok) {
        return TransitionDecision{ControllerState::kSlowStart,
                                  TransitionReason::kDiscoveryActive};
      }
      if (!recovery_active && !reduction_active && dwell_ok) {
        return TransitionDecision{ControllerState::kCongestionAvoidance,
                                  TransitionReason::kRecoveryExited};
      }
      return std::nullopt;
  }
  return std::nullopt;
}

const char* webrtc::L4SNetworkController::StateToString(ControllerState state) {
  switch (state) {
    case ControllerState::kRouteReset:
      return "route_reset";
    case ControllerState::kSlowStart:
      return "slow_start";
    case ControllerState::kCongestionAvoidance:
      return "congestion_avoidance";
    case ControllerState::kCongestionExperienced:
      return "congestion_experienced";
    case ControllerState::kCongestionRecovery:
      return "congestion_recovery";
  }
  return "unknown";
}

const char* webrtc::L4SNetworkController::TransitionReasonToString(
    TransitionReason reason) {
  switch (reason) {
    case TransitionReason::kRouteChange:
      return "route_change";
    case TransitionReason::kInitComplete:
      return "init_complete";
    case TransitionReason::kDiscoveryActive:
      return "discovery_active";
    case TransitionReason::kDiscoveryExit:
      return "discovery_exit";
    case TransitionReason::kPragueReduction:
      return "prague_reduction";
    case TransitionReason::kPragueAdditive:
      return "prague_additive";
    case TransitionReason::kRecoveryEntered:
      return "recovery_entered";
    case TransitionReason::kRecoveryExited:
      return "recovery_exited";
  }
  return "unknown";
}

void webrtc::L4SNetworkController::TransitionToState(ControllerState new_state,
                                                     TransitionReason reason,
                                                     Timestamp at_time) {
  if (!at_time.IsFinite()) {
    at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  }
  if (new_state == controller_state_) {
    if (state_entered_at_.IsInfinite()) {
      state_entered_at_ = at_time;
    }
    return;
  }

  TimeDelta in_previous_state =
      state_entered_at_.IsInfinite() ? TimeDelta::Zero()
                                     : (at_time - state_entered_at_);
  ++state_transition_count_;
  RTC_LOG(LS_INFO) << "L4S: State transition #" << state_transition_count_ << " "
                   << StateToString(controller_state_) << " -> "
                   << StateToString(new_state) << " reason="
                   << TransitionReasonToString(reason)
                   << ", in_prev_state_ms=" << in_previous_state.ms()
                   << ", target_bps="
                   << target_rate_.value_or(DataRate::Zero()).bps()
                   << ", acked_bps="
                   << last_acked_bitrate_.value_or(DataRate::Zero()).bps()
                   << ", actual_bps=" << last_actual_bitrate_.bps()
                   << ", loss=" << last_loss_fraction_;

  controller_state_ = new_state;
  state_entered_at_ = at_time;
}

bool webrtc::L4SNetworkController::HasRecentCongestionSignals(Timestamp now) const {
  return !last_congestion_signal_.IsInfinite() && 
         (now - last_congestion_signal_) < TimeDelta::Seconds(2);
}

bool webrtc::L4SNetworkController::IsEcnFeedbackFresh(Timestamp now) const {
  return HasRecentCongestionSignals(now) || 
         (ecn_supported_ && (now - last_congestion_signal_) < TimeDelta::Seconds(5));
}

void webrtc::L4SNetworkController::UpdateThroughputWindow(const TransportPacketsFeedback& feedback) {
  constexpr TimeDelta kThroughputWindow = TimeDelta::Millis(500);

  // Add new packets using receive_time (receiver NTP clock), consistent with
  // GCC's AcknowledgedBitrateEstimator.
  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.receive_time.IsFinite() && packet.sent_packet.send_time.IsFinite()) {
      throughput_window_.emplace_back(packet.receive_time, packet.sent_packet.size.bytes());
    }
  }

  // Evict using the newest receive_time as reference so the subtraction stays
  // within the receiver clock domain.  The original code used feedback_time
  // (sender clock) which crosses clock domains: any NTP offset between sender
  // and receiver made the difference wrong, causing the window to grow
  // unbounded and turning the 500 ms average into a session-long average.
  if (!throughput_window_.empty()) {
    Timestamp latest_receive = throughput_window_.back().first;
    while (!throughput_window_.empty() &&
           latest_receive - throughput_window_.front().first > kThroughputWindow) {
      throughput_window_.pop_front();
    }
  }
  
  // Calculate throughput over the window
  int64_t window_bytes = 0;
  if (!throughput_window_.empty()) {
    Timestamp window_start = throughput_window_.front().first;
    Timestamp window_end = throughput_window_.back().first;
    for (const auto& entry : throughput_window_) {
      window_bytes += entry.second;
    }
    
    TimeDelta window_interval = window_end - window_start;
    if (window_interval > TimeDelta::Millis(1)) {
      last_actual_bitrate_ = DataRate::BitsPerSec(
          static_cast<int64_t>((window_bytes * 8) / window_interval.seconds<double>()));
    } else {
      last_actual_bitrate_ = DataRate::Zero();
    }
  }
}

void webrtc::L4SNetworkController::LogPeriodicMetrics(Timestamp at_time) {
  if (!metrics_enabled_ || !metrics_collector_) {
    return;
  }
  
  if (at_time - metrics_last_logged_ < kMetricsLoggingInterval) {
    return;
  }
  
  metrics_last_logged_ = at_time;
  
  // Log bandwidth metrics
  DataRate target_rate = target_rate_.value_or(DataRate::Zero());
  metrics_collector_->LogBandwidthMetrics(at_time, target_rate,
                                          last_actual_bitrate_,
                                          last_acked_bitrate_);
  
  // Log delay metrics
  if (last_rtt_.IsFinite()) {
    metrics_collector_->LogDelayMetrics(at_time, last_rtt_, last_rtt_ / 2, TimeDelta::Zero());
  }
  
  // Log loss metrics
  metrics_collector_->LogLossMetrics(at_time, last_loss_fraction_, last_packets_lost_);
  
  // Log periodic summary
  metrics_collector_->LogPeriodicSummary(at_time);
}

bool webrtc::L4SNetworkController::IsApplicationLimited() const {
  if (!alr_detector_) {
    return false;
  }
  return alr_detector_->GetApplicationLimitedRegionStartTime().has_value();
}

void webrtc::L4SNetworkController::UpdateAlrDetector(const TransportPacketsFeedback& feedback) {
  // Track ALR state end so the probe controller can fire an ALR-end probe.
  // Note: OnBytesSent is fed from OnSentPacket, which drives the ALR detector.
  if (alr_detector_ && probe_controller_) {
    std::optional<int64_t> alr_start_time =
        alr_detector_->GetApplicationLimitedRegionStartTime();
    if (previously_in_alr_ && !alr_start_time.has_value()) {
      // ALR just ended – tell ProbeController so it can trigger an ALR probe.
      probe_controller_->SetAlrEndedTimeMs(feedback.feedback_time.ms());
      if (acked_estimator_) {
        acked_estimator_->SetAlrEndedTime(feedback.feedback_time);
      }
      RTC_LOG(LS_INFO) << "L4S: ALR ended, notifying ProbeController at "
                       << feedback.feedback_time.ms() << " ms";
    }
    previously_in_alr_ = alr_start_time.has_value();
  }
}

bool webrtc::L4SNetworkController::CheckProbeAndPragueConvergence(Timestamp now) const {
  if (!prague_estimator_ || !bandwidth_fusion_) {
    return false;
  }

  auto sources = bandwidth_fusion_->GetCurrentSources();
  DataRate prague_rate = prague_estimator_->GetCurrentEstimate();
  DataRate probe_rate = sources.probe_estimate;

  // Need both estimates to be valid and recent
  if (probe_rate <= DataRate::Zero() || prague_rate <= DataRate::Zero()) {
    return false;
  }

  if (sources.probe_confidence < 0.7 || !IsRecentlyUpdated(sources.last_probe_update, now)) {
    return false;
  }

  // Check convergence: estimates within 10% of each other
  double rate_ratio = std::min(prague_rate.bps(), probe_rate.bps()) / 
                     static_cast<double>(std::max(prague_rate.bps(), probe_rate.bps()));

  bool converged = rate_ratio >= 0.9;  // 10% tolerance

  if (converged) {
    RTC_LOG(LS_INFO) << "L4S: Prague-Probe convergence detected - Prague: " 
                     << prague_rate.bps() << " bps, Probe: " << probe_rate.bps() 
                     << " bps, ratio: " << rate_ratio;
  }

  return converged;
}

bool webrtc::L4SNetworkController::ShouldExitDiscoveryMode(Timestamp now) const {
  if (!prague_estimator_ || !prague_estimator_->IsDiscoveryModeActive()) {
    return false;
  }

  // Exit if CE marks detected (already handled in Prague estimator)
  if (prague_estimator_->GetDirectionFlag() == -1) {
    return true;
  }

  // Exit if Prague and Probe converged on capacity
  if (CheckProbeAndPragueConvergence(now)) {
    return true;
  }

  // Fallback: Exit at higher rate threshold (10 Mbps instead of 5 Mbps)
  DataRate current_rate = prague_estimator_->GetCurrentEstimate();
  if (current_rate.bps() >= 10000000) {
    RTC_LOG(LS_INFO) << "L4S: Exiting discovery mode - fallback rate threshold (10 Mbps) reached";
    return true;
  }

  return false;
}

void webrtc::L4SNetworkController::InitiateRecoveryProbing(Timestamp now, NetworkControlUpdate* update) {
  if (!probe_controller_) {
    return;
  }

  // Get current best estimate for probe rate calculation
  DataRate current_estimate = target_rate_.value_or(DataRate::KilobitsPerSec(300));

  // Recovery is intentionally conservative for CE-sensitive AQMs.
  double recovery_multiplier = config_.recovery_probe_multiplier;
  if (IsApplicationLimited()) {
    recovery_multiplier = config_.recovery_alr_probe_multiplier;
  }

  // Calculate target probe rate
  DataRate probe_rate = current_estimate * recovery_multiplier;
  if (max_target_rate_) {
    probe_rate = std::min(probe_rate, *max_target_rate_);
  }

  // Skip recovery probing at very low rates where queue/scheduler effects
  // dominate and probe samples are typically misleading.
  if (current_estimate < config_.recovery_probe_min_rate) {
    RTC_LOG(LS_VERBOSE) << "L4S: Skipping recovery probe due to low target rate: "
                        << current_estimate.bps() << " < "
                        << config_.recovery_probe_min_rate.bps();
    return;
  }

  // Ensure probes are materially above current estimate; otherwise they are
  // not useful for capacity seeking and only add CE risk.
  DataRate min_useful_probe_rate = current_estimate * config_.min_useful_probe_uplift;
  if (probe_rate < min_useful_probe_rate) {
    RTC_LOG(LS_VERBOSE) << "L4S: Skipping recovery probe due to insufficient uplift: "
                        << probe_rate.bps() << " < " << min_useful_probe_rate.bps();
    return;
  }

  // Bootstrap recovery probing once, then avoid repeated reset->startup bursts.
  DataRate clamped_min = min_target_rate_.value_or(DataRate::KilobitsPerSec(30));
  DataRate clamped_max = max_target_rate_.value_or(DataRate::KilobitsPerSec(100000));
  if (!clamped_max.IsFinite()) {
    clamped_max = DataRate::KilobitsPerSec(100000);
  }
  // Ensure max >= current so the probe target is meaningful.
  clamped_max = std::max(clamped_max, current_estimate);

  std::vector<ProbeClusterConfig> probes;
  if (!recovery_probe_bootstrapped_) {
    probe_controller_->Reset(now);
    probes = probe_controller_->SetBitrates(
        clamped_min, current_estimate, clamped_max, now);
    recovery_probe_bootstrapped_ = true;
    last_reported_bitrate_to_probe_controller_ = current_estimate;
  } else {
    probes = probe_controller_->SetEstimatedBitrate(
        current_estimate, BandwidthLimitedCause::kLossLimitedBweIncreasing,
        now);
    if (probes.empty()) {
      probes = probe_controller_->RequestProbe(now);
    }
    last_reported_bitrate_to_probe_controller_ = current_estimate;
  }

  if (!probes.empty()) {
    update->probe_cluster_configs.insert(update->probe_cluster_configs.end(),
                                         probes.begin(), probes.end());
    StartProbeHold(now);
    RTC_LOG(LS_INFO) << "L4S: Recovery probe initiated - "
                     << probes.size() << " clusters from "
                     << current_estimate.bps() << " bps "
                     << "(max=" << clamped_max.bps() << " bps)";
  } else {
    // Network not yet available inside ProbeController; probe will fire once
    // OnNetworkAvailability is forwarded.
    RTC_LOG(LS_VERBOSE) << "L4S: Recovery probe deferred - network not available";
  }
  (void)probe_rate;         // computed above for max clamping
  (void)recovery_multiplier;
}

void webrtc::L4SNetworkController::HandleRecoveryDetection(int ect_count, int ce_count, Timestamp now) {
  // Handle recovery mode detection based on clean ECT1 packets
  if (ce_count == 0 && ect_count > 0) {
    if (clean_ect_run_start_.IsInfinite()) {
      clean_ect_run_start_ = now;
    }
    consecutive_clean_packets_ += ect_count;
    
    // Trigger recovery mode if enough clean packets seen, not already in discovery,
    // and the post-convergence cooldown has expired.
    bool cooldown_expired = recovery_cooldown_until_.IsInfinite() ||
                            now >= recovery_cooldown_until_;
    // Dynamic recovery threshold: the fixed count of 20 represents very
    // different durations at different bitrates (e.g. 45 ms at 5 Mbps vs
    // 444 ms at 500 Kbps).  Instead, compute the threshold as the number of
    // packets that fit in 1.5 RTTs at the current rate so that recovery always
    // waits at least 1.5 round-trips before probing, regardless of bitrate.
    //
    //   effective_rtt   = clamp(last_rtt_, 50 ms, 300 ms)
    //   target_duration = 1.5 × effective_rtt
    //   threshold       = clamp(packets/s × target_duration, recovery_min_clean_packets, 500)
    //
    // kRecoveryPacketThreshold (20) acts as the floor so the condition is
    // never trivially satisfied on very low-rate paths.
    TimeDelta effective_rtt =
        last_rtt_.IsFinite()
            ? std::clamp(last_rtt_,
                         TimeDelta::Millis(50),
                         TimeDelta::Millis(300))
            : TimeDelta::Millis(100);  // safe default until RTT is measured
    double target_duration_s = effective_rtt.seconds<double>() * 1.5;
    double rate_bps = target_rate_.has_value()
                          ? static_cast<double>(target_rate_->bps())
                          : 2'000'000.0;  // 2 Mbps safe default
    double packets_per_sec = rate_bps / (1400.0 * 8.0);
    int dynamic_threshold = static_cast<int>(packets_per_sec * target_duration_s);
    int recovery_threshold = std::clamp(
      dynamic_threshold,
      std::max(config_.recovery_min_clean_packets, kRecoveryPacketThreshold),
      500);
    TimeDelta clean_duration = now - clean_ect_run_start_;
    TimeDelta min_clean_duration =
      std::max(config_.recovery_min_clean_duration, effective_rtt * 2.0);
    bool clean_duration_ok = clean_duration >= min_clean_duration;
    bool rate_ok = target_rate_.value_or(DataRate::Zero()) >=
             config_.recovery_probe_min_rate;
    if (consecutive_clean_packets_ >= recovery_threshold &&
      clean_duration_ok &&
      rate_ok &&
        !recovery_mode_active_ &&
        !prague_estimator_->IsDiscoveryModeActive() &&
        cooldown_expired) {

      recovery_mode_active_ = true;
      recovery_probe_bootstrapped_ = false;
      recovery_start_time_ = now;

      RTC_LOG(LS_INFO) << "L4S: Entering recovery mode after " << consecutive_clean_packets_
                       << " clean ECT packets (threshold=" << recovery_threshold
                       << ", rtt=" << effective_rtt.ms() << "ms"
                       << ", clean_ms=" << clean_duration.ms()
                       << ", min_clean_ms=" << min_clean_duration.ms()
                       << ", rate=" << static_cast<int>(rate_bps / 1000) << "kbps)";
    }
  } else if (ce_count > 0) {
    // Reset clean packet count on congestion
    consecutive_clean_packets_ = 0;
    clean_ect_run_start_ = Timestamp::MinusInfinity();
    
    // Exit recovery mode on congestion
    if (recovery_mode_active_) {
      recovery_mode_active_ = false;
      recovery_probe_bootstrapped_ = false;
      recovery_cooldown_until_ = now + config_.recovery_reentry_cooldown;
      RTC_LOG(LS_INFO) << "L4S: Exiting recovery mode due to CE marks";
    }
  }
  
  // Check recovery mode exit conditions
  if (recovery_mode_active_) {
    TimeDelta recovery_duration = now - recovery_start_time_;
    
    // Exit conditions:
    // 1. Maximum recovery duration exceeded (10 seconds)
    // 2. Prague and probe converged at higher capacity
    if (recovery_duration > TimeDelta::Seconds(10) || 
        CheckProbeAndPragueConvergence(now)) {
      
      bool by_convergence = !( recovery_duration > TimeDelta::Seconds(10) );
      recovery_mode_active_ = false;
      recovery_probe_bootstrapped_ = false;
      consecutive_clean_packets_ = 0;
      clean_ect_run_start_ = Timestamp::MinusInfinity();

      if (by_convergence) {
        // Impose a cooldown so the controller doesn't oscillate back into
        // recovery immediately on a stable, low-congestion network.
        recovery_cooldown_until_ = now + kRecoveryCooldown;
        RTC_LOG(LS_INFO) << "L4S: Exiting recovery mode - convergence achieved, "
                         << "cooldown until +" << kRecoveryCooldown.seconds<int>() << "s";
      } else {
        recovery_cooldown_until_ = now + config_.recovery_reentry_cooldown;
        RTC_LOG(LS_INFO) << "L4S: Exiting recovery mode - timeout";
      }
    }
  }
}

bool webrtc::L4SNetworkController::IsRecentlyUpdated(Timestamp last_update, Timestamp now) const {
  // Check both timestamps for infinity before arithmetic to prevent crash
  if (last_update.IsInfinite() || now.IsInfinite()) {
    return false;
  }
  return (now - last_update) < TimeDelta::Seconds(10);
}

}  // namespace webrtc
