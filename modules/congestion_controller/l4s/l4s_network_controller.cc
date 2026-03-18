#include "modules/congestion_controller/l4s/l4s_network_controller.h"

#include <algorithm>
#include <cmath>
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
    
    // DISABLED: Probe constraint is disabled in L4S to prevent rate limiting during discovery
    // The probe ceiling was causing convergence failures and excessive rate dips
    // Keeping this code commented out for future use if needed
    
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

void webrtc::L4SBandwidthFusion::UpdateAlrEstimate(DataRate estimate, double confidence, Timestamp now) {
  RTC_LOG(LS_VERBOSE) << "L4S: Updating ALR estimate to " << estimate.bps() << " bps with confidence " << confidence;
  sources_.alr_estimate = estimate;
  sources_.alr_confidence = confidence;
  sources_.last_alr_update = now;
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
  
  // Probe estimate gets reduced weight due to underestimation tendency
  if (sources_.probe_confidence > 0.5 && IsRecentlyUpdated(sources_.last_probe_update, now)) {
    probe_weighted = sources_.probe_estimate;
    probe_weight = recovery_mode ? 0.65 : 0.60;  // Reduced weight due to probe underestimation
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

webrtc::L4SMetricsCollector::L4SMetricsCollector(test::MetricsLogger* logger, 
                                                    const std::string& test_case_name,
                                                    Clock* clock)
    : logger_(logger), test_case_name_(test_case_name), clock_(clock) {
  RTC_CHECK(logger_);
  RTC_CHECK(clock_);
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
        logger_to_use, config_.test_case_name, &env_.clock());
  }
  
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
  if (config_.enable_delay_estimation) {
    delay_estimator_ = std::make_unique<DelayBasedBwe>(&env_.field_trials(), nullptr, nullptr);
  }
  
  if (config_.enable_acked_estimation) {
    acked_estimator_ = std::make_unique<AcknowledgedBitrateEstimator>(&env_.field_trials());
  }
  
  if (config_.enable_alr_detection) {
    alr_detector_ = std::make_unique<AlrDetector>(&env_.field_trials());
  }
  
  RTC_LOG(LS_INFO) << "L4S: Initialized bandwidth estimators - "
                   << "Delay: " << (delay_estimator_ ? "enabled" : "disabled")
                   << ", Acked: " << (acked_estimator_ ? "enabled" : "disabled")
                   << ", ALR: " << (alr_detector_ ? "enabled" : "disabled");
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnNetworkAvailability(NetworkAvailability msg) {
  NetworkControlUpdate update;
  // Probing disabled - return empty update
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnNetworkRouteChange(NetworkRouteChange msg) {
  NetworkControlUpdate update;
  
  RTC_LOG(LS_INFO) << "L4S: OnNetworkRouteChange called";
  
  // Reset ECN support detection on network change
  ecn_supported_ = false;
  ecn_capable_network_ = false;
  ect_count_ = 0;
  ce_count_ = 0;
  last_congestion_signal_ = Timestamp::MinusInfinity();
  
  // Update rate constraints
  if (msg.constraints.starting_rate) {
    starting_rate_ = msg.constraints.starting_rate;
    target_rate_ = starting_rate_;
  }
  min_target_rate_ = msg.constraints.min_data_rate;
  max_target_rate_ = msg.constraints.max_data_rate;
  
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnProcessInterval(ProcessInterval msg) {
  NetworkControlUpdate update;
  
  // Log periodic metrics
  LogPeriodicMetrics(msg.at_time);

  // Update time-based decay in Prague estimator
  prague_estimator_->OnTimeUpdate(msg.at_time);
  
  // Fuse all bandwidth estimates and update target rate
  DataRate fused_rate = FuseBandwidthEstimates(msg.at_time);
  target_rate_ = fused_rate;
  
  // Create rate update
  MaybeTriggerOnNetworkChanged(&update, msg.at_time);
  
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
  // Probing disabled - return empty update
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
  
  // Handle periodic probing
  HandlePeriodicProbing(msg.feedback_time, &update);
  
  // Fuse bandwidth estimates and update target rate
  DataRate fused_rate = FuseBandwidthEstimates(msg.feedback_time);
  target_rate_ = fused_rate;
  
  // Update throughput calculation
  UpdateThroughputWindow(msg);
  
  // Create rate update
  MaybeTriggerOnNetworkChanged(&update, msg.feedback_time);
  
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
  
  // 1. Update non-ECN estimators first (delay, acked, probe)
  if (delay_estimator_) {
    UpdateDelayBasedEstimator(feedback);
  }
  
  if (acked_estimator_) {
    UpdateAckedBitrateEstimator(feedback);
  }
  
  // Probe controller removed - using pure ECN-based discovery
  
  // 2. Get initial fused estimate (without ECN input)
  DataRate base_fused_rate = GetBaseFusedEstimate(feedback.feedback_time);
  
  // 3. Update Prague ECN controller with the base fused rate (not application limited)
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

  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1 || packet.ecn == EcnMarking::kCe) {
      new_ect_count++;
    }
    if (packet.ecn == EcnMarking::kCe) {
      new_ce_count++;
      last_congestion_signal_ = feedback.feedback_time;
    }
  }
  
  // Log ECN feedback summary
  if (new_ect_count > 0) {
    double ce_ratio = new_ce_count > 0 ? static_cast<double>(new_ce_count) / (new_ect_count + new_ce_count) : 0.0;
    RTC_LOG(LS_VERBOSE) << "L4S: ECN FEEDBACK - ECT: " << new_ect_count << ", CE: " << new_ce_count 
                        << ", Ratio: " << (ce_ratio * 100) << "%, "
                        << "Current rate: " << (current_fused_rate.bps() / 1e6) << " Mbps";
  }
  
  // Update ECN support detection
  if (new_ect_count > 0 || new_ce_count > 0) {
    ecn_supported_ = true;
    ecn_capable_network_ = true;
    
    // Track ECN activity for confidence calculation
    prague_estimator_->UpdateEcnActivity(feedback.feedback_time);
  }
  
  // Update Prague estimator with CE ratio using intelligent bottleneck detection
  if (new_ect_count + new_ce_count > 0) {
    double ce_ratio = static_cast<double>(new_ce_count) / (new_ect_count + new_ce_count);
    
    // When CE marks appear after plateau, reset plateau to allow dynamic re-discovery
    if (new_ce_count > 0 && in_acked_plateau_) {
      RTC_LOG(LS_VERBOSE) << "L4S: CE marks detected during acked plateau - enabling re-discovery";
      ResetAckedPlateau();
    }
    
    // Always use Prague's own estimate as the input rate.
    // This prevents feedback loops where low fused estimates (from other estimators)
    // feed back and collapse Prague's rate. CE ratio will still apply congestion control.
    DataRate prague_input_rate = prague_estimator_->GetCurrentEstimate();
    if (prague_estimator_->IsDiscoveryModeActive()) {
      RTC_LOG(LS_VERBOSE) << "L4S: Discovery mode - using Prague's own estimate: " << prague_input_rate.bps() 
                       << " bps (ce_ratio=" << ce_ratio << ")";
    } else {
      RTC_LOG(LS_VERBOSE) << "L4S: Steady state - using Prague's own estimate as baseline: " << prague_input_rate.bps() 
                       << " bps (fused rate was: " << current_fused_rate.bps() << ", ce_ratio=" << ce_ratio << ")";
    }
    
    prague_estimator_->UpdateFromCongestionSignal(prague_input_rate, ce_ratio, feedback.feedback_time);
    
    // Log Prague's response after processing CE feedback
    DataRate prague_estimate_after_update = prague_estimator_->GetCurrentEstimate();
    RTC_LOG(LS_VERBOSE) << "L4S: PRAGUE RESPONSE - CE_ratio=" << (ce_ratio * 100) << "%, "
                        << "Input: " << (prague_input_rate.bps() / 1e6) << " Mbps, "
                        << "Output: " << (prague_estimate_after_update.bps() / 1e6) << " Mbps, "
                        << "Direction: " << (prague_estimator_->GetDirectionFlag() == 1 ? "ADD_INC" : "REDUC");
    
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
  
  // Update counters
  ect_count_ = new_ect_count;
  ce_count_ = new_ce_count;
}

webrtc::DataRate webrtc::L4SNetworkController::DetermineBottleneckAwareTarget(DataRate fused_rate, Timestamp now) {
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

void webrtc::L4SNetworkController::UpdateDelayBasedEstimator(const TransportPacketsFeedback& feedback) {
  // Delay estimation disabled for L4S - ECN marks are the primary signal
  // L4S philosophy: explicit congestion signals (CE marks) replace delay inference
  return;
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
  
  // Detect if acked rate is plateauing while Prague continues to climb
  DetectAckedRatePlateau(feedback.feedback_time);
  
  // Diagnostic: compare acked rate against Prague estimate to detect delivery issues
  if (prague_estimator_) {
    DataRate prague_est = prague_estimator_->GetCurrentEstimate();
    double ratio = prague_est.IsZero() ? 0.0 : acked_bitrate->bps() / prague_est.bps();
    if (ratio < 0.5) {
      // Acked rate is less than 50% of Prague estimate - significant mismatch
      RTC_LOG(LS_VERBOSE) << "L4S: EFFICIENCY ALERT - Acked rate " << (acked_bitrate->bps() / 1e6) 
                          << " Mbps is only " << (ratio * 100) << "% of Prague estimate " 
                          << (prague_est.bps() / 1e6) << " Mbps (Possible packet loss or asymmetric congestion)";
    }
  }
}

void webrtc::L4SNetworkController::ProcessRealProbeResults(const TransportPacketsFeedback& feedback) {
  // Probing disabled - no probe processing
}

std::optional<webrtc::DataRate> webrtc::L4SNetworkController::GetLastProbeResult() {
  // Probing disabled
  return std::nullopt;
}

void webrtc::L4SNetworkController::HandlePeriodicProbing(Timestamp now, NetworkControlUpdate* update) {
  // DISABLED: Probing is disabled in L4S to prevent bandwidth overshooting during discovery.
  // All probe requests are blocked to allow Prague to converge naturally to bottleneck capacity.
  RTC_LOG(LS_VERBOSE) << "L4S: Probing is disabled (all probes blocked)";
}

double webrtc::L4SNetworkController::CalculateEcnConfidence(Timestamp now) const {
  return prague_estimator_->GetConfidence(now);
}

double webrtc::L4SNetworkController::CalculateDelayConfidence(Timestamp now) const {
  if (!delay_estimator_ || !last_rtt_.IsFinite()) {
    return 0.0;
  }
  
  // High confidence if RTT is stable
  if (IsRttStable()) {
    return 0.8;
  }
  return 0.5;
}

double webrtc::L4SNetworkController::CalculateProbeConfidence(Timestamp now) const {
  // Probing disabled - return 0 confidence
  return 0.0;
}

double webrtc::L4SNetworkController::CalculateAckedConfidence(Timestamp now) const {
  if (!acked_estimator_) {
    return 0.0;
  }
  
  // Moderate confidence in acknowledged bitrate
  return 0.6;
}

webrtc::DataRate webrtc::L4SNetworkController::FuseBandwidthEstimates(Timestamp now) {
  bool discovery_active = prague_estimator_ && prague_estimator_->IsDiscoveryModeActive();
  
  // Check if we should exit discovery mode based on convergence
  if (discovery_active && ShouldExitDiscoveryMode(now)) {
    DataRate prague_at_exit = prague_estimator_->GetCurrentEstimate();
    RTC_LOG(LS_VERBOSE) << "L4S: EXIT DISCOVERY MODE - ECN-based convergence at rate: " 
                        << (prague_at_exit.bps() / 1e6) << " Mbps";
    prague_estimator_->ExitDiscoveryMode("ECN-based convergence");
    discovery_active = false;
  }
  
  // Use Prague ECN-based fusion (no probe constraints, no recovery mode boost)
  DataRate fused_rate = bandwidth_fusion_->GetFusedEstimateWithMode(now, discovery_active, false);
  
  // During discovery mode, use Prague's estimate directly
  if (discovery_active) {
    DataRate prague_rate = prague_estimator_->GetCurrentEstimate();
    int dir_flag = prague_estimator_->GetDirectionFlag();
    double alpha = prague_estimator_->GetAlpha();
    RTC_LOG(LS_VERBOSE) << "L4S: DISCOVERY MODE - Prague: " << (prague_rate.bps() / 1e6) << " Mbps, "
                        << "Mode: " << (dir_flag == 1 ? "ADD_INC" : "REDUC")
                        << ", Alpha: " << alpha;
    fused_rate = prague_rate;
  } else {
    // Not in discovery - log the fused rate and its sources
    auto sources = bandwidth_fusion_->GetCurrentSources();
    RTC_LOG(LS_VERBOSE) << "L4S: STEADY STATE - Fused: " << (fused_rate.bps() / 1e6) << " Mbps, "
                        << "ECN: " << (sources.ecn_estimate.bps() / 1e6) << " Mbps, "
                        << "Delay: " << (sources.delay_estimate.bps() / 1e6) << " Mbps, "
                        << "Acked: " << (sources.acked_estimate.bps() / 1e6) << " Mbps";
  }
  
  // Apply rate constraints
  DataRate original_fused = fused_rate;
  if (min_target_rate_ && fused_rate < *min_target_rate_) {
    RTC_LOG(LS_VERBOSE) << "L4S: Applying min constraint: " << (fused_rate.bps() / 1e6) 
                        << " -> " << (min_target_rate_->bps() / 1e6) << " Mbps";
    fused_rate = *min_target_rate_;
  }
  // Final safety: enforce absolute minimum of 20 kbps to prevent pacer crashes
  fused_rate = std::max(fused_rate, DataRate::KilobitsPerSec(20));
  if (max_target_rate_ && fused_rate > *max_target_rate_) {
    RTC_LOG(LS_VERBOSE) << "L4S: Applying max constraint: " << (fused_rate.bps() / 1e6) 
                        << " -> " << (max_target_rate_->bps() / 1e6) << " Mbps";
    fused_rate = *max_target_rate_;
  }
  
  // IMPORTANT: Sanity check - don't let estimate exceed actual delivery by too much
  // If acked rate exists and is much lower than fused rate, constrain fused estimate
  // This prevents the optimistic ECN estimate from overshooting when actual network
  // capacity is lower (e.g., due to packet loss or asymmetric congestion)
  if (last_acked_bitrate_.has_value()) {
    // Allow fused_rate to be 1.5x the acked rate (headroom for growth)
    DataRate acked_ceiling = last_acked_bitrate_.value() * 1.5;
    if (fused_rate > acked_ceiling) {
      RTC_LOG(LS_VERBOSE) << "L4S: ACKED RATE SANITY CHECK - Fused: " << (fused_rate.bps() / 1e6) 
                          << " Mbps exceeds 1.5x acked rate (" << (last_acked_bitrate_.value().bps() / 1e6) 
                          << " Mbps) - capping to " << (acked_ceiling.bps() / 1e6) << " Mbps";
      fused_rate = acked_ceiling;
    }
  }
  
  if (fused_rate.bps() != original_fused.bps()) {
    RTC_LOG(LS_INFO) << "L4S: RATE UPDATE - Before constraints: " << (original_fused.bps() / 1e6) 
                     << " Mbps, After: " << (fused_rate.bps() / 1e6) << " Mbps";
  }
  
  return fused_rate;
}

webrtc::DataRate webrtc::L4SNetworkController::GetBaseFusedEstimate(Timestamp now) {
  // Get fused estimate from non-ECN sources only (delay, probe, acked)
  // This provides the base capacity estimate before Prague applies AI/MD
  
  L4SBandwidthFusion::BandwidthSources temp_sources = bandwidth_fusion_->GetCurrentSources();
  
  // Temporarily zero out ECN estimate for base fusion
  temp_sources.ecn_estimate = DataRate::Zero();
  temp_sources.ecn_confidence = 0.0;
  
  // Use weighted combination of delay, probe, acked, and ALR estimates as base
  double total_weight = temp_sources.delay_confidence + 
                       temp_sources.probe_confidence + 
                       temp_sources.acked_confidence +
                       temp_sources.alr_confidence;
  if (total_weight > 0.01) {  // Very low threshold - almost always use weighted combination
    DataRate weighted_estimate = 
        (temp_sources.delay_estimate * temp_sources.delay_confidence + 
         temp_sources.probe_estimate * temp_sources.probe_confidence +
         temp_sources.acked_estimate * temp_sources.acked_confidence +
         temp_sources.alr_estimate * temp_sources.alr_confidence) / total_weight;
    
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
  
  if (temp_sources.alr_confidence > best_confidence) {
    best_estimate = temp_sources.alr_estimate;
    best_confidence = temp_sources.alr_confidence;
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

    // Probe controller removed - using pure ECN-based discovery
  }
}

bool webrtc::L4SNetworkController::IsL4SActive() const {
  return ecn_supported_ && ecn_capable_network_;
}

bool webrtc::L4SNetworkController::HasRecentCongestionSignals(Timestamp now) const {
  return !last_congestion_signal_.IsInfinite() && 
         (now - last_congestion_signal_) < TimeDelta::Seconds(2);
}

bool webrtc::L4SNetworkController::IsEcnFeedbackFresh(Timestamp now) const {
  return HasRecentCongestionSignals(now) || 
         (ecn_supported_ && (now - last_congestion_signal_) < TimeDelta::Seconds(5));
}

bool webrtc::L4SNetworkController::EstimatesAreDiverging() const {
  // Simple check for estimate divergence
  auto sources = bandwidth_fusion_->GetCurrentSources();
  if (sources.ecn_estimate > DataRate::Zero() && sources.delay_estimate > DataRate::Zero()) {
    double ratio = sources.ecn_estimate.bps() / static_cast<double>(sources.delay_estimate.bps());
    return ratio > 2.0 || ratio < 0.5;  // 2x divergence threshold
  }
  return false;
}

bool webrtc::L4SNetworkController::IsRttStable() const {
  // Simplified RTT stability check
  return last_rtt_.IsFinite() && last_rtt_ < TimeDelta::Millis(100);
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
  // Track ALR state for logging/metrics only (probing disabled in L4S)
  if (alr_detector_) {
    std::optional<int64_t> alr_start_time =
        alr_detector_->GetApplicationLimitedRegionStartTime();
    if (previously_in_alr_ && !alr_start_time.has_value()) {
      if (acked_estimator_) {
        acked_estimator_->SetAlrEndedTime(feedback.feedback_time);
      }
      RTC_LOG(LS_VERBOSE) << "L4S: ALR ended at " << feedback.feedback_time.ms() << " ms";
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

  // Fallback: Exit at high rate threshold (90 Mbps) - allow discovery to reach near-capacity before exiting
  // This ensures Prague actually discovers bottleneck via CE marks instead of artificial threshold
  DataRate current_rate = prague_estimator_->GetCurrentEstimate();
  if (current_rate.bps() >= 90000000) {  // 90 Mbps
    RTC_LOG(LS_INFO) << "L4S: Exiting discovery mode - fallback rate threshold (90 Mbps) reached. "
                     << "Final discovered rate: " << (current_rate.bps() / 1e6) << " Mbps";
    return true;
  }

  return false;
}

bool webrtc::L4SNetworkController::IsRecentlyUpdated(Timestamp last_update, Timestamp now) const {
  // Check both timestamps for infinity before arithmetic to prevent crash
  if (last_update.IsInfinite() || now.IsInfinite()) {
    return false;
  }
  return (now - last_update) < TimeDelta::Seconds(10);
}

void webrtc::L4SNetworkController::DetectAckedRatePlateau(Timestamp now) {
  if (!last_acked_bitrate_.has_value() || !prague_estimator_) {
    return;
  }

  DataRate acked_rate = *last_acked_bitrate_;

  // Add to history
  acked_rate_history_.push_back(acked_rate);
  if (acked_rate_history_.size() > kAckedRateHistorySize) {
    acked_rate_history_.pop_front();
  }

  // Need enough history to detect trend
  if (acked_rate_history_.size() < 3) {
    ResetAckedPlateau();
    return;
  }

  // Check if acked rate has plateaued
  // Plateau = all recent samples within 2% of median
  std::vector<DataRate> sorted_history(acked_rate_history_.begin(), acked_rate_history_.end());
  std::sort(sorted_history.begin(), sorted_history.end(), 
            [](const DataRate& a, const DataRate& b) { return a.bps() < b.bps(); });
  
  DataRate median_acked = sorted_history[sorted_history.size() / 2];
  if (median_acked.IsZero()) {
    ResetAckedPlateau();
    return;
  }

  // Check if all recent samples are within threshold of median
  bool all_flat = true;
  for (const auto& sample : acked_rate_history_) {
    double variation = std::abs(sample.bps() - median_acked.bps()) / median_acked.bps();
    if (variation > kAckedRatePlateauThreshold) {
      all_flat = false;
      break;
    }
  }

  if (all_flat) {
    // Acked rate is flat
    plateau_consecutive_updates_++;
    
    if (plateau_consecutive_updates_ >= kPlateauThresholdUpdates) {
      // Plateau confirmed
      if (!in_acked_plateau_) {
        in_acked_plateau_ = true;
        plateau_detected_at_acked_rate_ = median_acked;
        RTC_LOG(LS_VERBOSE) << "L4S: ACKED RATE PLATEAU DETECTED - Rate: " 
                            << (median_acked.bps() / 1e6) << " Mbps (stable "
                            << plateau_consecutive_updates_ << " updates)";
      }
      // Now check if Prague is climbing while acked is flatlined
      HandleAckedPlateau(now);
    }
  } else {
    // Acked rate is growing - plateau broken
    if (in_acked_plateau_) {
      RTC_LOG(LS_VERBOSE) << "L4S: Acked rate plateau broken - rate increasing from " 
                          << (median_acked.bps() / 1e6) << " Mbps";
      ResetAckedPlateau();
    }
  }
}

void webrtc::L4SNetworkController::HandleAckedPlateau(Timestamp now) {
  if (!in_acked_plateau_ || !prague_estimator_) {
    return;
  }

  DataRate prague_rate = prague_estimator_->GetCurrentEstimate();
  DataRate plateau_acked = plateau_detected_at_acked_rate_.value_or(DataRate::Zero());

  if (plateau_acked.IsZero()) {
    return;
  }

  // If Prague is climbing above the plateau rate, apply brake
  DataRate ceiling = plateau_acked * 1.2;  // Allow 20% headroom for temporary growth
  
  if (prague_rate > ceiling && prague_estimator_->GetDirectionFlag() == 1) {
    // Prague is in additive mode and climbing above acked ceiling
    // Force it to stop or reduce to respect acked rate limit
    RTC_LOG(LS_VERBOSE) << "L4S: PLATEAU BRAKE - Prague climbing to " 
                        << (prague_rate.bps() / 1e6) << " Mbps while acked capped at "
                        << (plateau_acked.bps() / 1e6) << " Mbps. Freezing Prague at "
                        << (ceiling.bps() / 1e6) << " Mbps";
    
    // Freeze Prague's internal rate at ceiling to prevent further growth during plateau
    prague_estimator_->SetCurrentEstimate(ceiling);
  }
}

void webrtc::L4SNetworkController::ResetAckedPlateau() {
  in_acked_plateau_ = false;
  plateau_consecutive_updates_ = 0;
  plateau_detected_at_acked_rate_ = std::nullopt;
}

}  // namespace webrtc
