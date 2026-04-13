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

// 


//time driven AI

void webrtc::PragueCapacityEstimator::UpdateFromCongestionSignal(DataRate current_rate, double ce_ratio, Timestamp current_time) {
  last_feedback_time_ = current_time; // Track the last time we heard from the network

  if (ce_ratio > 0.0) {  // CE-marked packets detected
    non_ce_packet_count_ = 0;
    if (discovery_mode_active_ && !first_ce_mark_detected_) {
      discovery_mode_active_ = false;
      first_ce_mark_detected_ = true;
      // RTC_LOG(LS_INFO) << "Prague: Exiting discovery mode - first CE mark detected (ce_ratio=" 
      //                  << ce_ratio << ")";
    }
    
    // --- PILLAR 2A: WebRTC-Tuned Gain ---
    // RFC 9330 standard gain is 1/16, but WebRTC batches feedback. 
    // Use 1/8 to make alpha grow fast enough to matter.
    constexpr double g = 1.0 / 8.0; 
    alpha_ = (1.0 - g) * alpha_ + g * ce_ratio;


    double dynamic_floor = 0.80; // Default safe encoder limit

    if (current_rtt_.IsFinite() && baseline_rtt_.IsFinite()) {
        TimeDelta rtt_delta = current_rtt_ - baseline_rtt_;
        if (rtt_delta > TimeDelta::Zero()) {
            // Map the bloat to a penalty factor (0.0 to 1.0)
            // 150.0ms represents a total 10:1 path collapse.
            double penalty_factor = std::min(rtt_delta.ms() / 150.0, 1.0);
            
            // Slide the floor: 0.80 - (0.60 * penalty)
            // e.g., 75ms bloat = 0.5 penalty = 0.50 floor
            dynamic_floor = 0.80 - (0.60 * penalty_factor);
            
            if (penalty_factor > 0.1) {
                RTC_LOG(LS_VERBOSE) << "L4S Circuit Breaker: RTT bloat " << rtt_delta.ms() 
                                    << "ms. Dynamic floor scaled to " << dynamic_floor;
            }
        }
    }

    if (direction_flag_ == 1) {
      direction_flag_ = -1;
      double reduction_factor = 1.0 - alpha_ / 2.0;

      // --- PILLAR 2B: The Panic Drain ---
      // Apply the dynamic floor to the primary cut
      reduction_factor = std::min(reduction_factor, 0.95);
      reduction_factor = std::max(reduction_factor, dynamic_floor);

      DataRate reduced = std::max(current_rate * reduction_factor, min_target_rate_);
      reduced = std::max(reduced, DataRate::KilobitsPerSec(20));
      congestion_based_estimate_ = reduced;
      last_md_time_ = current_time;
      last_ai_update_time_ = current_time; // CRITICAL FIX: Reset AI clock on cut

      RTC_LOG(LS_VERBOSE) << "Prague: Switched to reduction mode (alpha=" << alpha_
                       << ", reduction_factor=" << reduction_factor
                       << "), new rate=" << congestion_based_estimate_.bps() << " bps";
    } else {
      TimeDelta gate_rtt = current_rtt_.IsFinite() && !current_rtt_.IsZero() ? current_rtt_ : TimeDelta::Millis(100);
      bool gate_open = last_md_time_.IsInfinite() || (current_time - last_md_time_ >= gate_rtt);
      if (gate_open) {
        double additional_reduction = 1.0 - alpha_ / 4.0;
        
        // Panic drain for continuous, unbroken congestion
        // Apply the exact same dynamic floor to continuous cuts
        additional_reduction = std::min(additional_reduction, 0.95);
        additional_reduction = std::max(additional_reduction, dynamic_floor);

        DataRate further_reduced = std::max(current_rate * additional_reduction, min_target_rate_);
        further_reduced = std::max(further_reduced, DataRate::KilobitsPerSec(20));
        congestion_based_estimate_ = further_reduced;

        last_md_time_ = current_time;
        last_ai_update_time_ = current_time; // Reset AI clock on cut
      }
    }
    last_congestion_signal_ = current_time;
  } else {  // No CE marks in this batch
    non_ce_packet_count_++;
    if (direction_flag_ == -1 && non_ce_packet_count_ >= kNonCeThreshold) {
      direction_flag_ = 1;
      non_ce_packet_count_ = 0;
      RTC_LOG(LS_VERBOSE) << "Prague: Switched to additive mode after " << kNonCeThreshold 
                           << " consecutive non-CE packets";
    }
  }
}


void webrtc::PragueCapacityEstimator::UpdateEcnActivity(Timestamp current_time) {
  // Track any ECN activity (ECT or CE packets) to maintain confidence
  last_ecn_feedback_ = current_time;
}

void webrtc::PragueCapacityEstimator::UpdateFromRtt(TimeDelta rtt) {
  if (rtt.IsFinite() && !rtt.IsZero()) {
    current_rtt_ = rtt;
    
    // --- STEP 1: Track the physical baseline ---
    // If this is the lowest RTT we've seen, or if we haven't set one yet, record it.
    if (baseline_rtt_.IsInfinite() || rtt < baseline_rtt_) {
        baseline_rtt_ = rtt;
    }
  }
}

void webrtc::PragueCapacityEstimator::OnPacketLoss(DataRate current_rate, Timestamp current_time) {
  // Multiplicative decrease for packet loss (fallback mechanism)
  DataRate reduced = std::max(current_rate * 0.5, min_target_rate_);
  // Enforce absolute minimum of 20 kbps to prevent pacer crashes
  reduced = std::max(reduced, DataRate::KilobitsPerSec(20));
  congestion_based_estimate_ = reduced;

  last_ai_update_time_ = current_time; // CRITICAL FIX: Reset AI clock on cut

  
  // Switch to reduction mode and reset non-CE counter
  direction_flag_ = -1;
  non_ce_packet_count_ = 0;
  
  RTC_LOG(LS_INFO) << "Prague: Packet loss detected, halving estimate to "
                      << congestion_based_estimate_.bps() << " bps, switched to reduction mode";
  
  last_update_time_ = current_time;
}

// 

//time-driven and ALR aware AI

void webrtc::PragueCapacityEstimator::OnTimeUpdate(Timestamp current_time, bool is_app_limited) {
  if (last_update_time_.IsInfinite() || last_ai_update_time_.IsInfinite()) {
    last_update_time_ = current_time;
    last_ai_update_time_ = current_time;
    return;
  }
  
  // TimeDelta elapsed = current_time - last_update_time_;
  TimeDelta ai_elapsed = current_time - last_ai_update_time_;

  if (ai_elapsed < TimeDelta::Millis(1)) {
    return; 
  }

  bool network_is_alive = !last_feedback_time_.IsInfinite() && 
                          (current_time - last_feedback_time_) < TimeDelta::Seconds(10);

  // 1. Growth Logic (ALR Aware)
  if (direction_flag_ == 1 && network_is_alive) {
    if (!is_app_limited && (additive_hold_until_.IsInfinite() || current_time >= additive_hold_until_)) {
      
      double rtt_s = current_rtt_.IsFinite() && !current_rtt_.IsZero() ? current_rtt_.seconds<double>() : 0.05;
      double elapsed_s = ai_elapsed.seconds<double>();
      
      constexpr double mss_bits = 1440.0 * 8.0;
      double theoretical_increase = (mss_bits / rtt_s) * (elapsed_s / rtt_s);
      
      int64_t ai_step_bps = static_cast<int64_t>(theoretical_increase);

      if (discovery_mode_active_ && !first_ce_mark_detected_ && congestion_based_estimate_.bps() < 5000000) {
        ai_step_bps *= 5;
        ai_step_bps = std::min(ai_step_bps, static_cast<int64_t>(2000000 * elapsed_s)); 
      } else {
        ai_step_bps = CalculateContextAwareAiStep(ai_step_bps, congestion_based_estimate_, current_time, elapsed_s);
      }

      if (ai_step_bps > 0) {
        DataRate increased = congestion_based_estimate_ + DataRate::BitsPerSec(ai_step_bps);
        if (max_target_rate_ > DataRate::Zero()) {
            increased = std::min(increased, max_target_rate_);
        }
        congestion_based_estimate_ = increased;
      }
    }
  }

  last_ai_update_time_ = current_time;

  // --- CRITICAL FIX 2A: Stop the ALR Decay Spiral ---
  // If the application is underutilizing the link, the network is NOT congested,
  // it is just idle.
  // We must freeze the 5% decay so the encoder has a stable platform.

  /* ---> COMMENT THIS ENTIRE BLOCK OUT <---
  if (elapsed >= kDecayInterval) {
    if (!is_app_limited) {
      congestion_based_estimate_ = std::max(congestion_based_estimate_ * 0.95, min_target_rate_);
      congestion_based_estimate_ = std::max(congestion_based_estimate_, DataRate::KilobitsPerSec(20));
    }
    // Always reset the timer so we don't build up "decay debt" while frozen
    last_update_time_ = current_time;
  }
  */
  
  // 3. Mode Escapes & Alpha Decay
  if (first_ce_mark_detected_ && !discovery_mode_active_) {
    TimeDelta since_congestion = current_time - last_congestion_signal_;
    if (since_congestion > TimeDelta::Seconds(30)) {
      discovery_mode_active_ = true;
      first_ce_mark_detected_ = false; 
    }
  }

  if (alpha_ > 0.0) {
    TimeDelta since_last_ce = current_time - last_congestion_signal_;
    TimeDelta safe_clearance = current_rtt_.IsFinite() ? (current_rtt_ * 2) : TimeDelta::Millis(100);
    
    if (!last_congestion_signal_.IsInfinite() && since_last_ce > safe_clearance) {
      alpha_ *= 0.95; 
      if (alpha_ < 0.001) {
        alpha_ = 0.0;
      }
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

// double webrtc::PragueCapacityEstimator::GetConfidence(Timestamp now) const {
//   // Check for any ECN activity (ECT or CE packets)
//   if (last_ecn_feedback_.IsInfinite()) {
//     return 0.3;  // Low confidence without any ECN feedback
//   }
  
//   TimeDelta since_ecn_activity = now - last_ecn_feedback_;
//   if (since_ecn_activity < TimeDelta::Seconds(2)) {
//     return 0.9;  // Very confident with recent ECN activity
//   } else if (since_ecn_activity < TimeDelta::Seconds(5)) {
//     return 0.7;  // Moderately confident
//   }
//   return 0.4;  // Lower confidence with stale ECN feedback
// }

double webrtc::PragueCapacityEstimator::GetConfidence(Timestamp now) const {
  if (last_ecn_feedback_.IsInfinite()) {
    return 0.3;  // Low confidence without any ECN feedback
  }
  
  TimeDelta since_ecn_activity = now - last_ecn_feedback_;
  
  // In L4S, silence is golden. As long as the connection hasn't totally 
  // timed out, Prague remains the absolute authority.
  if (since_ecn_activity < TimeDelta::Seconds(60)) {
    return 0.95; // Highly confident. Priority 1.
  }
  return 0.4;  // Path is likely dead
}



// int64_t webrtc::PragueCapacityEstimator::CalculateContextAwareAiStep(int64_t theoretical_ai_bps, DataRate current_rate, Timestamp current_time) {
//   // Context-aware AI step calculation that adapts to network conditions
  
//   // Safety checks for input parameters
//   if (theoretical_ai_bps <= 0 || !current_rate.IsFinite() || current_time.IsInfinite()) {
//     RTC_LOG(LS_WARNING) << "Prague: Invalid input parameters to CalculateContextAwareAiStep";
//     return 100000;  // Fallback: 100 Kbps step
//   }
  
//   // 1. Base multiplier starts at 1.0 (full DCTCP behavior)
//   double context_multiplier = 1.0;
  
//   // 2. Consider time since last congestion signal
//   TimeDelta since_congestion = current_time - last_congestion_signal_;
//   if (!last_congestion_signal_.IsInfinite()) {
//     if (since_congestion < TimeDelta::Seconds(1)) {
//       // Very recent congestion - be more conservative
//       context_multiplier *= 0.3;
//     } else if (since_congestion < TimeDelta::Seconds(5)) {
//       // Recent congestion - moderate caution
//       context_multiplier *= 0.6;
//     } else if (since_congestion > TimeDelta::Seconds(10)) {
//       // No recent congestion - can be more aggressive
//       context_multiplier *= 1.5;
//     }
//     // Between 5-10 seconds: use base multiplier (1.0)
//   }
  
//   // 3. Consider current alpha value (congestion severity)
//   if (alpha_ > 0.5) {
//     // High congestion memory - be very conservative
//     context_multiplier *= 0.2;
//   } else if (alpha_ > 0.1) {
//     // Moderate congestion memory - be somewhat conservative
//     context_multiplier *= 0.5;
//   } else if (alpha_ < 0.01) {
//     // Very low congestion memory - can be more aggressive
//     context_multiplier *= 1.3;
//   }
  
//   // 4. Consider rate magnitude (avoid explosive growth at high rates)
//   int64_t current_bps = current_rate.bps();
//   if (current_bps > 50000000) {  // > 50 Mbps
//     // At very high rates, cap growth to prevent network overload
//     context_multiplier *= 0.1;
//   } else if (current_bps > 10000000) {  // > 10 Mbps
//     // At high rates, be very conservative
//     context_multiplier *= 0.2;
//   } else if (current_bps > 5000000) {  // > 5 Mbps
//     // At medium-high rates, be conservative
//     context_multiplier *= 0.3;
//   } else if (current_bps > 1000000) {  // > 1 Mbps
//     // At medium rates, moderate increases
//     context_multiplier *= 0.6;
//   } else if (current_bps < 300000) {  // < 300 Kbps
//     // At very low rates, allow more aggressive growth
//     context_multiplier *= 1.5;
//   }
  
//   // 5. Consider direction flag stability
//   if (direction_flag_ == 1 && non_ce_packet_count_ > kNonCeThreshold * 2) {
//     // Been in additive mode for a while - can be more aggressive
//     context_multiplier *= 1.2;
//   }
  
//   // 6. Apply RTT-based scaling (better responsiveness for high RTT)
//   double rtt_seconds = current_rtt_.IsFinite() ? current_rtt_.seconds<double>() : 0.05;
//   if (rtt_seconds < 0.01) {  // < 10ms RTT - very low latency scenario
//     // For very low RTT, dramatically reduce AI steps to prevent explosive growth
//     context_multiplier *= 0.1;
//     RTC_LOG(LS_VERBOSE) << "Prague: Very low RTT (" << (rtt_seconds * 1000) << " ms), applying aggressive dampening";
//   } else if (rtt_seconds < 0.05) {  // < 50ms RTT - low latency
//     // For low RTT, moderate reduction to prevent excessive growth
//     context_multiplier *= 0.3;
//   } else if (rtt_seconds > 0.1) {  // > 100ms RTT
//     // High RTT networks need more aggressive AI to maintain fairness
//     context_multiplier *= std::min(2.0, rtt_seconds / 0.05);  // Scale with RTT, cap at 2x
//   }
  
//   // 7. Calculate context-aware step
//   int64_t context_ai_bps = static_cast<int64_t>(theoretical_ai_bps * context_multiplier);
  
//   // 8. Apply reasonable bounds to prevent pathological behavior
//   int64_t min_step_bps = theoretical_ai_bps / 20;  // At least 5% of DCTCP standard
  
//   // Much more aggressive rate-based capping for low RTT scenarios
//   int64_t rate_based_max_step = std::max(
//       static_cast<int64_t>(current_bps * 0.1),   // 10% of current rate
//       static_cast<int64_t>(100000)               // Minimum 100 Kbps step
//   );
  
//   int64_t max_step_bps = std::min(
//       theoretical_ai_bps * 2,     // At most 2x DCTCP standard
//       rate_based_max_step         // But respect rate-based limit
//   );
  
//   context_ai_bps = std::max(min_step_bps, std::min(context_ai_bps, max_step_bps));
  
//   // Final safety check to ensure result is valid
//   if (context_ai_bps <= 0 || context_ai_bps > 1000000000 || !std::isfinite(context_ai_bps)) {
//     RTC_LOG(LS_WARNING) << "Prague: Invalid context AI step calculated: " << context_ai_bps;
//     context_ai_bps = 100000;  // Fallback: 100 Kbps
//   }
  
//   // Log the decision for debugging
//   RTC_LOG(LS_VERBOSE) << "Prague: Context-aware AI calculation - "
//                       << "theoretical=" << theoretical_ai_bps << " bps, "
//                       << "multiplier=" << context_multiplier << ", "
//                       << "context_step=" << context_ai_bps << " bps, "
//                       << "alpha=" << alpha_ << ", "
//                       << "since_congestion=" << (last_congestion_signal_.IsInfinite() ? -1.0 : since_congestion.ms<double>()) << " ms, "
//                       << "current_rate=" << current_bps << " bps";
  
//   return context_ai_bps;
// }


//time-driven, context-aware AI step calculation that adapts to network conditions and recent history

int64_t webrtc::PragueCapacityEstimator::CalculateContextAwareAiStep(
    int64_t time_scaled_ai_step, 
    DataRate current_rate, 
    Timestamp current_time, 
    double elapsed_s) {
  
  // Safety checks - fallback scales with time (e.g., 1 Mbps per second of elapsed time)
  int64_t time_scaled_fallback = static_cast<int64_t>(1000000 * elapsed_s);
  
  if (time_scaled_ai_step <= 0 || !current_rate.IsFinite() || current_time.IsInfinite() || elapsed_s <= 0.0) {
    RTC_LOG(LS_WARNING) << "Prague: Invalid input parameters to CalculateContextAwareAiStep";
    return time_scaled_fallback; 
  }
  
  double context_multiplier = 1.0;
  
  // 1. Time since last congestion signal
  TimeDelta since_congestion = current_time - last_congestion_signal_;
  if (!last_congestion_signal_.IsInfinite()) {
    if (since_congestion < TimeDelta::Seconds(1)) {
      context_multiplier *= 0.3;
    } else if (since_congestion < TimeDelta::Seconds(5)) {
      context_multiplier *= 0.6;
    } else if (since_congestion > TimeDelta::Seconds(10)) {
      context_multiplier *= 1.5;
    }
  }
  
  // 2. Alpha value (congestion severity)
  if (alpha_ > 0.5) {
    context_multiplier *= 0.2;
  } else if (alpha_ > 0.1) {
    context_multiplier *= 0.5;
  } else if (alpha_ < 0.01) {
    context_multiplier *= 1.3;
  }
  
  // 3. Rate magnitude
  int64_t current_bps = current_rate.bps();
  if (current_bps > 50000000) {        // > 50 Mbps
    context_multiplier *= 0.1;
  } else if (current_bps > 10000000) { // > 10 Mbps
    context_multiplier *= 0.2;
  } else if (current_bps > 5000000) {  // > 5 Mbps
    context_multiplier *= 0.3;
  } else if (current_bps > 1000000) {  // > 1 Mbps
    context_multiplier *= 0.6;
  } else if (current_bps < 300000) {   // < 300 Kbps
    context_multiplier *= 1.5;
  }
  
  // 4. Direction flag stability
  if (direction_flag_ == 1 && non_ce_packet_count_ > kNonCeThreshold * 2) {
    context_multiplier *= 1.2;
  }
  
  // 5. RTT-based scaling
  double rtt_seconds = current_rtt_.IsFinite() ? current_rtt_.seconds<double>() : 0.05;
  if (rtt_seconds < 0.01) {
    context_multiplier *= 0.1;
  } else if (rtt_seconds < 0.05) {
    context_multiplier *= 0.3;
  } else if (rtt_seconds > 0.1) {
    context_multiplier *= std::min(2.0, rtt_seconds / 0.05);
  }
  
  // 6. Apply multiplier to the ALREADY time-scaled step
  int64_t context_ai_bps = static_cast<int64_t>(time_scaled_ai_step * context_multiplier);
  
  // 7. Time-scaled bounding
  int64_t min_step_bps = time_scaled_ai_step / 20; 
  
  // The rate-based cap limits the step to 10% of current rate PER SECOND of elapsed time
  int64_t rate_based_max_step = std::max(
      static_cast<int64_t>(current_bps * 0.1 * elapsed_s), 
      time_scaled_fallback 
  );
  
  int64_t max_step_bps = std::min(
      time_scaled_ai_step * 2,  
      rate_based_max_step       
  );
  
  context_ai_bps = std::max(min_step_bps, std::min(context_ai_bps, max_step_bps));
  
  if (context_ai_bps <= 0 || context_ai_bps > 1000000000 || !std::isfinite(context_ai_bps)) {
    RTC_LOG(LS_WARNING) << "Prague: Invalid context AI step calculated: " << context_ai_bps;
    context_ai_bps = time_scaled_fallback; 
  }
  
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

// 


// =============================================================================
// L4SBandwidthFusion Implementation
// =============================================================================

webrtc::L4SBandwidthFusion::L4SBandwidthFusion(const L4SControllerConfig& config) : config_(config) {}

webrtc::L4SBandwidthFusion::~L4SBandwidthFusion() = default;

void webrtc::L4SBandwidthFusion::UpdateEcnEstimate(DataRate estimate, double confidence, Timestamp now) {
  // Enforce absolute minimum of 20 kbps to prevent pacer crashes
  DataRate clamped_estimate = std::max(estimate, DataRate::KilobitsPerSec(20));
  RTC_LOG(LS_VERBOSE) << "L4S: Updating ECN estimate to " << clamped_estimate.bps() 
                      << " bps with confidence " << confidence;
  sources_.ecn_estimate = clamped_estimate;
  sources_.ecn_confidence = confidence;
  sources_.last_ecn_update = now;
}

void webrtc::L4SBandwidthFusion::UpdateProbeEstimate(DataRate estimate, double confidence, Timestamp now) {
  RTC_LOG(LS_VERBOSE) << "L4S: Updating probe estimate to " << estimate.bps() 
                      << " bps with confidence " << confidence;
  sources_.probe_estimate = estimate;
  sources_.probe_confidence = confidence;
  sources_.last_probe_update = now;
}

void webrtc::L4SBandwidthFusion::UpdateAckedEstimate(DataRate estimate, double confidence, Timestamp now) {
  RTC_LOG(LS_VERBOSE) << "L4S: Updating acked estimate to " << estimate.bps() 
                      << " bps with confidence " << confidence;
  sources_.acked_estimate = estimate;
  sources_.acked_confidence = confidence;
  sources_.last_acked_update = now;
}

webrtc::DataRate webrtc::L4SBandwidthFusion::GetFusedEstimateWithMode(
    Timestamp now, bool discovery_mode, bool recovery_mode) const {
  
  // 1. STATE MACHINE BRANCH: Fast capacity seeking
  if (discovery_mode || recovery_mode) {
    return GetDiscoveryModeFusedEstimate(now, recovery_mode);
  }
  
  // 2. STEADY STATE: Prague Time-Based AI is the Dictator
  DataRate prague_rate = sources_.ecn_estimate;
  DataRate probe_rate = sources_.probe_estimate;
  DataRate acked_rate = sources_.acked_estimate;
  
  bool prague_confident = sources_.ecn_confidence > config_.ecn_confidence_threshold && 
                          IsRecentlyUpdated(sources_.last_ecn_update, now);
  bool probe_confident = sources_.probe_confidence > config_.probe_confidence_threshold && 
                         IsRecentlyUpdated(sources_.last_probe_update, now);
  bool acked_confident = sources_.acked_confidence > config_.acked_confidence_threshold && 
                         IsRecentlyUpdated(sources_.last_acked_update, now);

  DataRate fused_rate = DataRate::KilobitsPerSec(300); // Safe fallback

  if (prague_confident) {
    fused_rate = prague_rate;
    

    //turning off the uplift for testing
    // // Micro-Probe Uplift: If a probe cleanly found higher capacity, allow it to 
    // // pull the time-based AI forward, but cap the jump (e.g., 1.5x) to prevent runaway AQM overshoot.
    // if (probe_confident && probe_rate > prague_rate) {
    //   DataRate max_uplift = prague_rate * 1.5;
    //   fused_rate = std::min(probe_rate, max_uplift);
    //   RTC_LOG(LS_VERBOSE) << "L4S Fusion: Steady State Probe Uplift applied -> " << fused_rate.bps() << " bps";
    // }
  } 
  // 3. Fallbacks if ECN feedback is completely dead
  else if (probe_confident) {
    fused_rate = probe_rate;
    RTC_LOG(LS_VERBOSE) << "L4S Fusion: Prague dead, falling back to Probe -> " << fused_rate.bps() << " bps";
  } 
  else if (acked_confident) {
    // Acked rate is a terrible ceiling. If we MUST use it, gently lift it by 10% 
    // to give the video encoder room to grow.
    fused_rate = acked_rate * 1.1; 
    RTC_LOG(LS_VERBOSE) << "L4S Fusion: Falling back to Acked rate -> " << fused_rate.bps() << " bps";
  } else {
    fused_rate = GetMostConfidentEstimate(now);
  }

  // 4. THE REALITY CHECK FLOOR (Crucial for Application Limited Regions)
  // Never let the fused estimate fall below what the network is actively delivering right now.
  if (sources_.acked_confidence > 0.3 && IsRecentlyUpdated(sources_.last_acked_update, now)) {
    if (fused_rate < acked_rate) {
      RTC_LOG(LS_VERBOSE) << "L4S Fusion: Acked Floor engaged. Fused " << fused_rate.bps() 
                          << " bps raised to actual " << acked_rate.bps() << " bps";
      fused_rate = acked_rate;
    }
  }

  return std::max(fused_rate, DataRate::KilobitsPerSec(20));
}



// webrtc::DataRate webrtc::L4SBandwidthFusion::GetDiscoveryModeFusedEstimate(
//     Timestamp now, bool recovery_mode) const {
  
//   // In Discovery or Recovery mode, the goal is to find the ceiling FAST.
//   // Therefore, Probes take priority over the slow Time-Based AI, UNLESS Prague 
//   // has seen a CE mark and tells us to stop.
  
//   DataRate probe_rate = sources_.probe_estimate;
//   DataRate prague_rate = sources_.ecn_estimate;
  
//   bool probe_confident = sources_.probe_confidence > config_.probe_confidence_threshold && 
//                          IsRecentlyUpdated(sources_.last_probe_update, now);
//   bool prague_confident = sources_.ecn_confidence > config_.ecn_confidence_threshold && 
//                           IsRecentlyUpdated(sources_.last_ecn_update, now);

//   DataRate fused_rate = DataRate::KilobitsPerSec(300);

//   if (probe_confident) {
//     fused_rate = probe_rate;
    
//     // The Prague Veto: If Prague is confident but its estimate is LOWER than the probe,
//     // it means Prague applied a Multiplicative Decrease (saw CE marks). 
//     // Congestion avoidance ALWAYS overrides capacity discovery.
//     if (prague_confident && prague_rate < fused_rate) {
//       fused_rate = prague_rate;
//       RTC_LOG(LS_VERBOSE) << "L4S Fusion: Discovery Mode - Prague vetoed probe, using ECN rate: " 
//                           << fused_rate.bps() << " bps";
//     } else {
//       RTC_LOG(LS_VERBOSE) << "L4S Fusion: Discovery Mode - Trusting probe: " 
//                           << fused_rate.bps() << " bps";
//     }
//   } else if (prague_confident) {
//     fused_rate = prague_rate;
//     RTC_LOG(LS_VERBOSE) << "L4S Fusion: Discovery Mode - No valid probe, using Prague AI: " 
//                         << fused_rate.bps() << " bps";
//   } else {
//     fused_rate = GetMostConfidentEstimate(now);
//   }

//   // Apply Reality Check Floor
//   if (sources_.acked_confidence > 0.3 && IsRecentlyUpdated(sources_.last_acked_update, now)) {
//     if (fused_rate < sources_.acked_estimate) {
//       fused_rate = sources_.acked_estimate;
//     }
//   }

//   return std::max(fused_rate, DataRate::KilobitsPerSec(20));
// }





webrtc::DataRate webrtc::L4SBandwidthFusion::GetDiscoveryModeFusedEstimate(
    Timestamp now, bool recovery_mode) const {
  
  DataRate probe_rate = sources_.probe_estimate;
  DataRate prague_rate = sources_.ecn_estimate;
  
  bool probe_confident = sources_.probe_confidence > config_.probe_confidence_threshold && 
                         IsRecentlyUpdated(sources_.last_probe_update, now);
  bool prague_confident = sources_.ecn_confidence > config_.ecn_confidence_threshold && 
                          IsRecentlyUpdated(sources_.last_ecn_update, now);

  DataRate fused_rate = DataRate::KilobitsPerSec(300);

  if (probe_confident) {
    fused_rate = probe_rate;
    
    if (prague_confident) {
      if (prague_rate < fused_rate) {
        // The Prague Veto: Prague saw CE marks and dictates a lower rate.
        fused_rate = prague_rate;
        RTC_LOG(LS_VERBOSE) << "L4S Fusion: Discovery Mode - Prague vetoed probe, using ECN rate: " 
                            << fused_rate.bps() << " bps";
      } else {
        // --- CRITICAL FIX 2B: The Smeared Probe Guard ---
        // Prague is HIGHER than the probe. The probe got smeared by the AQM queue.
        // We MUST hold the Prague rate so we don't crash the BWE and rubber-band!
        fused_rate = prague_rate;
        RTC_LOG(LS_VERBOSE) << "L4S Fusion: Discovery Mode - Ignored smeared probe (" 
                            << probe_rate.bps() << " bps), holding Prague rate: " 
                            << prague_rate.bps() << " bps";
      }
    } else {
      RTC_LOG(LS_VERBOSE) << "L4S Fusion: Discovery Mode - Trusting probe: " 
                          << fused_rate.bps() << " bps";
    }
  } else if (prague_confident) {
    fused_rate = prague_rate;
    RTC_LOG(LS_VERBOSE) << "L4S Fusion: Discovery Mode - No valid probe, using Prague AI: " 
                        << fused_rate.bps() << " bps";
  } else {
    fused_rate = GetMostConfidentEstimate(now);
  }

  // Apply Reality Check Floor
  if (sources_.acked_confidence > 0.3 && IsRecentlyUpdated(sources_.last_acked_update, now)) {
    if (fused_rate < sources_.acked_estimate) {
      fused_rate = sources_.acked_estimate;
    }
  }

  return std::max(fused_rate, DataRate::KilobitsPerSec(20));
}



webrtc::DataRate webrtc::L4SBandwidthFusion::GetMostConfidentEstimate(Timestamp now) const {
  DataRate best_estimate = DataRate::KilobitsPerSec(300);  // Fallback
  double best_confidence = 0.0;
  
  if (sources_.ecn_confidence > best_confidence && IsRecentlyUpdated(sources_.last_ecn_update, now)) {
    best_estimate = sources_.ecn_estimate;
    best_confidence = sources_.ecn_confidence;
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

webrtc::DataRate webrtc::L4SBandwidthFusion::ValidateWithOtherSources(
    DataRate primary_estimate, const BandwidthSources& sources) const {
  // Utility method to prevent probe artifacts from causing runaway targets
  DataRate max_alternative = DataRate::Zero();
  
  if (sources.ecn_confidence > 0.3) {
    max_alternative = std::max(max_alternative, sources.ecn_estimate);
  }
  if (sources.acked_confidence > 0.3) {
    max_alternative = std::max(max_alternative, sources.acked_estimate);
  }
  
  // If a probe result is more than 2x our other known valid signals, it's likely 
  // a DualPI2 scheduling artifact (burst spread over a long window). Cap it.
  if (max_alternative > DataRate::Zero() && primary_estimate > max_alternative * 2.0) {
    return max_alternative * 1.5;
  }
  
  return primary_estimate;
}

bool webrtc::L4SBandwidthFusion::IsRecentlyUpdated(Timestamp last_update, Timestamp now) const {
  if (last_update.IsInfinite() || now.IsInfinite()) {
    return false;
  }
  // In a sparse-RTCP setup, data is considered valid for up to 60 seconds 
  // (to survive periodic reporting gaps) before being flagged as stale.
  return (now - last_update) < TimeDelta::Seconds(60);
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

//old code, new code is below


// void webrtc::L4SMetricsCollector::LogBandwidthMetrics(
//     Timestamp at_time,
//     DataRate target_bitrate,
//     DataRate actual_bitrate,
//     std::optional<DataRate> acked_bitrate) {
//   if (at_time - last_bandwidth_log_ < kBandwidthLogInterval) {
//     return;
//   }
  
//   last_bandwidth_log_ = at_time;
//   // Shared GCC/L4S contract: throughput for comparison is acked rate.
//   DataRate rate_to_log = acked_bitrate.value_or(DataRate::Zero());
//   UpdateThroughputStats(rate_to_log);

//   logger_->LogSingleValueMetric("acked_rate_mbps", test_case_name_,
//                                 rate_to_log.bps() / 1e6,
//                                 webrtc::test::Unit::kUnitless,
//                                 webrtc::test::ImprovementDirection::kBiggerIsBetter,
//                                 {{"timestamp_ms", std::to_string(at_time.ms())}});

//   (void)target_bitrate;
//   (void)actual_bitrate;
// }


void webrtc::L4SMetricsCollector::LogBandwidthMetrics(
    Timestamp at_time,
    DataRate target_bitrate,
    DataRate actual_bitrate,
    std::optional<DataRate> acked_bitrate,
    std::optional<DataRate> send_rate) { // <-- New parameter
    
  if (at_time - last_bandwidth_log_ < kBandwidthLogInterval) {
    return;
  }
  last_bandwidth_log_ = at_time;

  // 1. Target Rate (The Software Budget)
  if (target_bitrate.IsFinite()) {
      logger_->LogSingleValueMetric("target_rate_mbps", test_case_name_,
                                    target_bitrate.bps() / 1e6,
                                    webrtc::test::Unit::kUnitless,
                                    webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                    {{"timestamp_ms", std::to_string(at_time.ms())}});
  }

  // 2. Send Rate (The Pacer's Physical Exhaust)
  DataRate tx_rate = send_rate.value_or(DataRate::Zero());
  logger_->LogSingleValueMetric("send_rate_mbps", test_case_name_,
                                tx_rate.bps() / 1e6,
                                webrtc::test::Unit::kUnitless,
                                webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});

  // 3. Actual Rate (L4S Raw Physical Throughput Window)
  logger_->LogSingleValueMetric("actual_rate_mbps", test_case_name_,
                                actual_bitrate.bps() / 1e6,
                                webrtc::test::Unit::kUnitless,
                                webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});

  // 4. Acked Rate (GCC-Parity Kalman Filtered Throughput)
  DataRate filtered_acked_rate = acked_bitrate.value_or(DataRate::Zero());
  UpdateThroughputStats(filtered_acked_rate); // Keep summary stats based on GCC's metric
  logger_->LogSingleValueMetric("acked_rate_mbps", test_case_name_,
                                filtered_acked_rate.bps() / 1e6,
                                webrtc::test::Unit::kUnitless,
                                webrtc::test::ImprovementDirection::kBiggerIsBetter,
                                {{"timestamp_ms", std::to_string(at_time.ms())}});
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

  // Route change must clear path-specific history. Keeping old recovery/fusion
  // state can cause immediate post-route transitions that reflect the previous
  // path rather than the new one.
  DataRate reset_starting_rate =
      starting_rate_.value_or(DataRate::KilobitsPerSec(300));
  DataRate reset_min_rate =
      min_target_rate_.value_or(DataRate::KilobitsPerSec(30));
  DataRate reset_max_rate =
      max_target_rate_.value_or(DataRate::KilobitsPerSec(100000));
  prague_estimator_ = std::make_unique<PragueCapacityEstimator>(
      reset_starting_rate, reset_min_rate, reset_max_rate);
  bandwidth_fusion_ = std::make_unique<L4SBandwidthFusion>(config_);

  recovery_mode_active_ = false;
  recovery_probe_bootstrapped_ = false;
  consecutive_clean_packets_ = 0;
  clean_ect_run_start_ = Timestamp::MinusInfinity();
  recovery_start_time_ = Timestamp::MinusInfinity();
  recovery_cooldown_until_ = Timestamp::MinusInfinity();

  last_probe_time_ = Timestamp::MinusInfinity();
  probe_hold_until_ = Timestamp::MinusInfinity();
  initial_probes_sent_ = false;
  last_reported_bitrate_to_probe_controller_ = DataRate::Zero();
  previously_in_alr_ = false;

  throughput_window_.clear();
  last_actual_bitrate_ = DataRate::Zero();
  last_acked_bitrate_.reset();
  last_loss_fraction_ = 0.0;
  last_packets_lost_ = 0;
  last_state_snapshot_log_ = Timestamp::MinusInfinity();

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

  // Update time-based growth/decay in Prague estimator
// Update time-based growth/decay in Prague estimator
  prague_estimator_->OnTimeUpdate(msg.at_time, IsApplicationLimited() || !EncoderNeedsMoreHeadroom(1.2));
  
  // --- PRAGUE DICTATOR MODE ---
  // Unconditionally push Prague's state to the Fusion Engine every 100ms.
  // Never hide Prague's estimate just because we are in ALR or Reduction.
  double ecn_confidence = prague_estimator_->GetConfidence(msg.at_time);
  bandwidth_fusion_->UpdateEcnEstimate(
      prague_estimator_->GetCurrentEstimate(), 
      ecn_confidence, 
      msg.at_time
  );

// State-owned fusion policy
  DataRate fused_rate = ApplyStateFusionPolicy(msg.at_time); // or msg.feedback_time
  target_rate_ = fused_rate;


  // --- CRITICAL FIX 1B: CLOSE THE FUSION GAP ---
  // The Driver (Prague) must always steer from the actual enforced limit. 
  // If Fusion bounded the rate (e.g., max limits, probe ceilings), 
  // Prague's internal state must be synchronized to match reality.
  if (prague_estimator_) {
      prague_estimator_->SetCurrentEstimate(fused_rate);
  }

  // Create rate update
  MaybeTriggerOnNetworkChanged(&update, msg.at_time);

  AdvanceStateMachine(msg.at_time);
  
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnRemoteBitrateReport(RemoteBitrateReport msg) {
  NetworkControlUpdate update;
  return update;
}


//smoothed rtt update handler with EMA and safety floor

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnRoundTripTimeUpdate(RoundTripTimeUpdate msg) {
  NetworkControlUpdate update;
  if (msg.smoothed) {
    return update;
  }

  if (msg.round_trip_time.IsFinite() && !msg.round_trip_time.IsZero()) {
    // --- CRITICAL FIX: Apply the same EMA smoothing here ---
    if (last_rtt_.IsFinite() && !last_rtt_.IsZero()) {
      last_rtt_ = (last_rtt_ * 0.8) + (msg.round_trip_time * 0.2);
    } else {
      last_rtt_ = msg.round_trip_time;
    }
    
    // Enforce the 20ms safety floor
    TimeDelta safe_rtt = std::max(last_rtt_, TimeDelta::Millis(20));
    last_estimated_round_trip_time_ = safe_rtt;
    prague_estimator_->UpdateFromRtt(safe_rtt);
  }

  RTC_LOG(LS_VERBOSE) << "L4S: RTCP RTT updated/smoothed to " << last_rtt_.ms() << " ms";
  return update;
}



// webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnSentPacket(SentPacket msg) {
//   NetworkControlUpdate update;
//   // Feed ALR detector so it can track application-limited periods.
//   if (alr_detector_) {
//     alr_detector_->OnBytesSent(msg.size.bytes(), msg.send_time.ms());
//     if (acked_estimator_) {
//       acked_estimator_->SetAlr(
//           alr_detector_->GetApplicationLimitedRegionStartTime().has_value());
//     }
//   }
//   return update;
// }


// new method with send rate tracking for better probe scheduling and fusion accuracy
webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnSentPacket(SentPacket msg) {
  NetworkControlUpdate update;
  
  // 1. Feed ALR detector (existing logic)
  if (alr_detector_) {
    alr_detector_->OnBytesSent(msg.size.bytes(), msg.send_time.ms());
    if (acked_estimator_) {
      acked_estimator_->SetAlr(
          alr_detector_->GetApplicationLimitedRegionStartTime().has_value());
    }
  }

  // --- 2. Calculate Pacer's Actual Send Rate ---
  constexpr TimeDelta kSendWindow = TimeDelta::Millis(500); // 500ms smoothing window
  
  if (msg.send_time.IsFinite()) {
    send_rate_window_.emplace_back(msg.send_time, msg.size.bytes());
  }

  // Evict packets older than the window
  while (!send_rate_window_.empty() && 
         (msg.send_time - send_rate_window_.front().first) > kSendWindow) {
    send_rate_window_.pop_front();
  }

  // Calculate the transmission rate if we have a valid time gap
  if (send_rate_window_.size() > 1) {
    Timestamp window_start = send_rate_window_.front().first;
    Timestamp window_end = send_rate_window_.back().first;
    TimeDelta window_interval = window_end - window_start;

    // Only update the rate if the window is statistically significant (e.g., > 100ms)
    if (window_interval >= TimeDelta::Millis(100)) {
      int64_t window_bytes = 0;
      for (const auto& entry : send_rate_window_) {
        window_bytes += entry.second;
      }
      last_send_rate_ = DataRate::BitsPerSec(
          static_cast<int64_t>((window_bytes * 8) / window_interval.seconds<double>()));
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

  // --- CRITICAL FIX: Capture padding constraints ---
  if (msg.max_padding_rate) {
      max_padding_rate_ = *msg.max_padding_rate;
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
  // int packets_with_feedback = 0;
  // int lost_packets_in_feedback = 0;
  // for (const auto& packet_feedback : msg.PacketsWithFeedback()) {
  //   ++packets_with_feedback;
  //   if (!packet_feedback.IsReceived()) {
  //     ++lost_packets_in_feedback;
  //   }
  // }
  // if (packets_with_feedback > 0) {
  //   last_packets_lost_ = lost_packets_in_feedback;
  // }
  
  // Update all bandwidth estimators
  UpdateAllBandwidthEstimators(msg);

  // if (throughput_estimator_) {
  //   last_actual_bitrate_ = throughput_estimator_->GetCurrentEstimate();
  // }
  
  // State-owned probing policy
  ApplyStateProbingPolicy(msg.feedback_time, &update);
  
// State-owned fusion policy
  DataRate fused_rate = ApplyStateFusionPolicy(msg.feedback_time); // or msg.feedback_time
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
      // --- CRITICAL FIX 1A: Smooth the RTT ---
      // Never feed raw, instantaneous RTT into Prague. 
      if (last_rtt_.IsFinite() && !last_rtt_.IsZero()) {
          // Exponential moving average: 80% old, 20% new
          last_rtt_ = (last_rtt_ * 0.8) + (feedback_min_rtt * 0.2);
      } else {
          last_rtt_ = feedback_min_rtt;
      }
      
      // Enforce a hard physical minimum of 20ms to prevent division-by-zero explosions
      TimeDelta safe_rtt = std::max(last_rtt_, TimeDelta::Millis(20));
      
      prague_estimator_->UpdateFromRtt(safe_rtt);
      last_estimated_round_trip_time_ = safe_rtt;
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

// void webrtc::L4SNetworkController::ApplyStateEcnPolicy(
//     const TransportPacketsFeedback& feedback,
//     DataRate base_fused_rate) {
//   if (!IsApplicationLimited()) {
//     ProcessEcnFeedback(feedback, base_fused_rate);
//   } else {
//     RTC_LOG(LS_VERBOSE) << "L4S: Skipping ECN processing during ALR period";
//   }
// }


void webrtc::L4SNetworkController::ApplyStateEcnPolicy(
    const TransportPacketsFeedback& feedback,
    DataRate base_fused_rate) {
  // --- CRITICAL FIX 2: Always respect the network ---
  // Congestion is congestion, regardless of whether the application is 
  // currently filling the pipe. We must always process ECN feedback.
  ProcessEcnFeedback(feedback, base_fused_rate);
}


// 


//time-driven AI 
void webrtc::L4SNetworkController::ProcessEcnFeedback(const TransportPacketsFeedback& feedback, DataRate current_fused_rate) {
  if (feedback.packet_feedbacks.empty()) {
    return;
  }

  // NOTE: window_ce_count_, window_ect_count_, and window_start_time_ 
  // MUST be moved to private members in l4s_network_controller.h
  
  TimeDelta window_duration = last_rtt_.IsFinite() && !last_rtt_.IsZero() ? last_rtt_ : TimeDelta::Millis(100);

  int batch_ect_count = 0;
  int batch_ce_count = 0;
  bool probe_caused_congestion = false;

  // 1. Extract Batch Info
  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1) {
      batch_ect_count++;
    }
    if (packet.ecn == EcnMarking::kCe) {
      batch_ce_count++;
      last_congestion_signal_ = feedback.feedback_time;
      if (packet.sent_packet.pacing_info.probe_cluster_id != PacedPacketInfo::kNotAProbe) {
        probe_caused_congestion = true;
      }
    }
  }

  // 2. Heartbeat/Activity Update
  if (batch_ect_count > 0 || batch_ce_count > 0) {
    ecn_supported_ = true;
    prague_estimator_->UpdateEcnActivity(feedback.feedback_time);
  }

  // 3. Accumulate Window (Do this before ANY returns!)
  window_ce_count_ += batch_ce_count;
  window_ect_count_ += batch_ect_count;

  // 4. THE WINDOW GATE (RTT-Aware processing)
  // We process the accumulated data if:
  // a) The RTT window has expired
  // b) OR we have a significant number of packets (e.g., > 15) even if RTT hasn't passed
  bool window_expired = (window_start_time_.IsInfinite() || (feedback.feedback_time - window_start_time_) >= window_duration);
  int window_total = window_ce_count_ + window_ect_count_;

  if (window_expired || window_total >= 20) {
    double ce_ratio = (window_total > 0) ? static_cast<double>(window_ce_count_) / window_total : 0.0;

    // Minimum packets to trust the ratio
    if (window_total >= 3) {
      if (window_ce_count_ > 0 && probe_caused_congestion) {
        RTC_LOG(LS_INFO) << "L4S: Probe-triggered CE. Freezing AI for 2 RTTs.";
        prague_estimator_->SetAdditiveHoldUntil(feedback.feedback_time + (last_rtt_ * 2));
        
        // CRITICAL FIX: The probe FAILED. Clear it! Do not save last_actual_bitrate_ here.
        bandwidth_fusion_->UpdateProbeEstimate(DataRate::Zero(), 0.0, feedback.feedback_time);
      }
      else {
        DataRate prague_input_rate = prague_estimator_->GetCurrentEstimate();
        
        // --- CRITICAL FIX: Only snap when we actually hit congestion! ---
        if (window_ce_count_ > 0) {
            // L4S AQMs throw occasional CE marks to smooth out Pacer micro-bursts.
            // We ONLY vaporize the headroom if congestion is severe (>10% marked).
            bool is_severe_congestion = (ce_ratio > 0.10);
            // If the Target is floating more than 30% above the Actual throughput, 
            // the Pacer was barely working. Snap the Target down to reality before cutting.
            if (is_severe_congestion && !last_actual_bitrate_.IsZero() && 
                prague_input_rate > last_actual_bitrate_ * 1.3){
                prague_input_rate = last_actual_bitrate_ * 1.1; // Snap to actual + 10% breathing room
                RTC_LOG(LS_INFO) << "L4S: Severe Congestion (" << (ce_ratio*100) 
                                 << "% CE). Snapping Target from " 
                                 << prague_estimator_->GetCurrentEstimate().kbps() 
                                 << "k down to " << prague_input_rate.kbps() << "k.";

                // Force Prague to adopt this snapped reality immediately
                prague_estimator_->SetCurrentEstimate(prague_input_rate);
            }
        }

        // Now apply the math (if ce_ratio > 0, it cuts. If ce_ratio == 0, it grows).
        prague_estimator_->UpdateFromCongestionSignal(prague_input_rate, ce_ratio, feedback.feedback_time);

        // Safety Floor to prevent total collapse
        if (!last_actual_bitrate_.IsZero()) {
          DataRate floor = last_actual_bitrate_ * 0.5;
          if (prague_estimator_->GetCurrentEstimate() < floor) {
            prague_estimator_->SetCurrentEstimate(floor);
          }
        }
      }

      // Sync with Fusion Engine
      double ecn_confidence = prague_estimator_->GetConfidence(feedback.feedback_time);
      if (prague_estimator_->GetDirectionFlag() == -1) {
        ecn_confidence = std::max(ecn_confidence, 0.95);
      }
      bandwidth_fusion_->UpdateEcnEstimate(prague_estimator_->GetCurrentEstimate(), ecn_confidence, feedback.feedback_time);
    }

    // RESET WINDOW
    window_ce_count_ = 0;
    window_ect_count_ = 0;
    window_start_time_ = feedback.feedback_time;
  }

  // 5. Recovery logic always sees the raw batch info
  HandleRecoveryDetection(batch_ect_count, batch_ce_count, feedback.feedback_time);
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
      DataRate gentle_target = acked_throughput * 1.1;  // 10% increase
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



// void webrtc::L4SNetworkController::ProcessRealProbeResults(const TransportPacketsFeedback& feedback) {
//   if (!probe_bitrate_estimator_) {
//     return;
//   }

//   // Process each packet to detect probe clusters and measure real throughput
//   for (const auto& packet_feedback : feedback.SortedByReceiveTime()) {
//     if (packet_feedback.sent_packet.pacing_info.probe_cluster_id != PacedPacketInfo::kNotAProbe) {
//       // This is a real probe packet - let ProbeBitrateEstimator measure it
//       probe_bitrate_estimator_->HandleProbeAndEstimateBitrate(packet_feedback);
      
//       RTC_LOG(LS_VERBOSE) << "L4S: Processing probe packet from cluster " 
//                          << packet_feedback.sent_packet.pacing_info.probe_cluster_id
//                          << ", size: " << packet_feedback.sent_packet.size.bytes() << " bytes";
//     }
//   }

//   // Get the real measured probe result (if any)
//   std::optional<DataRate> measured_probe_rate = GetLastProbeResult();
//   if (measured_probe_rate) {
//     // Layer 1 sanity guard: discard probe results that are physically impossible.
//     // If the link is already delivering more actual throughput than the probe measured,
//     // the probe result is an artifact (e.g. DualPI2 scheduling spreading the burst
//     // across a long receive window at uncongested rates) and must not be used to cap
//     // the rate. A real capacity measurement can never be below the already-observed
//     // delivery rate.
//     if (last_actual_bitrate_.IsZero() || *measured_probe_rate >= last_actual_bitrate_) {
//       double probe_confidence = CalculateProbeConfidence(feedback.feedback_time);
//       bandwidth_fusion_->UpdateProbeEstimate(*measured_probe_rate, probe_confidence, feedback.feedback_time);

//       RTC_LOG(LS_VERBOSE) << "L4S: Real probe result accepted: " << measured_probe_rate->bps()
//                           << " bps (actual throughput: " << last_actual_bitrate_.bps()
//                           << " bps) with confidence " << probe_confidence;
//     } else {
//       RTC_LOG(LS_INFO) << "L4S: Probe result discarded (physically impossible): "
//                        << measured_probe_rate->bps() << " bps < actual throughput "
//                        << last_actual_bitrate_.bps() << " bps - likely AQM scheduling artifact";
//     }
//   }
// }

//nudging logic to break ALR deadlock when a probe proves the path is clean
void webrtc::L4SNetworkController::ProcessRealProbeResults(const TransportPacketsFeedback& feedback) {
  if (!probe_bitrate_estimator_) {
    return;
  }

  bool probe_packet_has_ce = false;
  for (const auto& packet_feedback : feedback.SortedByReceiveTime()) {
    if (packet_feedback.sent_packet.pacing_info.probe_cluster_id != PacedPacketInfo::kNotAProbe) {
      probe_bitrate_estimator_->HandleProbeAndEstimateBitrate(packet_feedback);
      
      // Monitor if the physical probe actually hit a queue
      if (packet_feedback.ecn == EcnMarking::kCe) {
        probe_packet_has_ce = true;
      }
    }
  }

  std::optional<DataRate> measured_probe_rate = GetLastProbeResult();
  if (measured_probe_rate) {
    // Reality Check Floor: Only accept probes that aren't mathematically impossible
    if (last_actual_bitrate_.IsZero() || *measured_probe_rate >= last_actual_bitrate_) {
      double probe_confidence = CalculateProbeConfidence(feedback.feedback_time);
      bandwidth_fusion_->UpdateProbeEstimate(*measured_probe_rate, probe_confidence, feedback.feedback_time);

      // --- THE SAFE ESCALATION HANDSHAKE (Event-Driven) ---
      // If a probe physically proved the path is CLEAN, nudge Prague up.
      // This handles both ALR deadlock breaking AND fast congestion recovery.
      if (prague_estimator_ && !probe_packet_has_ce) {
          DataRate current_prague = prague_estimator_->GetCurrentEstimate();
          
          // Accept the probe if it proves even a 5% capacity uplift
          if (*measured_probe_rate > (current_prague * 1.05)) {
              
              // FAST RECOVERY: Pull Prague up to the probe rate, 
              // but cap the jump at 1.5x per probe to prevent AQM shock.
              DataRate max_uplift = current_prague * 1.5;
              DataRate new_target = std::min(*measured_probe_rate, max_uplift);

              RTC_LOG(LS_INFO) << "L4S: Fast Recovery/Escalation! Probe proved " 
                               << measured_probe_rate->kbps() 
                               << "k. Nudging Prague to " << new_target.kbps() << "k.";
                               
              prague_estimator_->SetCurrentEstimate(new_target);
              
              // Force the Fusion Engine and ALR Detector to sync immediately
              bandwidth_fusion_->UpdateEcnEstimate(new_target, 0.95, feedback.feedback_time);
              if (alr_detector_) {
                alr_detector_->SetEstimatedBitrate(new_target.bps());
              }
          }
      }
    } else {
      RTC_LOG(LS_INFO) << "L4S: Probe result discarded (physically impossible): "
                       << measured_probe_rate->bps() << " bps < actual throughput "
                       << last_actual_bitrate_.bps() << " bps";
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
    InitiateProbing(now, update);
    last_probe_time_ = now;
  }
}

void webrtc::L4SNetworkController::ApplyStateProbingPolicy(
    Timestamp now,
    NetworkControlUpdate* update) {
  HandlePeriodicProbing(now, update);
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


// 



std::optional<webrtc::ProbeClusterConfig> webrtc::L4SNetworkController::CreateCustomProbe(
    Timestamp now, DataRate target_rate) {
    
  if (target_rate <= DataRate::Zero()) {
      return std::nullopt;
  }

  // Use a static ID counter starting high to avoid any collisions with native WebRTC probes
  static int32_t next_custom_probe_id = 10000; 

  ProbeClusterConfig custom_probe;
  custom_probe.at_time = now;
  custom_probe.target_data_rate = target_rate;
  // 15ms is the sweet spot: long enough to get a good read, short enough not to bloat the DualPI2 queue
  custom_probe.target_duration = TimeDelta::Millis(15); 
  custom_probe.target_probe_count = 5;
  custom_probe.id = next_custom_probe_id++;

  return custom_probe;
}

//time-driven probing used for both periodic and recovery probes, with different parameters and constraints


// Helper: Returns true if encoder needs more headroom for probing
bool webrtc::L4SNetworkController::EncoderNeedsMoreHeadroom(double multiplier) const {
  if (!IsApplicationLimited()) {
    return true;
  }
  DataRate current_estimate = target_rate_.value_or(DataRate::Zero());
  DataRate actual_throughput = this->last_actual_bitrate_;
  if (actual_throughput <= DataRate::Zero()) {
    return true; // Don't block probes if we have no data yet
  }
  
  // --- HYBRID HEADROOM FIX ---
  // Calculate both relative (e.g., 20%) and absolute (e.g., +1.5 Mbps) headroom.
  // Use whichever provides MORE room, ensuring resolution jumps (720p -> 1080p) 
  // always have enough absolute runway without requiring massive multipliers.
  DataRate relative_cap = actual_throughput * multiplier;
  DataRate absolute_cap = actual_throughput + DataRate::KilobitsPerSec(1500); // 1.5 Mbps flat buffer
  
  DataRate allowed_target = std::max(relative_cap, absolute_cap);
  
  if (current_estimate > allowed_target) {
    return false; // Cap exceeded, freeze growth
  }
  
  return true;
}

void webrtc::L4SNetworkController::InitiateProbing(Timestamp now, NetworkControlUpdate* update) {
  // Headroom cap: block probe if already >2x actual throughput
  if (!EncoderNeedsMoreHeadroom(1.2)) {
    RTC_LOG(LS_VERBOSE) << "L4S: Skipping periodic probe due to headroom cap (2x actual throughput).";
    return;
  }
  
  RTC_LOG(LS_INFO) << "L4S: Initiating periodic probe!";
  DataRate current_estimate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  double micro_probe_multiplier = 1.05;
  if (IsApplicationLimited()) {
    // Widen the net to help the encoder out of the yo-yo trap
    micro_probe_multiplier = 1.25; 
  }
  DataRate probe_rate = current_estimate * micro_probe_multiplier;
  if (max_target_rate_) {
    probe_rate = std::min(probe_rate, *max_target_rate_);
  }
  // --- Bypass WebRTC ProbeController: Call our unified helper ---
  auto custom_probe = CreateCustomProbe(now, probe_rate);
  if (custom_probe) {
    update->probe_cluster_configs.push_back(*custom_probe);
    StartProbeHold(now);
    RTC_LOG(LS_INFO) << "L4S: Manually injected Periodic Micro-Probe at " 
                     << probe_rate.bps() << " bps with ID " << custom_probe->id;
  }
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
  double total_weight = temp_sources.probe_confidence + temp_sources.acked_confidence;
  if (total_weight > 0.01) {  // Very low threshold - almost always use weighted combination
    DataRate weighted_estimate = 
        (temp_sources.probe_estimate * temp_sources.probe_confidence +
         temp_sources.acked_estimate * temp_sources.acked_confidence) / total_weight;
    
    RTC_LOG(LS_VERBOSE) << "L4S: Base fused estimate (no ECN): " << weighted_estimate.bps() << " bps";
    return weighted_estimate;
  }
  
  // Fallback to most confident non-ECN estimate
  DataRate best_estimate = DataRate::KilobitsPerSec(300);  // Fallback
  double best_confidence = 0.0;
  
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
  
  // Pacing Window (1.0x multiplier is correct for L4S to avoid CE micro-bursts)
  update.pacer_config->data_window = current_rate * update.pacer_config->time_window;
  
  // --- CRITICAL FIX: Enable Padding ---
  // Allow padding up to the app's limit, but never exceed the current L4S target rate
  DataRate padding_rate = max_padding_rate_.value_or(DataRate::Zero());
  padding_rate = std::min(padding_rate, current_rate);
  update.pacer_config->pad_window = padding_rate * update.pacer_config->time_window;
  
  return update;
}

// void webrtc::L4SNetworkController::MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update, Timestamp at_time) {
//   if (!at_time.IsFinite()) {
//     at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
//   }
  
//   NetworkControlUpdate rate_update = CreateRateUpdate(at_time);
  
//   if (rate_update.pacer_config) {
//     update->pacer_config = rate_update.pacer_config;
//   }
//   if (rate_update.target_rate) {
//     update->target_rate = rate_update.target_rate;

//     // Notify ProbeController of a meaningful bitrate change (>5% shift).
//     // Calling SetEstimatedBitrate on every feedback batch would flood
//     // probe_controller.cc's "Measured bitrate" log because that log fires
//     // unconditionally while state == kWaitingForProbingResult.
//     if (probe_controller_) {
//       DataRate target_bitrate = rate_update.target_rate->target_rate;
//       bool is_first_report = last_reported_bitrate_to_probe_controller_.IsZero();
//       bool changed_significantly =
//           is_first_report ||
//           (std::abs(static_cast<int64_t>(target_bitrate.bps()) -
//                     static_cast<int64_t>(
//                         last_reported_bitrate_to_probe_controller_.bps())) >
//            static_cast<int64_t>(
//                0.05 * last_reported_bitrate_to_probe_controller_.bps()));
//       if (changed_significantly) {
//         // Use kLossLimitedBweIncreasing during recovery so ProbeController
//         // records the cause correctly for future RequestProbe decisions.
//         BandwidthLimitedCause cause =
//             recovery_mode_active_
//                 ? BandwidthLimitedCause::kLossLimitedBweIncreasing
//                 : BandwidthLimitedCause::kDelayBasedLimited;
//         auto probes = probe_controller_->SetEstimatedBitrate(
//             target_bitrate, cause, at_time);
//         last_reported_bitrate_to_probe_controller_ = target_bitrate;
//         if (!probes.empty()) {
//           for (const auto& probe : probes) {
//             TimeDelta since_last_probe =
//                 last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity()
//                                               : (at_time - last_probe_time_);
//             if (since_last_probe >= config_.probe_interval) {
//               update->probe_cluster_configs.push_back(probe);
//               last_probe_time_ = at_time;
//               StartProbeHold(at_time);
//             } else {
//               RTC_LOG(LS_VERBOSE)
//                   << "L4S: Dropping bitrate-change probe due to global interval gate";
//             }
//           }
//         }
//       }
//     }
//   }
// }

//ALR based method 

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

    DataRate target_bitrate = rate_update.target_rate->target_rate;

    // --- CRITICAL FIX: Tell ALR Detector our new limit! ---
    if (alr_detector_) {
      alr_detector_->SetEstimatedBitrate(target_bitrate.bps());
    }

    if (probe_controller_) {
      bool is_first_report = last_reported_bitrate_to_probe_controller_.IsZero();
      bool changed_significantly =
          is_first_report ||
          (std::abs(static_cast<int64_t>(target_bitrate.bps()) -
                    static_cast<int64_t>(
                        last_reported_bitrate_to_probe_controller_.bps())) >
           static_cast<int64_t>(
               0.05 * last_reported_bitrate_to_probe_controller_.bps()));
      if (changed_significantly) {
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

// 

//alr included snapshot in state transition evaluation to prevent recovery flapping when ALR ends during congestion

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

  RTC_LOG(LS_INFO)
      << "L4S: State snapshot state=" << StateToString(controller_state_)
      << ", alr=" << IsApplicationLimited()
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
  // within the receiver clock domain.
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
    
    // THE PURE FIX: Do not calculate a rate until the window is statistically significant.
    // We hold the previous valid estimate until we have at least 200ms of new data.
    if (window_interval >= TimeDelta::Millis(200)) {
      last_actual_bitrate_ = DataRate::BitsPerSec(
          static_cast<int64_t>((window_bytes * 8) / window_interval.seconds<double>()));
    } 
    // If < 200ms, we simply do nothing and retain the existing last_actual_bitrate_.
    // DO NOT set it to Zero here, otherwise the headroom checks will break!
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
  
  // Log all 4 bandwidth metrics simultaneously
  DataRate target_rate = target_rate_.value_or(DataRate::Zero());
  metrics_collector_->LogBandwidthMetrics(at_time, 
                                          target_rate,           // 1. Target
                                          last_actual_bitrate_,  // 2. Actual
                                          last_acked_bitrate_,   // 3. Acked
                                          last_send_rate_);      // 4. Send
  
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
  // Headroom cap: block probe if already >2x actual throughput (slightly more generous for recovery)
  if (!EncoderNeedsMoreHeadroom(2.0)) {
    RTC_LOG(LS_VERBOSE) << "L4S: Skipping recovery probe due to headroom cap (2.0x actual throughput).";
    return;
  }
  DataRate current_estimate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  double recovery_multiplier = config_.recovery_probe_multiplier;
  if (IsApplicationLimited()) {
    recovery_multiplier = config_.recovery_alr_probe_multiplier;
  }
  DataRate probe_rate = current_estimate * recovery_multiplier;
  if (max_target_rate_) {
    probe_rate = std::min(probe_rate, *max_target_rate_);
  }
  if (current_estimate < config_.recovery_probe_min_rate) {
    RTC_LOG(LS_VERBOSE) << "L4S: Skipping recovery probe due to low target rate.";
    return;
  }
  DataRate min_useful_probe_rate = current_estimate * config_.min_useful_probe_uplift;
  if (probe_rate < min_useful_probe_rate) {
    RTC_LOG(LS_VERBOSE) << "L4S: Skipping recovery probe due to insufficient uplift.";
    return;
  }
  // --- Bypass WebRTC ProbeController: Call our unified helper ---
  auto custom_probe = CreateCustomProbe(now, probe_rate);
  if (custom_probe) {
    update->probe_cluster_configs.push_back(*custom_probe);
    StartProbeHold(now);
    RTC_LOG(LS_INFO) << "L4S: Manually injected Recovery Probe at " 
                     << probe_rate.bps() << " bps with ID " << custom_probe->id;
  }

}


double webrtc::L4SNetworkController::CalculateProbeConfidence(Timestamp now) const {
  if (last_probe_time_.IsInfinite()) {
    return 0.0;
  }

  // --- CAUSALITY CHECK ---
  // If we received a CE congestion signal AFTER this probe was measured,
  // the network queue has filled. The probe's capacity measurement is dead.
  if (!last_congestion_signal_.IsInfinite() && last_congestion_signal_ >= last_probe_time_) {
    return 0.0; 
  }

  TimeDelta since_probe = now - last_probe_time_;
  
  // In a sparse-feedback L4S architecture, we trust the micro-probe result 
  // for the entire duration of the periodic probe interval (typically 5-8 seconds).
  // This ensures the Fusion Engine doesn't drop the probe discovery before 
  // the next cycle begins.
  if (since_probe < config_.probe_interval) {
    // Return a high confidence (e.g., 0.75+) so it can successfully act 
    // as the Priority 2 capacity ceiling in GetFusedEstimateWithMode.
    return config_.probe_confidence_fresh; 
  } else if (since_probe < (config_.probe_interval * 2)) {
    // If we missed a probe cycle, decay it so it no longer leads the fusion engine,
    // but don't completely discard it yet.
    return config_.probe_confidence_recent; 
  }
  
  // Stale probe data (older than 2 intervals) is no longer trustworthy.
  return 0.2;  
}

double webrtc::L4SNetworkController::CalculateAckedConfidence(Timestamp now) const {
  if (!acked_estimator_) {
    return 0.0;
  }
  
  // CRITICAL L4S OVERRIDE: 
  // The Acknowledged Bitrate is highly volatile and represents current encoder 
  // output, not maximum network capacity. If this confidence is too high, 
  // it will cause a downward spiral during Application-Limited Regions (ALR).
  // 
  // By hard-capping this below the Fusion engine's standard thresholds (0.5+), 
  // we ensure the Acked rate is ONLY used as a reality-check floor, never as 
  // the dictating ceiling when Prague or Probes are active.
  return 0.4; 
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

    // --- CRITICAL FIX: The ALR Recovery Block ---
    // Do not attempt to probe for the network ceiling if the application 
    // is currently the bottleneck.
    if (consecutive_clean_packets_ >= recovery_threshold &&
      clean_duration_ok &&
      rate_ok &&
        !recovery_mode_active_ &&
        !prague_estimator_->IsDiscoveryModeActive() &&
        cooldown_expired &&
        !IsApplicationLimited() ) {

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
