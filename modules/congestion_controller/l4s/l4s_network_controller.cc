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

int webrtc::PragueCapacityEstimator::ComputeAdaptiveNonCeThreshold() const {
  double rtt_ms =
      (current_rtt_.IsFinite() && !current_rtt_.IsZero()) ? current_rtt_.ms() : 120.0;
  double rtt_factor = std::clamp(rtt_ms / 120.0, 0.8, 2.0);
  double alpha_factor = 1.0 + std::clamp(alpha_ / 0.15, 0.0, 1.0);

  int threshold = static_cast<int>(
      std::lround(kNonCeThresholdBase * rtt_factor * alpha_factor));
  return std::clamp(threshold, kNonCeThresholdMin, kNonCeThresholdMax);
}


void webrtc::PragueCapacityEstimator::UpdateFromCongestionSignal(DataRate current_rate, double ce_ratio, Timestamp current_time) {
  last_feedback_time_ = current_time;

  if (ce_ratio > 0.0) {  // CE-marked packets detected
    non_ce_packet_count_ = 0;
    if (discovery_mode_active_ && !first_ce_mark_detected_) {
      discovery_mode_active_ = false;
      first_ce_mark_detected_ = true;
      RTC_LOG(LS_INFO) << "Prague: Exiting discovery mode - first CE mark detected (ce_ratio=" << ce_ratio << ")";
    }
    
    // --- PILLAR 2A: WebRTC-Tuned Gain ---
    // Use 1/8 to make alpha grow fast enough to matter.
    constexpr double g = 1.0 / 8.0;
    alpha_ = (1.0 - g) * alpha_ + g * ce_ratio;

    // --- STEP 2 FIX: The Active Circuit Breaker ---
    // double max_allowed_factor = 1.0; // By default, do not force a cut
    
    // if (current_rtt_.IsFinite() && baseline_rtt_.IsFinite()) {
    //     TimeDelta rtt_delta = current_rtt_ - baseline_rtt_;
    //     if (rtt_delta > TimeDelta::Millis(25)) {
    //         // Stage 2: Aggressive Correction
    //         double bloat_ratio = std::min(rtt_delta.ms() / 150.0, 1.0);
    //         max_allowed_factor = 1.0 - (0.50 * bloat_ratio);
    //     } else if (rtt_delta > TimeDelta::Millis(10)) {
    //         // Stage 1: Mild Correction
    //         max_allowed_factor = 0.90; 
    //     }
    // }

    if (direction_flag_ == 1) {
      direction_flag_ = -1;
      double reduction_factor = 1.0 - alpha_ / 2.0;

      // --- PILLAR 2B: The Panic Drain (Fixed Math) ---
      // // Force the cut deeper if the Circuit Breaker demands it
      // reduction_factor = std::min(reduction_factor, max_allowed_factor);
      
      // Safety bounds (always cut at least 5%, never cut below 20%)
      reduction_factor = std::min(reduction_factor, 0.95);
      reduction_factor = std::max(reduction_factor, 0.20);

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
        // Continuous reductions while still in congestion
        double additional_reduction = 1.0 - alpha_ / 4.0;
        
        // Also apply the circuit breaker to continuous cuts!
        // // Make it slightly gentler than the initial cut to prevent over-draining
        // double continuous_breaker = 1.0 - ((1.0 - max_allowed_factor) * 0.5);
        // additional_reduction = std::min(additional_reduction, continuous_breaker);

        additional_reduction = std::min(additional_reduction, 0.95);
        additional_reduction = std::max(additional_reduction, 0.20);

        DataRate further_reduced = std::max(congestion_based_estimate_ * additional_reduction, min_target_rate_);
        further_reduced = std::max(further_reduced, DataRate::KilobitsPerSec(20));
        congestion_based_estimate_ = further_reduced;
        last_md_time_ = current_time;
        last_ai_update_time_ = current_time;
      }
    }
    last_congestion_signal_ = current_time;
  
  }
  else {  // No CE marks in this batch
    non_ce_packet_count_++;
    
    // --- PLAN 2 FIX: The Anti-Flapping Cooldown ---
    // Calculate a physical clearance window based on the bloated RTT
    TimeDelta clearance_window = TimeDelta::Millis(200); // Safe default
    if (current_rtt_.IsFinite()) {
        // Enforce a strict minimum 2x RTT wait before allowing growth
        clearance_window = current_rtt_ * 2.0; 
    }
    
    bool clearance_time_met = last_md_time_.IsInfinite() || 
                              (current_time - last_md_time_ > clearance_window);

    // Only switch back to Additive Increase if enough PACKETS have passed 
    // AND enough physical TIME has passed to flush the queue.
    int adaptive_non_ce_threshold = ComputeAdaptiveNonCeThreshold();
    if (direction_flag_ == -1 &&
        non_ce_packet_count_ >= adaptive_non_ce_threshold &&
        clearance_time_met) {
      direction_flag_ = 1;
      non_ce_packet_count_ = 0;
      RTC_LOG(LS_VERBOSE)
          << "Prague: Queue drained. Switched to additive mode after "
          << clearance_window.ms() << "ms cooldown (threshold="
          << adaptive_non_ce_threshold << ", rtt_ms="
          << (current_rtt_.IsFinite() ? current_rtt_.ms() : -1)
          << ", alpha=" << alpha_ << ").";
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

void webrtc::PragueCapacityEstimator::OnAckedUpdate(
    Timestamp current_time,
    bool is_app_limited,
    DataRate actual_throughput,
    TimeDelta rtt_bloat) {
  // Feedback-driven update: this marks the connection as alive for growth logic.
  last_feedback_time_ = current_time;

  if (last_update_time_.IsInfinite() || last_ai_update_time_.IsInfinite()) {
    last_update_time_ = current_time;
    last_ai_update_time_ = current_time;
    return;
  }
  
  TimeDelta ai_elapsed = current_time - last_ai_update_time_;
  if (ai_elapsed < TimeDelta::Millis(1)) {
    return; 
  }

  bool network_is_alive = !last_feedback_time_.IsInfinite() && 
                          (current_time - last_feedback_time_) < TimeDelta::Seconds(10);

  // Probe is a short-lived growth indicator, never a direct rate setter.
  if (!probe_constraint_time_.IsInfinite() &&
      (current_time - probe_constraint_time_) >= TimeDelta::Seconds(10)) {
    ClearProbeConstraint();
  }

  // 1. Growth Logic (ALR Aware)
  if (direction_flag_ == 1 && network_is_alive) {
    bool queue_is_clear = rtt_bloat < TimeDelta::Millis(15);
    bool past_hold_time = additive_hold_until_.IsInfinite() ||
                          current_time >= additive_hold_until_;

    if (!is_app_limited && queue_is_clear && past_hold_time) {
      double rtt_s = current_rtt_.IsFinite() && !current_rtt_.IsZero()
                         ? current_rtt_.seconds<double>()
                         : 0.05;
      double elapsed_s = ai_elapsed.seconds<double>();
      double current_bps = static_cast<double>(congestion_based_estimate_.bps());

      // GCC-like additive slope model: min 4 kbps/s, response time = 2*(RTT+100ms).
      constexpr double kFrameIntervalSeconds = 1.0 / 30.0;
      constexpr double kPacketSizeBytes = 1200.0;
      constexpr double kMinIncreaseBpsPerSec = 4000.0;

      double frame_size_bytes =
          std::max(1.0, (current_bps / 8.0) * kFrameIntervalSeconds);
      double packets_per_frame =
          std::max(1.0, std::ceil(frame_size_bytes / kPacketSizeBytes));
      double avg_packet_size_bytes = frame_size_bytes / packets_per_frame;

      double response_time_s = std::max(0.02, 2.0 * (rtt_s + 0.1));
      double increase_rate_bps_per_s =
          std::max(kMinIncreaseBpsPerSec,
                   (avg_packet_size_bytes * 8.0) / response_time_s);

      double desired_step_bps = increase_rate_bps_per_s * elapsed_s;
      
      // --- THE SLOW START BYPASS ---
      double max_step_bps;
      if (discovery_mode_active_) {
          // FAST GROWTH: Allow 50% growth per second during discovery
          max_step_bps = std::max(10000.0, current_bps * 0.50 * elapsed_s);
      } else {
          // SMOOTH GLIDE: Strict 3% growth per second during congestion avoidance
          max_step_bps = std::max(1000.0, current_bps * 0.03 * elapsed_s);
      }
      
      double bounded_step_bps = std::min(desired_step_bps, max_step_bps);

      ai_bits_accumulator_ += bounded_step_bps;
      if (ai_bits_accumulator_ >= 1.0) {
        int64_t whole_bits_per_sec = static_cast<int64_t>(ai_bits_accumulator_);
        DataRate proposed_rate =
            congestion_based_estimate_ + DataRate::BitsPerSec(whole_bits_per_sec);

        DataRate bounded_rate = proposed_rate;

        // --- THE DYNAMIC THROUGHPUT TETHER ---
        if (actual_throughput > DataRate::Zero()) {
          DataRate max_allowed;
          if (discovery_mode_active_) {
              // LOOSE TETHER: Allow 2.0x gap to break the chicken-and-egg deadlock
              max_allowed = actual_throughput * 2.0; 
          } else {
              // STRICT TETHER: GCC-style 1.15x tether in steady state
              max_allowed = actual_throughput * 1.15; 
          }
          if (proposed_rate > max_allowed) {
              bounded_rate = std::max(congestion_based_estimate_, max_allowed);
          } else {
              bounded_rate = proposed_rate;
          }
        }

        bool probe_constraint_fresh =
            !probe_constraint_time_.IsInfinite() &&
            (current_time - probe_constraint_time_) < TimeDelta::Seconds(10);
        if (probe_constraint_fresh &&
            probe_constraint_confidence_ > 0.0 &&
            probe_constraint_ > congestion_based_estimate_) {
          bounded_rate = std::min(bounded_rate, probe_constraint_);
        }

        congestion_based_estimate_ = bounded_rate;

        congestion_based_estimate_ =
            std::min(congestion_based_estimate_, max_target_rate_);
        ai_bits_accumulator_ -= whole_bits_per_sec;
      }
    }
  }

  last_ai_update_time_ = current_time;

  // // 2. Mode Escapes & Alpha Decay
  // if (first_ce_mark_detected_ && !discovery_mode_active_) {
  //   TimeDelta since_congestion = current_time - last_congestion_signal_;
  //   if (since_congestion > TimeDelta::Seconds(30)) {
  //     discovery_mode_active_ = true;
  //     first_ce_mark_detected_ = false;
  //   }
  // }

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

void webrtc::PragueCapacityEstimator::OnTimeUpdate(Timestamp current_time,
                                                    bool is_app_limited) {
  OnAckedUpdate(current_time, is_app_limited, DataRate::Zero(),
                TimeDelta::Zero());
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
  if (direction_flag_ == 1 &&
      non_ce_packet_count_ > ComputeAdaptiveNonCeThreshold() * 2) {
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


void webrtc::PragueCapacityEstimator::SetProbeConstraint(DataRate probe_estimate,
                                                         double probe_confidence,
                                                         Timestamp now) {
  probe_constraint_ = probe_estimate;
  probe_constraint_confidence_ = probe_confidence;
  probe_constraint_time_ = now;
  
  RTC_LOG(LS_VERBOSE) << "Prague: Setting probe constraint to " << probe_estimate.bps() 
                   << " bps with confidence " << probe_confidence;
}

void webrtc::PragueCapacityEstimator::ClearProbeConstraint() {
  probe_constraint_ = DataRate::Zero();
  probe_constraint_confidence_ = 0.0;
  probe_constraint_time_ = Timestamp::MinusInfinity();
  
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
    Timestamp now, bool discovery_mode, bool recovery_mode, bool in_reduction, DataRate actual_rate) const {
  (void)now;
  (void)discovery_mode;
  (void)recovery_mode;
  (void)in_reduction;
  (void)actual_rate;
  
  DataRate prague_rate = sources_.ecn_estimate;
  // Fusion acts as a thin safety guard around Prague, not as a co-equal estimator.
  DataRate fused_rate = prague_rate;

  return std::max(fused_rate, DataRate::KilobitsPerSec(20));
}


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
  // Keep probe authority short-lived so stale estimates cannot override Prague.
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

  // Route change must clear path-specific history.
  base_rtt_ = TimeDelta::PlusInfinity(); // <--- ADD THIS LINE
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
  next_probe_allowed_at_ = Timestamp::MinusInfinity();
  demand_high_since_ = Timestamp::MinusInfinity();
  probe_reject_streak_ = 0;
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

  // Growth is feedback-driven via OnTransportPacketsFeedback.
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
  DataRate fused_rate = ApplyStateFusionPolicy(msg.at_time); // (or msg.at_time)
  target_rate_ = fused_rate;

  // // CHATGPT: Downward-only safety clamp. Never push probes up into Prague.
  // if (prague_estimator_ && fused_rate < prague_estimator_->GetCurrentEstimate()) {
  //     prague_estimator_->SetCurrentEstimate(fused_rate);
  // }

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
  if (msg.smoothed) return update;

  if (msg.round_trip_time.IsFinite() && !msg.round_trip_time.IsZero()) {
    
    // 1. Standard EMA Smoothing for WebRTC Core
    if (last_smoothed_rtt_.IsFinite() && !last_smoothed_rtt_.IsZero()) {
      last_smoothed_rtt_ = (last_smoothed_rtt_ * 0.8) + (msg.round_trip_time * 0.2);
    } else {
      last_smoothed_rtt_ = msg.round_trip_time;
    }
    
    // 2. Raw RTT for L4S Reflexes
    last_rtt_ = msg.round_trip_time;

    // 3. Base RTT Tracking (Must use RAW to find the absolute speed-of-light floor)
    TimeDelta raw_safe_floor = std::max(last_rtt_, TimeDelta::Millis(20));
    if (base_rtt_.IsInfinite() || raw_safe_floor < base_rtt_) {
        base_rtt_ = raw_safe_floor;
    }
    
    // 4. Feed the separate signals to their respective engines
    prague_estimator_->UpdateFromRtt(last_rtt_); // Fast
    
    TimeDelta smoothed_safe_rtt = std::max(last_smoothed_rtt_, TimeDelta::Millis(20));
    last_estimated_round_trip_time_ = smoothed_safe_rtt; // Stable
  }
  return update;
}


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

    // Update throughput calculation
  UpdateThroughputWindow(msg);
  
  // Update all bandwidth estimators
  UpdateAllBandwidthEstimators(msg);

  TimeDelta rtt_bloat =
      (last_rtt_.IsFinite() && base_rtt_.IsFinite())
          ? (last_rtt_ - base_rtt_)
          : TimeDelta::Zero();

  if (prague_estimator_ && !msg.packet_feedbacks.empty()) {
    prague_estimator_->OnAckedUpdate(msg.feedback_time, IsApplicationLimited(),
                                     last_actual_bitrate_, rtt_bloat);
  }

  // if (throughput_estimator_) {
  //   last_actual_bitrate_ = throughput_estimator_->GetCurrentEstimate();
  // }
  
  // State-owned probing policy
  ApplyStateProbingPolicy(msg.feedback_time, &update);
  
// State-owned fusion policy
  DataRate fused_rate = ApplyStateFusionPolicy(msg.feedback_time); // or msg.feedback_time
  target_rate_ = fused_rate;

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
      // if (last_rtt_.IsFinite() && !last_rtt_.IsZero()) {
      //     // Exponential moving average: 80% old, 20% new
      //     last_rtt_ = (last_rtt_ * 0.8) + (feedback_min_rtt * 0.2);
      // } else {
      //     last_rtt_ = feedback_min_rtt;
      // }
      
      // --- CRITICAL FIX 1B: Remove RTT Smoothing for Now ---
      last_rtt_ = feedback_min_rtt;


      // Enforce a hard physical minimum of 20ms to prevent division-by-zero explosions
      TimeDelta safe_rtt = std::max(last_rtt_, TimeDelta::Millis(20));
      // --- NEW: Track the physical baseline ---
      if (base_rtt_.IsInfinite() || safe_rtt < base_rtt_) {
          base_rtt_ = safe_rtt;
      }
      
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


void webrtc::L4SNetworkController::ApplyStateEcnPolicy(
    const TransportPacketsFeedback& feedback,
    DataRate base_fused_rate) {
  // --- CRITICAL FIX 2: Always respect the network ---
  // Congestion is congestion, regardless of whether the application is 
  // currently filling the pipe. We must always process ECN feedback.
  ProcessEcnFeedback(feedback, base_fused_rate);
}


// 


void webrtc::L4SNetworkController::ProcessEcnFeedback(const TransportPacketsFeedback& feedback, DataRate current_fused_rate) {
  if (feedback.packet_feedbacks.empty()) {
    return;
  }

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

  // 3. Accumulate Window
  window_ce_count_ += batch_ce_count;
  window_ect_count_ += batch_ect_count;

  // 4. THE WINDOW GATE (Dynamic BDP-Aware processing)
  double rtt_s = last_rtt_.IsFinite() ? last_rtt_.seconds<double>() : 0.1;
  double rate_bps = current_fused_rate.bps();
  double pkt_size_bits = 1400.0 * 8.0;

  // Clamp between 10 (prevent divide-by-zero on low rates) and 200 (prevent waiting forever)
  int dynamic_threshold = std::clamp(static_cast<int>((rate_bps * rtt_s) / pkt_size_bits), 10, 200);

  bool window_expired = (window_start_time_.IsInfinite() || (feedback.feedback_time - window_start_time_) >= window_duration);
  int window_total = window_ce_count_ + window_ect_count_;

  if (window_expired || window_total >= dynamic_threshold) {
    double ce_ratio = (window_total > 0) ? static_cast<double>(window_ce_count_) / window_total : 0.0;

    if (window_total >= 3) {
      if (window_ce_count_ > 0 && probe_caused_congestion) {
        prague_estimator_->SetAdditiveHoldUntil(feedback.feedback_time + (last_rtt_ * 2));
        bandwidth_fusion_->UpdateProbeEstimate(DataRate::Zero(), 0.0, feedback.feedback_time);
      } else {
        // Pure CE math only. Reality Anchor is handled in Fusion.
        prague_estimator_->UpdateFromCongestionSignal(prague_estimator_->GetCurrentEstimate(), ce_ratio, feedback.feedback_time);
      }

      // Safety Floor to prevent total collapse
      if (!last_actual_bitrate_.IsZero()) {
        DataRate floor = last_actual_bitrate_ * 0.5;
        if (prague_estimator_->GetCurrentEstimate() < floor) {
          prague_estimator_->SetCurrentEstimate(floor);
        }
      }
    }

    // Sync confidence with Fusion Engine
    double ecn_confidence = prague_estimator_->GetConfidence(feedback.feedback_time);
    if (prague_estimator_->GetDirectionFlag() == -1) {
      ecn_confidence = std::max(ecn_confidence, 0.95);
    }
    bandwidth_fusion_->UpdateEcnEstimate(prague_estimator_->GetCurrentEstimate(), ecn_confidence, feedback.feedback_time);

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
      bool in_reduction =
          prague_estimator_ && prague_estimator_->GetDirectionFlag() == -1;
      bool recent_congestion = HasRecentCongestionSignals(feedback.feedback_time);
      bool block_probe_uplift = probe_packet_has_ce || in_reduction || recent_congestion;

      double probe_confidence = CalculateProbeConfidence(feedback.feedback_time);
      if (block_probe_uplift) {
        probe_confidence = 0.0;
        RTC_LOG(LS_VERBOSE)
            << "L4S: Suppressing probe authority during active congestion handling"
            << " (probe_ce=" << probe_packet_has_ce
            << ", reduction=" << in_reduction
            << ", recent_congestion=" << recent_congestion << ")";
        if (prague_estimator_) {
          prague_estimator_->ClearProbeConstraint();
        }
      }
      bandwidth_fusion_->UpdateProbeEstimate(*measured_probe_rate, probe_confidence, feedback.feedback_time);

      // Probe indicates headroom; additive increase determines ramp speed.
      if (prague_estimator_ && !block_probe_uplift) {
        DataRate current_prague = prague_estimator_->GetCurrentEstimate();

        // Accept the probe as an upper growth indicator only if it proves uplift.
        if (*measured_probe_rate > (current_prague * 1.05)) {
          DataRate max_uplift = current_prague * 1.5;
          DataRate probe_ceiling =
              std::min(*measured_probe_rate * 0.95, max_uplift);

          probe_reject_streak_ = 0;
          next_probe_allowed_at_ = Timestamp::MinusInfinity();

          RTC_LOG(LS_INFO)
              << "L4S: Probe indicator accepted. Probe="
              << measured_probe_rate->kbps()
              << "k, additive-growth ceiling=" << probe_ceiling.kbps() << "k.";

          prague_estimator_->SetProbeConstraint(probe_ceiling, probe_confidence,
                                                feedback.feedback_time);
        }
      }
    } else {
      probe_reject_streak_ = std::min(probe_reject_streak_ + 1, 4);
      int64_t backoff_seconds =
          std::min<int64_t>(12, 3 * (1LL << (probe_reject_streak_ - 1)));
      next_probe_allowed_at_ = feedback.feedback_time + TimeDelta::Seconds(backoff_seconds);

      RTC_LOG(LS_INFO) << "L4S: Probe result discarded (physically impossible): "
                       << measured_probe_rate->bps() << " bps < actual throughput "
                       << last_actual_bitrate_.bps() << " bps"
                       << ", backoff_s=" << backoff_seconds;
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

// void webrtc::L4SNetworkController::HandlePeriodicProbing(Timestamp now, NetworkControlUpdate* update) {
//   if (!config_.enable_probing || !probe_controller_) {
//     RTC_LOG(LS_WARNING) << "L4S: Probing disabled - enable_probing=" << config_.enable_probing 
//                         << ", probe_controller=" << (probe_controller_ ? "available" : "null");
//     return;
//   }

//   if (!probe_hold_until_.IsInfinite() && now < probe_hold_until_) {
//     RTC_LOG(LS_VERBOSE) << "L4S: Probe hold active, skipping probe scheduling";
//     return;
//   }
  
//   // Recovery probing has higher priority and frequency
//   if (recovery_mode_active_) {
//     TimeDelta since_last_probe = last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity() : (now - last_probe_time_);
//     int64_t probe_ms = since_last_probe.IsInfinite() ? -1 : since_last_probe.ms();
//     RTC_LOG(LS_VERBOSE) << "L4S: Recovery mode active, time since last probe: " << probe_ms << "ms";
//     // Use a dedicated recovery interval, slower than before to avoid CE bursts.
//     TimeDelta recovery_interval = GetRttScaledInterval();
//     if (since_last_probe >= recovery_interval) {
//       RTC_LOG(LS_INFO) << "L4S: Initiating recovery probe!";
//       InitiateRecoveryProbing(now, update);
//       last_probe_time_ = now;
//     }
//     return;
//   }
  
//   // Regular periodic probing
//   bool interval_ok = last_probe_time_.IsInfinite() || (now - last_probe_time_) >= config_.probe_interval;
//   bool probe_allowed = ShouldProbeNow(now);
//   bool should_probe = interval_ok && probe_allowed;
  
//   // More detailed debug logging with INFO level (safe timestamp handling)
//   int64_t time_since_last_ms = last_probe_time_.IsInfinite() ? -1 : (now - last_probe_time_).ms();
//   RTC_LOG(LS_VERBOSE) << "L4S: Probe decision - time_since_last=" << time_since_last_ms
//                    << "ms, interval_req=" << config_.probe_interval.ms() 
//                    << "ms, interval_ok=" << interval_ok 
//                    << ", probe_allowed=" << probe_allowed 
//                    << ", final_decision=" << should_probe;
  
//   if (should_probe) {
//     InitiateProbing(now, update);
//     last_probe_time_ = now;
//   }
// }

// New method with smarter probe scheduling based on application demand, network conditions, and queue state

void webrtc::L4SNetworkController::HandlePeriodicProbing(Timestamp now, NetworkControlUpdate* update) {
  if (!config_.enable_probing || !probe_controller_) {
    return;
  }

  if (!next_probe_allowed_at_.IsInfinite() && now < next_probe_allowed_at_) {
    RTC_LOG(LS_VERBOSE) << "L4S: Probe suppressed by backoff window. next_allowed_in_ms="
                        << (next_probe_allowed_at_ - now).ms();
    return;
  }

  if (!probe_hold_until_.IsInfinite() && now < probe_hold_until_) {
    return;
  }

  // Single-source arbitration: if any probe is already queued in this update,
  // do not schedule an additional custom probe in the same cycle.
  if (update && !update->probe_cluster_configs.empty()) {
    RTC_LOG(LS_VERBOSE)
        << "L4S: Skipping custom probe; a probe is already queued this update.";
    return;
  }
  
  DataRate current_target = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  DataRate send_rate = last_send_rate_;
  DataRate actual_rate = last_actual_bitrate_;
  DataRate acked_rate = last_acked_bitrate_.value_or(DataRate::Zero());
  bool has_acked = acked_rate > DataRate::Zero();
  bool app_limited = IsApplicationLimited();
  
  // TRIGGER A: Probe demand is delivery-driven (not sender burst-driven).
  // If acked is not available/stable, rely on actual throughput only.
  bool demand_from_actual = !actual_rate.IsZero() && actual_rate > (current_target * 0.90);
  bool demand_from_acked = has_acked && acked_rate > (current_target * 0.75);
  bool demand_now = !app_limited && (demand_from_actual || demand_from_acked);

  TimeDelta effective_rtt =
      last_rtt_.IsFinite() && !last_rtt_.IsZero() ? last_rtt_ : TimeDelta::Millis(100);
  TimeDelta demand_hold_time = std::clamp(effective_rtt * 2.0,
                                          TimeDelta::Millis(300),
                                          TimeDelta::Seconds(3));

  if (demand_now) {
    if (demand_high_since_.IsInfinite()) {
      demand_high_since_ = now;
    }
  } else {
    demand_high_since_ = Timestamp::MinusInfinity();
  }

  bool demand_is_sustained = !demand_high_since_.IsInfinite() &&
                             (now - demand_high_since_) >= demand_hold_time;

  // TRIGGER B: Is delivery supporting target (avoid probing on sender bursts alone)?
  bool delivery_supports_target =
      (!actual_rate.IsZero() && actual_rate > (current_target * 0.80)) ||
      (acked_rate > DataRate::Zero() && acked_rate > (current_target * 0.70));
  
  // TRIGGER C: Is the physical queue completely drained?
  // FIX: Fail closed. If we don't know the baseline, assume the queue is bloated.
  TimeDelta rtt_bloat = (last_rtt_.IsFinite() && base_rtt_.IsFinite()) ?
                        (last_rtt_ - base_rtt_) : TimeDelta::PlusInfinity();
  bool queue_is_empty_recovery = rtt_bloat < TimeDelta::Millis(5);
  bool queue_is_empty_periodic = rtt_bloat < TimeDelta::Millis(8);

  TimeDelta since_last_probe = last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity() : (now - last_probe_time_);

  // --- 1. EVENT-DRIVEN RECOVERY PROBING ---
  if (recovery_mode_active_) {
    TimeDelta recovery_interval = std::max(GetRttScaledInterval(), TimeDelta::Seconds(3));
    
    if (since_last_probe >= recovery_interval) {
        // Probe only if app demand is sustained, delivery supports it, and queue is truly clear.
        if (demand_is_sustained && delivery_supports_target && queue_is_empty_recovery) {
            RTC_LOG(LS_INFO) << "L4S: Event-Driven Recovery Probe Triggered! Send(" 
                             << send_rate.kbps() << "k) Actual(" << actual_rate.kbps()
                             << "k) Acked(" << acked_rate.kbps() << "k) Target("
                             << current_target.kbps() << "k). base rtt (" 
                             << base_rtt_.ms() << "ms ). last rtt (" 
                             << last_rtt_.ms() << "ms ). Queue is empty (" 
                             << rtt_bloat.ms() << "ms bloat).";
            InitiateRecoveryProbing(now, update);
            last_probe_time_ = now;
        } else {
            RTC_LOG(LS_VERBOSE)
                << "L4S: Recovery probe suppressed. app_limited=" << app_limited
                << ", demand_sustained=" << demand_is_sustained
                << ", delivery_supports_target=" << delivery_supports_target
                << ", bloat_ms=" << rtt_bloat.ms();
        }
    }
    return;
  }
  
  // --- 2. EVENT-DRIVEN PERIODIC PROBING ---
  bool cooldown_ok = since_last_probe >= TimeDelta::Seconds(3); // Don't spam
  
  if (cooldown_ok && ShouldProbeNow(now)) {
      if (demand_is_sustained && delivery_supports_target && queue_is_empty_periodic) {
          RTC_LOG(LS_INFO) << "L4S: Event-Driven Periodic Probe Triggered! Send(" 
                           << send_rate.kbps() << "k) Actual(" << actual_rate.kbps()
                           << "k) Acked(" << acked_rate.kbps() << "k) Target("
                           << current_target.kbps() << "k). base rtt (" 
                           << base_rtt_.ms() << "ms ). last rtt (" 
                           << last_rtt_.ms() << "ms ). Queue is empty (" 
                           << rtt_bloat.ms() << "ms bloat).";
          InitiateProbing(now, update);
          last_probe_time_ = now;
      }
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
  if (last_rtt_.IsFinite() && last_rtt_ > TimeDelta::Millis(250)) {
      RTC_LOG(LS_VERBOSE) << "L4S: Blocking probe due to ABSOLUTE high RTT: " << last_rtt_.ms() << "ms";
      return false;
  }

  // --- HIGH-BDP FIX: Global Latency Gate (Fail Closed) ---
  if (last_rtt_.IsFinite() && base_rtt_.IsFinite()) {
      if (last_rtt_ - base_rtt_ > TimeDelta::Millis(15)) {
          RTC_LOG(LS_VERBOSE) << "L4S: Blocking periodic probe due to standing queue bloat.";
          return false;
      }
  } else {
      // If we don't have a baseline yet, we can't prove the path is clear.
      return false;
  }

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
  // Keep probe byte budget small to avoid token-bucket overflow on strict shapers.
  custom_probe.target_duration = TimeDelta::Millis(10);
  custom_probe.target_probe_count = 4;
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
  (controller_state_ == ControllerState::kSlowState) && discovery_signal;

  // Keep discovery semantics active during route reset until state machine settles.
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
  
  // Determine if Prague is actively slashing the rate to clear a queue
  bool in_reduction = prague_estimator_ && prague_estimator_->GetDirectionFlag() == -1;

  // Prague is the authority; fusion acts only as a guard layer.
  DataRate prague_rate = prague_estimator_->GetCurrentEstimate();
  DataRate guard_rate =
      bandwidth_fusion_->GetFusedEstimateWithMode(now, discovery_active,
                                                  recovery_active, in_reduction, last_actual_bitrate_);
  DataRate fused_rate = std::min(prague_rate, guard_rate);

  RTC_LOG(LS_VERBOSE) << "L4S: Prague authority with fusion guard - Prague: "
                      << prague_rate.bps() << " bps, guard: "
                      << guard_rate.bps() << " bps, selected: "
                      << fused_rate.bps() << " bps";
  
  // Apply rate constraints
  if (min_target_rate_ && fused_rate < *min_target_rate_) {
    fused_rate = *min_target_rate_;
  }
  // Final safety: enforce absolute minimum of 20 kbps to prevent pacer crashes
  fused_rate = std::max(fused_rate, DataRate::KilobitsPerSec(20));
  if (max_target_rate_ && fused_rate > *max_target_rate_) {
    fused_rate = *max_target_rate_;
  }
  
  return fused_rate;
}

webrtc::DataRate webrtc::L4SNetworkController::ApplyStateFusionPolicy(
    Timestamp now) {
  switch (controller_state_) {
    case ControllerState::kRouteReset:
    case ControllerState::kSlowState:
    case ControllerState::kCongestionAvoidance:
    case ControllerState::kCongestionExperienced:
    case ControllerState::kCongestionRecovery:
      return FuseBandwidthEstimates(now);
  }
  return FuseBandwidthEstimates(now);
}

webrtc::DataRate webrtc::L4SNetworkController::GetBaseFusedEstimate(Timestamp now) {
  (void)now;

  // Keep ECN window sizing aligned with Prague to avoid cross-signal tug-of-war.
  if (prague_estimator_) {
    return prague_estimator_->GetCurrentEstimate();
  }
  return target_rate_.value_or(DataRate::KilobitsPerSec(300));
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
  // Tight burst window to avoid token-bucket collisions on strict shapers.
  update.pacer_config->time_window = TimeDelta::Millis(5);
  
  // Use a modest pacing headroom multiplier for L4S while avoiding large bursts.
  DataRate pacing_rate = current_rate * 1.15;
  update.pacer_config->data_window = pacing_rate * update.pacer_config->time_window;
  
  // --- CRITICAL FIX: Enable Padding ---
  // Allow padding up to the app's limit, but never exceed the current L4S target rate
  DataRate padding_rate = max_padding_rate_.value_or(DataRate::Zero());
  padding_rate = std::min(padding_rate, current_rate);
  update.pacer_config->pad_window = padding_rate * update.pacer_config->time_window;
  
  return update;
}

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
            if (!update->probe_cluster_configs.empty()) {
              RTC_LOG(LS_VERBOSE)
                  << "L4S: Dropping bitrate-change probe due to existing queued probe";
              break;
            }
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
      if (recovery_active && recovery_cooldown_ok && dwell_ok) {
        return TransitionDecision{ControllerState::kCongestionRecovery,
                                  TransitionReason::kRecoveryEntered};
      }
      if (reduction_active) {
        return TransitionDecision{ControllerState::kCongestionExperienced,
                                  TransitionReason::kPragueReduction};
      }
      if (discovery_active) {
        return TransitionDecision{ControllerState::kSlowState,
                                  TransitionReason::kInitComplete};
      }
      return TransitionDecision{ControllerState::kCongestionAvoidance,
                                TransitionReason::kInitComplete};

    case ControllerState::kSlowState:
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
      if (reduction_active && dwell_ok) {
        return TransitionDecision{ControllerState::kCongestionExperienced,
                                  TransitionReason::kPragueReduction};
      }
      if (recovery_active && recovery_cooldown_ok && dwell_ok) {
        return TransitionDecision{ControllerState::kCongestionRecovery,
                                  TransitionReason::kRecoveryEntered};
      }
      return std::nullopt;

    case ControllerState::kCongestionExperienced:
      if (recovery_active && recovery_cooldown_ok && dwell_ok) {
        return TransitionDecision{ControllerState::kCongestionRecovery,
                                  TransitionReason::kRecoveryEntered};
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
    case ControllerState::kSlowState:
      return "slow_state";
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
  
  // --- LOGGING FREQUENCY FIX ---
  // Throttle RTT logging to 500ms (GCC RTCP speed) to prevent measurement bias
  // on the scatter plots, while keeping the high-speed RTT for internal math.
  static Timestamp last_delay_log_time = Timestamp::MinusInfinity();
  if (last_rtt_.IsFinite() && (last_delay_log_time.IsInfinite() || 
                              (at_time - last_delay_log_time) >= TimeDelta::Millis(500))) {
      
    metrics_collector_->LogDelayMetrics(at_time, last_rtt_, last_rtt_ / 2, TimeDelta::Zero());
    last_delay_log_time = at_time;
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

  // Exit on convergence only after a small dwell and with no very recent CE signal.
  bool dwell_met = state_entered_at_.IsInfinite() ||
                   (now - state_entered_at_) >= TimeDelta::Seconds(2);
  if (dwell_met && !HasRecentCongestionSignals(now) &&
      CheckProbeAndPragueConvergence(now)) {
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

    // --- CRITICAL FIX: Calculate the physical queue bloat ---
    TimeDelta rtt_bloat = TimeDelta::Zero();
    if (last_rtt_.IsFinite() && base_rtt_.IsFinite()) {
        rtt_bloat = last_rtt_ - base_rtt_;
    }

    // Do not attempt to probe for the network ceiling if the application 
    // is currently the bottleneck OR if the queue is still clearing.
    if (consecutive_clean_packets_ >= recovery_threshold &&
        clean_duration_ok &&
        rate_ok &&
        !recovery_mode_active_ &&
        !prague_estimator_->IsDiscoveryModeActive() &&
        cooldown_expired &&
        !IsApplicationLimited() && 
        rtt_bloat < TimeDelta::Millis(15)) { // <--- THE LATENCY GATE

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
