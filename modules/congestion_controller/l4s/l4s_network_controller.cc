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

PragueCapacityEstimator::PragueCapacityEstimator(DataRate starting_rate, 
                                                 DataRate min_rate, 
                                                 DataRate max_rate)
    : congestion_based_estimate_(starting_rate),
      min_target_rate_(min_rate),
      max_target_rate_(max_rate),
      pre_loss_target_(DataRate::Zero()),
      current_rtt_(TimeDelta::Millis(50)),
      baseline_rtt_(TimeDelta::PlusInfinity()),
      last_update_time_(Timestamp::MinusInfinity()),
      last_feedback_time_(Timestamp::MinusInfinity()),
      last_congestion_signal_(Timestamp::MinusInfinity()),
      last_md_time_(Timestamp::MinusInfinity()),
      last_ai_update_time_(Timestamp::MinusInfinity()),
      additive_hold_until_(Timestamp::MinusInfinity()),
      last_ecn_feedback_(Timestamp::MinusInfinity()),
      last_hard_loss_time_(Timestamp::MinusInfinity()),
      alpha_(0.0),
      ai_bits_accumulator_(0.0),
      non_ce_packet_count_(0),    
      accumulated_lost_packets_(0),
      accumulated_expected_packets_(0),
      discovery_mode_active_(true)
      // first_ce_mark_detected_(false) 
      {
  
  // Rate clamping logic
  if (congestion_based_estimate_ < min_target_rate_) {
    congestion_based_estimate_ = min_target_rate_;
  } else if (congestion_based_estimate_ > max_target_rate_) {
    congestion_based_estimate_ = max_target_rate_;
  }
}


PragueCapacityEstimator::~PragueCapacityEstimator() = default;


enum class PragueMode {
  kBaseline = 0,
  kQueueAssist = 1,
  kCeDensity = 2
};

// Change this to run experiments
constexpr PragueMode kPragueMode = PragueMode::kQueueAssist;

int webrtc::PragueCapacityEstimator::ComputeAdaptiveNonCeThreshold() const {
  double rtt_ms =
      (current_rtt_.IsFinite() && !current_rtt_.IsZero()) ? current_rtt_.ms() : 120.0;
  double rtt_factor = std::clamp(rtt_ms / 120.0, 0.8, 2.0);
  double alpha_factor = 1.0 + std::clamp(alpha_ / 0.15, 0.0, 1.0);

  int threshold = static_cast<int>(
      std::lround(kNonCeThresholdBase * rtt_factor * alpha_factor));
  return std::clamp(threshold, kNonCeThresholdMin, kNonCeThresholdMax);
}

// // Continuous Equilibrium Engine - Multiplicative Decrease Step
// void webrtc::PragueCapacityEstimator::UpdateFromCongestionSignal(
//     DataRate current_rate, double ce_ratio, int window_packet_count, Timestamp current_time, DataRate historical_max) {
//   last_feedback_time_ = current_time;

//   if (ce_ratio > 0.0) {  
//     non_ce_packet_count_ = 0;

//     TimeDelta queue_bloat = current_rtt_.IsFinite() && baseline_rtt_.IsFinite() 
//                             ? (current_rtt_ - baseline_rtt_) 
//                             : TimeDelta::Millis(0);
//     TimeDelta pipeline_delay = (current_rtt_.IsFinite() ? current_rtt_ : TimeDelta::Millis(100)) + queue_bloat;
//     pipeline_delay = std::max(pipeline_delay, TimeDelta::Millis(50));

//     bool gate_open = last_md_time_.IsInfinite() || (current_time - last_md_time_ >= pipeline_delay);

//     if (discovery_mode_active_ && !first_ce_mark_detected_) {
//       discovery_mode_active_ = false;
//       first_ce_mark_detected_ = true;
      
//       if (ce_ratio < 0.10) {
//         RTC_LOG(LS_INFO) << "Prague: Soft Exit from discovery mode (ce_ratio=" << ce_ratio << " < 0.10). Holding rate.";
//         TimeDelta hold_duration = current_rtt_.IsFinite() ? current_rtt_ * 2.0 : TimeDelta::Millis(200);
//         additive_hold_until_ = current_time + hold_duration;
//         last_congestion_signal_ = current_time;
//         return; 
//       }
//     }

    
//       // --- FIX 1: THE ALPHA VELOCITY BUG ---
//       constexpr double g = 1.0 / 8.0;
//       alpha_ = (1.0 - g) * alpha_ + g * ce_ratio;


//     if (gate_open) {

//       // Execute Continuous Multiplicative Decrease
//       double reduction_factor = 1.0 - (alpha_ / 2.0);
//       reduction_factor = std::clamp(reduction_factor, 0.50, 1.0); 

//       DataRate reduced = std::max(current_rate * reduction_factor, min_target_rate_);

//       // =========================================================
//       // --- C6: ELASTIC BULLY SHIELD ---
//       // =========================================================
//       DataRate bully_floor = DataRate::KilobitsPerSec(600);
//       if (historical_max > DataRate::Zero()) {
//           bully_floor = std::max(bully_floor, historical_max * 0.45);
//       }

//       if (reduced < bully_floor && ce_ratio > 0.05) {
//           // ELASTIC YIELD: 5% micro-cut instead of a concrete wall
//           reduced = std::max(current_rate * 0.95, min_target_rate_);
//           alpha_ *= 0.5; // Halve alpha debt while shielding
//           RTC_LOG(LS_WARNING) << "L4S: Elastic Bully Shield engaged! 5% micro-cut. Rate=" << reduced.kbps() << " kbps.";
//       }


//       congestion_based_estimate_ = std::max(reduced, DataRate::KilobitsPerSec(20));
//       last_md_time_ = current_time;

//       RTC_LOG(LS_INFO) << "L4S: Continuous MD (alpha=" << alpha_
//                        << ", reduction_factor=" << reduction_factor
//                        << "), target=" << congestion_based_estimate_.bps() 
//                        << " bps.";
//     }
//     last_congestion_signal_ = current_time;
//   }
//   else {  
//     non_ce_packet_count_ += window_packet_count;
//   } 
// }


DataRate PragueCapacityEstimator::ApplyBullyShield(DataRate calculated_rate, DataRate historical_max) {
  DataRate bully_floor = DataRate::KilobitsPerSec(600);
  if (historical_max > DataRate::Zero()) {
    bully_floor = std::max(bully_floor, historical_max * 0.40);
  }

  if (calculated_rate < bully_floor) {
    RTC_LOG(LS_INFO) << "L4S: Bully Shield ENGAGED. Clamping rate from " 
                     << calculated_rate.kbps() << " kbps up to floor " << bully_floor.kbps() << " kbps.";
    return bully_floor;
  }
  return calculated_rate;
}

void webrtc::PragueCapacityEstimator::UpdateFromCongestionSignal(
    DataRate current_rate,
    double ce_ratio,
    int window_packet_count,
    Timestamp current_time,
    DataRate historical_max) {

  last_feedback_time_ = current_time;
  
  if (ce_ratio <= 0.0) {
    non_ce_packet_count_ += window_packet_count;
    return;
  }

  non_ce_packet_count_ = 0;

  if (discovery_mode_active_) {
    discovery_mode_active_ = false;
    RTC_LOG(LS_INFO) << "L4S: Hard exit from discovery mode due to CE marks.";
  }

  // =========================================================
  // 1. RTT + queue estimation (NORMALIZED TO AQM CLIFF)
  // =========================================================
  TimeDelta queue_delay = TimeDelta::Millis(0);

  if (current_rtt_.IsFinite() && baseline_rtt_.IsFinite()) {
    queue_delay = std::max(current_rtt_ - baseline_rtt_,
                           TimeDelta::Millis(0));
  }

  double queue_pressure = 0.0;
  
  double retention_high = 1.0;
  double retention_low = 0.5;

  if (queue_delay.ms() > 0) {
    // 100.0ms represents a typical physical AQM drop timer limit
    queue_pressure = static_cast<double>(queue_delay.ms()) / 100.0;
  }

  queue_pressure = std::clamp(queue_pressure, 0.0, 1.0);

  // =========================================================
  // 2. PRAGUE ALPHA (CORE SIGNAL)
  // =========================================================
  constexpr double g = 1.0 / 16.0;

  alpha_ = (1.0 - g) * alpha_ + g * ce_ratio;

  double effective_alpha = alpha_;

  // =========================================================
  // 3. MODE SWITCHING
  // =========================================================

  if (kPragueMode == PragueMode::kQueueAssist) {

    // Smooth assist (no hard threshold)
    double assist =queue_pressure * (1.0 - alpha_);

    effective_alpha = std::max(alpha_, assist);
  
  }

  else if (kPragueMode == PragueMode::kCeDensity) {

    // =====================================================
    // FIXED CE DENSITY MODEL (history-free, stable)
    // =====================================================

    static double avg_ce_ratio = 0.01;
    constexpr double beta = 0.05;

    avg_ce_ratio =
        (1.0 - beta) * avg_ce_ratio +
        beta * ce_ratio;

    double density =
        ce_ratio / std::max(avg_ce_ratio, 1e-6);

    density = std::clamp(density, 0.3, 3.0);

    effective_alpha = alpha_ * density;
  }

  // =========================================================
  // 4. GATE (ACCELERATED UNDER PRESSURE)
  // =========================================================
  TimeDelta pipeline_delay =
      current_rtt_.IsFinite()
          ? current_rtt_
          : TimeDelta::Millis(180);

  // Shrink the pipeline delay up to 25% under severe queue pressure
  // to force faster reaction before tail-drops occur.
  pipeline_delay = pipeline_delay * (1.0 - (queue_pressure * 0.25));

  bool gate_open =
      last_md_time_.IsInfinite() ||
      (current_time - last_md_time_) >= pipeline_delay;

  if (!gate_open) {
    last_congestion_signal_ = current_time;
    return;
  }

  // =========================================================
  // 5. MULTIPLICATIVE DECREASE
  // =========================================================
  double reduction_factor = 1.0 - (effective_alpha / 2.0);

  if (queue_pressure > 0.15) {
    retention_high = 0.95;
    retention_low = 0.50; 
  }

  else {
    retention_high = 1.0;
    retention_low = 0.7; 
  }
  
  reduction_factor = std::clamp(reduction_factor, retention_low, retention_high);
  
  DataRate reduced = current_rate * reduction_factor;

  // =========================================================
  // 6. BULLY SHIELD (unchanged but stable)
  // =========================================================
  DataRate clamped_rate = ApplyBullyShield(reduced, historical_max);
  congestion_based_estimate_ = std::max(clamped_rate, DataRate::KilobitsPerSec(300));

  // =========================================================
  // 7. COMMIT
  // =========================================================
  // congestion_based_estimate_ = std::max(reduced, DataRate::KilobitsPerSec(300));
 
  last_md_time_ = current_time;
  last_congestion_signal_ = current_time;

  RTC_LOG(LS_INFO) << "L4S: Continuous MD (alpha=" << alpha_
                  << ", reduction_factor=" << reduction_factor
                  << "), target=" << congestion_based_estimate_.bps() 
                  << " bps.";
}

void webrtc::PragueCapacityEstimator::EnterAdditiveMode(Timestamp current_time) {
  non_ce_packet_count_ = 0;
  ai_bits_accumulator_ = 0.0;
  last_ai_update_time_ = current_time;
  additive_hold_until_ = Timestamp::MinusInfinity();
  last_update_time_ = current_time;
  last_feedback_time_ = current_time;
}

void webrtc::PragueCapacityEstimator::UpdateEcnActivity(Timestamp current_time) {
  last_ecn_feedback_ = current_time;
}

void webrtc::PragueCapacityEstimator::UpdateFromRtt(TimeDelta rtt) {
  if (rtt.IsFinite() && !rtt.IsZero()) {
    current_rtt_ = rtt;
    if (baseline_rtt_.IsInfinite() || rtt < baseline_rtt_) {
        baseline_rtt_ = rtt;
    }
  }
}

void webrtc::PragueCapacityEstimator::OnPacketLoss(DataRate current_rate, Timestamp current_time, int lost_packets, int total_packets, DataRate historical_max) {
  // Accumulate loss incrementally as WebRTC reports tiny asynchronous batches
  accumulated_lost_packets_ += lost_packets;
  accumulated_expected_packets_ += total_packets;

  TimeDelta rtt = current_rtt_.IsFinite() ? current_rtt_ : TimeDelta::Millis(175);
  
  // Only react to hard loss once per RTT to prevent cascading collapse
  if (last_hard_loss_time_.IsInfinite() || (current_time - last_hard_loss_time_ >= rtt)) {
      
      // --- FIX 2: THE MICRO-BATCH LOSS TRAP ---
      double loss_ratio = (accumulated_expected_packets_ > 0) ? 
          (static_cast<double>(accumulated_lost_packets_) / accumulated_expected_packets_) : 0.0;

      RTC_LOG(LS_WARNING) << "L4S: HARD LOSS DETECTED! Ratio: " << loss_ratio 
                          << ". Packets lost: " << accumulated_lost_packets_ 
                          << " / " << accumulated_expected_packets_ 
                          << ". Current target: " << current_rate.kbps() << " kbps.";
          
      accumulated_lost_packets_ = 0;
      accumulated_expected_packets_ = 0;
      
      // --- THE GCC LOSS THRESHOLD ---
      if (loss_ratio <= 0.02) {
          return;
      }

      // Save Pre-Loss Memory for Fast Convergence
      if (pre_loss_target_ == DataRate::Zero() || congestion_based_estimate_ > pre_loss_target_) {
          pre_loss_target_ = congestion_based_estimate_;
      }

      // --- PURE WEBRTC/GCC LOSS MATH (No TCP Mimicry) ---
      // You are right: We are building for RTP, not TCP. 
      // GCC explicitly scales the penalty by exactly half of the loss ratio.
      // e.g., 10% loss = 5% cut. 40% loss = 20% cut.
      // It naturally bounds itself (100% loss = 50% cut maximum). No artificial caps needed!
      double loss_penalty = 0.5 * loss_ratio;
      // double loss_penalty = 1.0 * loss_ratio;
      
      double retention_factor = 1.0 - loss_penalty;
      DataRate reduced = std::max(current_rate * retention_factor, min_target_rate_);
      // Apply the same structural shield to the packet loss path
      DataRate clamped_rate = ApplyBullyShield(reduced, historical_max); 
      congestion_based_estimate_ = std::max(clamped_rate, DataRate::KilobitsPerSec(300));
      // Reset Alpha so the continuous CE engine doesn't double-penalize the drop
      alpha_ = 0.0;
      last_hard_loss_time_ = current_time;
      last_update_time_ = current_time;
      
      RTC_LOG(LS_WARNING) << "L4S: PURE RTP HARD LOSS! Ratio: " << loss_ratio 
                          << ". Slashed target gently by " << (loss_penalty * 100) << "% to " 
                          << congestion_based_estimate_.kbps() 
                          << " kbps. Memorized pre-loss target: " << pre_loss_target_.kbps() << " kbps.";
  }
}




double webrtc::PragueCapacityEstimator::CalculateDiscoveryStep(double current_bps, double elapsed_s) const {
  double growth_factor = 1.1; 
  growth_factor = std::pow(growth_factor, std::min(elapsed_s, 1.0)); 
  return std::max(current_bps * (growth_factor - 1.0), 1000.0);
}

double webrtc::PragueCapacityEstimator::CalculateRecoveryStep(double current_bps, double target_probe_bps, double elapsed_s) const {
  double gap_bps = target_probe_bps - current_bps;
  double catch_up_rate_bps_per_s = std::max(gap_bps * 0.33, 100000.0); 
  catch_up_rate_bps_per_s = std::min(catch_up_rate_bps_per_s, 500000.0);
  return catch_up_rate_bps_per_s * elapsed_s;
}

double webrtc::PragueCapacityEstimator::CalculateStableStep(double current_bps, double elapsed_s) const {
  double increase_rate_bps_per_s = std::max(10000.0, current_bps * 0.07); 
  increase_rate_bps_per_s = std::min(increase_rate_bps_per_s, 500000.0);
  return increase_rate_bps_per_s * elapsed_s;
}

webrtc::DataRate webrtc::PragueCapacityEstimator::ApplyThroughputTether(
    DataRate proposed_rate, DataRate actual_throughput) const {
  if (actual_throughput <= DataRate::Zero()) {
    return proposed_rate;
  }

  // --- THE HYBRID HEADROOM FIX ---
  // If the video encoder drops resolution, 15% headroom is mathematically 
  // too small to ever trigger an upgrade. We provide an absolute flat runway.
  DataRate relative_cap = discovery_mode_active_ ? (actual_throughput * 2.0) : (actual_throughput * 1.35); // Boosted to 35%
  DataRate absolute_cap = actual_throughput + DataRate::KilobitsPerSec(1500); // +1.5 Mbps flat headroom!
  
  DataRate max_allowed = std::max(relative_cap, absolute_cap);
  
  if (proposed_rate > max_allowed) {
    return std::max(congestion_based_estimate_, max_allowed);
  }
  return proposed_rate;
}

void webrtc::PragueCapacityEstimator::DecayAlpha(Timestamp current_time) {
  if (alpha_ <= 0.0) return;

  TimeDelta since_last_ce = current_time - last_congestion_signal_;
  TimeDelta safe_clearance = current_rtt_.IsFinite() ? (current_rtt_ * 2) : TimeDelta::Millis(100);

  if (!last_congestion_signal_.IsInfinite() && since_last_ce > safe_clearance) {
    alpha_ *= 0.80; 
    if (alpha_ < 0.001) alpha_ = 0.0;
  }
}

// Continuous Equilibrium Engine - Additive Increase Step
void webrtc::PragueCapacityEstimator::OnAckedUpdate(
    Timestamp current_time,
    bool is_app_limited,
    DataRate actual_throughput,
    TimeDelta rtt_bloat) {
  
  last_feedback_time_ = current_time;

  if (last_update_time_.IsInfinite() || last_ai_update_time_.IsInfinite()) {
    last_update_time_ = current_time;
    last_ai_update_time_ = current_time;
    return;
  }
  
  TimeDelta ai_elapsed = current_time - last_ai_update_time_;
  if (ai_elapsed < TimeDelta::Millis(1)) return;

  bool network_is_alive = !last_feedback_time_.IsInfinite() && 
                          (current_time - last_feedback_time_) < TimeDelta::Seconds(10);

  if (!probe_constraint_time_.IsInfinite() &&
      (current_time - probe_constraint_time_) >= TimeDelta::Seconds(10)) {
    ClearProbeConstraint();
  }

  // --- THE CONTINUOUS GROWTH PIPELINE ---
  TimeDelta rtt = current_rtt_.IsFinite() ? current_rtt_ : TimeDelta::Millis(175);
  bool in_hard_loss_cooldown = !last_hard_loss_time_.IsInfinite() && ((current_time - last_hard_loss_time_) < rtt);

  // We grow continuously unless recovering from a physical packet drop
  if (!in_hard_loss_cooldown && network_is_alive) {
    bool queue_is_clear = rtt_bloat < TimeDelta::Millis(30);
    bool past_hold_time = additive_hold_until_.IsInfinite() || current_time >= additive_hold_until_;
    
    DataRate proposed_rate = congestion_based_estimate_;
    double current_bps = static_cast<double>(congestion_based_estimate_.bps());
    double elapsed_s = ai_elapsed.seconds<double>();

    bool probe_pulling_up = !probe_constraint_time_.IsInfinite() && 
                            (current_time - probe_constraint_time_) < TimeDelta::Seconds(5) &&
                            probe_constraint_ > (congestion_based_estimate_ * 1.05);

    if (queue_is_clear && past_hold_time) {
      double final_step_bps = 0.0;

      if (!is_app_limited) {
        if (discovery_mode_active_) {
          final_step_bps = CalculateDiscoveryStep(current_bps, elapsed_s);
        } else if (probe_pulling_up) {
          final_step_bps = CalculateRecoveryStep(current_bps, static_cast<double>(probe_constraint_.bps()), elapsed_s);
        } else {
          final_step_bps = CalculateStableStep(current_bps, elapsed_s);
          
          // --- NEW: FAST CONVERGENCE (The 4x Multiplier) ---
          if (congestion_based_estimate_ < pre_loss_target_) {
              final_step_bps *= 4.0; 
              // Note: This is safe because 'queue_is_clear' is true, meaning ce_ratio == 0.
          }
        }
      } 
      else {
        bool delivery_supports_nudge = actual_throughput >= (congestion_based_estimate_ * 0.80);
        if (probe_pulling_up || delivery_supports_nudge) {
          double alr_nudge_bps_per_s = std::clamp(current_bps * 0.002, 2000.0, 10000.0);
          if (discovery_mode_active_) {
            alr_nudge_bps_per_s = std::clamp(current_bps * 0.005, 5000.0, 20000.0);
          }
          final_step_bps = alr_nudge_bps_per_s * elapsed_s;
          
          // Fast convergence applies even in ALR to push the encoder up
          if (congestion_based_estimate_ < pre_loss_target_) {
              final_step_bps *= 4.0;
          }
        }
      }

      ai_bits_accumulator_ += final_step_bps;
    }

    if (ai_bits_accumulator_ >= 1.0) {
      int64_t whole_bits = static_cast<int64_t>(ai_bits_accumulator_);
      proposed_rate = congestion_based_estimate_ + DataRate::BitsPerSec(whole_bits);
      ai_bits_accumulator_ -= whole_bits; 
    }

    proposed_rate = ApplyThroughputTether(proposed_rate, actual_throughput);

    bool probe_constraint_fresh = !probe_constraint_time_.IsInfinite() &&
                                  (current_time - probe_constraint_time_) < TimeDelta::Seconds(10);
    if (probe_constraint_fresh && probe_constraint_ > congestion_based_estimate_) {
      proposed_rate = std::min(proposed_rate, probe_constraint_);
    }

    congestion_based_estimate_ = std::min(proposed_rate, max_target_rate_);
  }

  last_ai_update_time_ = current_time;

  // --- GRACEFUL MEMORY CLEARING ---
  if (pre_loss_target_ > DataRate::Zero()) {
      if (congestion_based_estimate_ >= pre_loss_target_) {
          RTC_LOG(LS_VERBOSE) << "L4S: Pre-loss target reclaimed. Disengaging GCC styled Fast Recovery.";
          pre_loss_target_ = DataRate::Zero();
      } else if (!last_hard_loss_time_.IsInfinite() && (current_time - last_hard_loss_time_) > TimeDelta::Seconds(15)) {
          RTC_LOG(LS_VERBOSE) << "L4S: Fast Recovery timed out (15s). Releasing memory.";
          pre_loss_target_ = DataRate::Zero();
      }
  }


  DecayAlpha(current_time);
}

void webrtc::PragueCapacityEstimator::OnTimeUpdate(Timestamp current_time, bool is_app_limited) {
  OnAckedUpdate(current_time, is_app_limited, DataRate::Zero(), TimeDelta::Zero());
}

webrtc::DataRate webrtc::PragueCapacityEstimator::GetCurrentEstimate() const {
  return congestion_based_estimate_;
}

void webrtc::PragueCapacityEstimator::SetCurrentEstimate(DataRate rate) {
  congestion_based_estimate_ = std::max(rate, min_target_rate_);
  congestion_based_estimate_ = std::max(congestion_based_estimate_, DataRate::KilobitsPerSec(20));
}

double webrtc::PragueCapacityEstimator::GetConfidence(Timestamp now) const {
  if (last_ecn_feedback_.IsInfinite()) return 0.3;  
  TimeDelta since_ecn_activity = now - last_ecn_feedback_;
  if (since_ecn_activity < TimeDelta::Seconds(60)) return 0.95; 
  return 0.4;  
}

void webrtc::PragueCapacityEstimator::SetProbeConstraint(DataRate probe_estimate, Timestamp now) {
  probe_constraint_ = probe_estimate;
  probe_constraint_time_ = now;
}

bool webrtc::PragueCapacityEstimator::HasFreshProbeCeiling(Timestamp now) const {
  if (probe_constraint_.IsZero() || probe_constraint_time_.IsInfinite()) return false;
  return (now - probe_constraint_time_) < TimeDelta::Seconds(5);
}

void webrtc::PragueCapacityEstimator::ClearProbeConstraint() {
  probe_constraint_ = DataRate::Zero();
  probe_constraint_time_ = Timestamp::MinusInfinity();
}

void webrtc::PragueCapacityEstimator::SetAdditiveHoldUntil(Timestamp hold_until) {
  additive_hold_until_ = hold_until;
}

void webrtc::PragueCapacityEstimator::ExitDiscoveryMode(const std::string& reason) {
  if (discovery_mode_active_) {
    discovery_mode_active_ = false;
    ClearProbeConstraint();  
    RTC_LOG(LS_INFO) << "Prague: Exiting discovery mode - " << reason;
  }
}

bool webrtc::PragueCapacityEstimator::HasConvergedWithProbe() const {
  if (probe_constraint_time_.IsInfinite() || probe_constraint_.IsZero()) return false;
  double rate_ratio = congestion_based_estimate_.bps() / static_cast<double>(probe_constraint_.bps());
  return rate_ratio >= 0.95;
}


// =============================================================================
// L4SNetworkController Implementation
// =============================================================================

webrtc::L4SNetworkController::L4SNetworkController(NetworkControllerConfig config, L4SControllerConfig l4s_config, test::MetricsLogger* metrics_logger)
    : env_(config.env), config_(l4s_config) {
  
  DataRate starting_rate = config.constraints.starting_rate.value_or(DataRate::KilobitsPerSec(300));
  DataRate min_rate = config.constraints.min_data_rate.value_or(DataRate::KilobitsPerSec(30));
  DataRate max_rate = config.constraints.max_data_rate.value_or(DataRate::KilobitsPerSec(100000));
  
  prague_estimator_ = std::make_unique<PragueCapacityEstimator>(starting_rate, min_rate, max_rate);

  starting_rate_ = config.constraints.starting_rate;
  min_target_rate_ = config.constraints.min_data_rate;
  max_target_rate_ = config.constraints.max_data_rate;
  target_rate_ = starting_rate;

  InitializeBandwidthEstimators();
  
  if (config_.enable_metrics_collection) {
    using webrtc::test::GetGlobalMetricsLogger;
    test::MetricsLogger* logger_to_use = metrics_logger;
    if (!logger_to_use) {
      logger_to_use = GetGlobalMetricsLogger();
    }
    metrics_collector_ = std::make_unique<L4SMetricsCollector>(logger_to_use, config_.test_case_name);
  }
}

webrtc::L4SNetworkController::~L4SNetworkController() {
  if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->ExportToJsonFile("l4s_test_c6.json");
  }
}

void webrtc::L4SNetworkController::InitializeBandwidthEstimators() {
  if (config_.enable_probing) {
    probe_controller_ = std::make_unique<ProbeController>(&env_.field_trials(), &env_.event_log());
    probe_bitrate_estimator_ = std::make_unique<ProbeBitrateEstimator>(&env_.event_log());
  }
  if (config_.enable_acked_estimation) {
    acked_estimator_ = std::make_unique<AcknowledgedBitrateEstimator>(&env_.field_trials());
  }
  if (config_.enable_alr_detection) {
    alr_detector_ = std::make_unique<AlrDetector>(&env_.field_trials());
  }
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnNetworkAvailability(NetworkAvailability msg) {
  NetworkControlUpdate update;
  if (probe_controller_) {
    auto avail_probes = probe_controller_->OnNetworkAvailability(msg);
    Timestamp now = Timestamp::Millis(env_.clock().TimeInMilliseconds());
    for (const auto& probe : avail_probes) {
      TimeDelta since_last_probe = last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity() : (now - last_probe_time_);
      if (since_last_probe >= config_.probe_interval) {
        update.probe_cluster_configs.push_back(probe);
        last_probe_time_ = now;
        StartProbeHold(now);
      }
    }
  }
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnNetworkRouteChange(NetworkRouteChange msg) {
  NetworkControlUpdate update;
  ecn_supported_ = false;
  last_congestion_signal_ = Timestamp::MinusInfinity();
  
  if (msg.constraints.starting_rate) {
    starting_rate_ = msg.constraints.starting_rate;
    target_rate_ = starting_rate_;
  }
  min_target_rate_ = msg.constraints.min_data_rate;
  max_target_rate_ = msg.constraints.max_data_rate;

  base_rtt_ = TimeDelta::PlusInfinity(); 
  DataRate reset_starting_rate = starting_rate_.value_or(DataRate::KilobitsPerSec(300));
  DataRate reset_min_rate = min_target_rate_.value_or(DataRate::KilobitsPerSec(30));
  DataRate reset_max_rate = max_target_rate_.value_or(DataRate::KilobitsPerSec(100000));
  prague_estimator_ = std::make_unique<PragueCapacityEstimator>(reset_starting_rate, reset_min_rate, reset_max_rate);

  recovery_mode_active_ = false;
  recovery_probe_bootstrapped_ = false;
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

  historical_capacity_window_.clear();
  historical_max_capacity_ = DataRate::Zero();
  recent_probes_window_.clear();

  throughput_window_.clear();
  last_actual_bitrate_ = DataRate::Zero();
  
  last_acked_bitrate_.reset();
  last_loss_fraction_ = 0.0;
  last_packets_lost_ = 0;
  last_state_snapshot_log_ = Timestamp::MinusInfinity();

  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnProcessInterval(ProcessInterval msg) {
  NetworkControlUpdate update;
  LogPeriodicMetrics(msg.at_time);

  if (probe_controller_) {
    if (!initial_probes_sent_) {
      initial_probes_sent_ = true;
      DataRate clamped_min = min_target_rate_.value_or(DataRate::KilobitsPerSec(30));
      DataRate clamped_start = starting_rate_.value_or(DataRate::KilobitsPerSec(300));
      DataRate clamped_max = max_target_rate_.value_or(DataRate::KilobitsPerSec(100000));
      if (!clamped_max.IsFinite()) clamped_max = DataRate::KilobitsPerSec(100000); 
      clamped_start = std::max(clamped_min, clamped_start);
      clamped_max   = std::max(clamped_start, clamped_max);

      auto init_probes = probe_controller_->SetBitrates(clamped_min, clamped_start, clamped_max, msg.at_time);
      update.probe_cluster_configs.insert(update.probe_cluster_configs.end(), init_probes.begin(), init_probes.end());
      if (!init_probes.empty()) StartProbeHold(msg.at_time);
      probe_controller_->EnablePeriodicAlrProbing(true);
    }

    if (alr_detector_) {
      probe_controller_->SetAlrStartTimeMs(alr_detector_->GetApplicationLimitedRegionStartTime());
    }

    auto periodic_probes = probe_controller_->Process(msg.at_time);
    for (const auto& probe : periodic_probes) {
      TimeDelta since_last_probe = last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity() : (msg.at_time - last_probe_time_);
      if (since_last_probe >= config_.probe_interval) {
        update.probe_cluster_configs.push_back(probe);
        last_probe_time_ = msg.at_time;
        StartProbeHold(msg.at_time);
      }
    }
  }

  HandlePeriodicProbing(msg.at_time, &update);
  DataRate fused_rate = FuseBandwidthEstimates(msg.at_time);
  target_rate_ = fused_rate;

  MaybeTriggerOnNetworkChanged(&update, msg.at_time);
  LogStateSnapshot(msg.at_time);

  if (fused_rate.IsFinite() && last_rtt_.IsFinite() && !last_rtt_.IsZero()) {
      TimeDelta unloaded_rtt = base_rtt_.IsFinite() ? base_rtt_ : last_rtt_;
      TimeDelta effective_rtt = std::max(unloaded_rtt, TimeDelta::Millis(25));
      DataSize bdp = fused_rate * effective_rtt;
      update.congestion_window = std::max(bdp + DataSize::Bytes(1500 * 5), DataSize::Bytes(1500 * 10));
  } else {
      update.congestion_window = std::nullopt;
  }
  
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnRemoteBitrateReport(RemoteBitrateReport msg) {
  return NetworkControlUpdate();
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnRoundTripTimeUpdate(RoundTripTimeUpdate msg) {
  NetworkControlUpdate update;
  if (msg.smoothed) return update;

  if (msg.round_trip_time.IsFinite() && !msg.round_trip_time.IsZero()) {
    if (last_smoothed_rtt_.IsFinite() && !last_smoothed_rtt_.IsZero()) {
      last_smoothed_rtt_ = (last_smoothed_rtt_ * 0.85) + (msg.round_trip_time * 0.15);
    } else {
      last_smoothed_rtt_ = msg.round_trip_time;
    }
    
    last_rtt_ = msg.round_trip_time;




    // RTC_LOG(LS_INFO)
    // << "raw=" << msg.round_trip_time.ms()
    // << " last_rtt=" << last_rtt_.ms()
    // << " smooth=" << last_smoothed_rtt_.ms()
    // << " estimated=" << last_estimated_round_trip_time_.ms();

      // Log RTT metrics.
    if (metrics_enabled_ && metrics_collector_) {
    metrics_collector_->LogDelayMetrics(
        Timestamp::Millis(env_.clock().TimeInMilliseconds()),
        last_rtt_, last_rtt_ / 2, TimeDelta::Zero());
    }

    TimeDelta raw_safe_floor = std::max(last_rtt_, TimeDelta::Millis(20));
    if (base_rtt_.IsInfinite() || raw_safe_floor < base_rtt_) {
        base_rtt_ = raw_safe_floor;
    }
    
    // prague_estimator_->UpdateFromRtt(last_rtt_); 

    // =========================================================
    // --- THE FIX: USE SMOOTHED RTT TO IGNORE TWCC BATCHING ---
    // =========================================================
    prague_estimator_->UpdateFromRtt(last_smoothed_rtt_);
    
    last_estimated_round_trip_time_ = std::max(last_smoothed_rtt_, TimeDelta::Millis(20));


  }
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnSentPacket(SentPacket msg) {
  NetworkControlUpdate update;
  if (alr_detector_) {
    alr_detector_->OnBytesSent(msg.size.bytes(), msg.send_time.ms());
    if (acked_estimator_) {
      acked_estimator_->SetAlr(alr_detector_->GetApplicationLimitedRegionStartTime().has_value());
    }
  }

  constexpr TimeDelta kSendWindow = TimeDelta::Millis(500); 
  if (msg.send_time.IsFinite()) {
    send_rate_window_.emplace_back(msg.send_time, msg.size.bytes());
  }

  while (!send_rate_window_.empty() && (msg.send_time - send_rate_window_.front().first) > kSendWindow) {
    send_rate_window_.pop_front();
  }

  if (send_rate_window_.size() > 1) {
    Timestamp window_start = send_rate_window_.front().first;
    Timestamp window_end = send_rate_window_.back().first;
    TimeDelta window_interval = window_end - window_start;

    if (window_interval >= TimeDelta::Millis(100)) {
      int64_t window_bytes = 0;
      for (const auto& entry : send_rate_window_) {
        window_bytes += entry.second;
      }
      last_send_rate_ = DataRate::BitsPerSec(static_cast<int64_t>((window_bytes * 8) / window_interval.seconds<double>()));
    }
  }

  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnReceivedPacket(ReceivedPacket msg) {
  return NetworkControlUpdate();
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnStreamsConfig(StreamsConfig msg) {
  NetworkControlUpdate update;
  if (probe_controller_) {
    if (msg.requests_alr_probing) probe_controller_->EnablePeriodicAlrProbing(*msg.requests_alr_probing);
    if (msg.max_total_allocated_bitrate) {
      auto probes = probe_controller_->OnMaxTotalAllocatedBitrate(*msg.max_total_allocated_bitrate, msg.at_time);
      update.probe_cluster_configs.insert(update.probe_cluster_configs.end(), probes.begin(), probes.end());
    }
  }
  if (msg.max_padding_rate) max_padding_rate_ = *msg.max_padding_rate;
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnTargetRateConstraints(TargetRateConstraints msg) {
  min_target_rate_ = msg.min_data_rate;
  max_target_rate_ = msg.max_data_rate;
  return NetworkControlUpdate();
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnTransportLossReport(TransportLossReport msg) {
  NetworkControlUpdate update;
  
  if (msg.packets_lost_delta > 0) {
    DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
    
    // --- INTEGRATION: Pass BOTH lost and received packets to calculate Loss Ratio ---
    int total_packets = msg.packets_lost_delta + msg.packets_received_delta;
    prague_estimator_->OnPacketLoss(current_rate, msg.receive_time, msg.packets_lost_delta, total_packets,historical_max_capacity_);
  }
  
  int total_packets = msg.packets_lost_delta + msg.packets_received_delta;
  last_loss_fraction_ = (total_packets > 0) ? static_cast<double>(msg.packets_lost_delta) / total_packets : 0.0;
  last_packets_lost_ = static_cast<int>(msg.packets_lost_delta);
  
  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnTransportPacketsFeedback(TransportPacketsFeedback msg) {
  NetworkControlUpdate update;
  UpdateThroughputWindow(msg);
  UpdateAllBandwidthEstimators(msg);

  TimeDelta rtt_bloat = (last_rtt_.IsFinite() && base_rtt_.IsFinite()) ? (last_rtt_ - base_rtt_) : TimeDelta::Zero();

  if (prague_estimator_ && !msg.packet_feedbacks.empty()) {
    prague_estimator_->OnAckedUpdate(msg.feedback_time, IsApplicationLimited(), last_actual_bitrate_, rtt_bloat);
  }

  HandlePeriodicProbing(msg.feedback_time, &update);
  DataRate fused_rate = FuseBandwidthEstimates(msg.feedback_time);
  target_rate_ = fused_rate;

  MaybeTriggerOnNetworkChanged(&update, msg.feedback_time);
  LogStateSnapshot(msg.feedback_time);

  if (fused_rate.IsFinite() && last_rtt_.IsFinite() && !last_rtt_.IsZero()) {
      TimeDelta unloaded_rtt = base_rtt_.IsFinite() ? base_rtt_ : last_rtt_;
      TimeDelta effective_rtt = std::max(unloaded_rtt, TimeDelta::Millis(25));
      DataSize bdp = fused_rate * effective_rtt;
      update.congestion_window = std::max(bdp + DataSize::Bytes(1500 * 5), DataSize::Bytes(1500 * 10));
  } else {
      update.congestion_window = std::nullopt;
  }

  // --- NEW: THE LIGHTWEIGHT PUSHBACK CONTROLLER ---
  pushback_target_rate_ = fused_rate; 
  if (update.congestion_window && msg.data_in_flight.bytes() > 0) {
      if (msg.data_in_flight > *update.congestion_window) {
          // The physical wire is over the limit! Calculate the overflow ratio.
          double pushback_ratio = static_cast<double>(update.congestion_window->bytes()) / 
                                  static_cast<double>(msg.data_in_flight.bytes());
          
          // Cap the drop at 50% to prevent total video collapse
          pushback_ratio = std::max(0.50, pushback_ratio);
          pushback_target_rate_ = fused_rate * pushback_ratio;
      }
  }


  return update;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::OnNetworkStateEstimate(NetworkStateEstimate msg) {
  return NetworkControlUpdate();
}

void webrtc::L4SNetworkController::UpdateAllBandwidthEstimators(const TransportPacketsFeedback& feedback) {
  UpdateAlrDetector(feedback);

  std::vector<PacketResult> received_feedback = feedback.SortedByReceiveTime();
  // if (!received_feedback.empty()) {
  //   const Timestamp max_recv_time = received_feedback.back().receive_time;
  //   TimeDelta feedback_min_rtt = TimeDelta::PlusInfinity();
  //   for (const auto& packet_feedback : received_feedback) {
  //     TimeDelta pending_time = max_recv_time - packet_feedback.receive_time;
  //     TimeDelta rtt = feedback.feedback_time - packet_feedback.sent_packet.send_time - pending_time;
  //     feedback_min_rtt = std::min(feedback_min_rtt, rtt);
  //   }
    
  //   if (feedback_min_rtt.IsFinite() && !feedback_min_rtt.IsZero()) {
  //     // last_rtt_ = feedback_min_rtt;
  //     TimeDelta safe_rtt = std::max(last_rtt_, TimeDelta::Millis(20));
  //     if (base_rtt_.IsInfinite() || safe_rtt < base_rtt_) {
  //         base_rtt_ = safe_rtt;
  //     }
  //     // prague_estimator_->UpdateFromRtt(safe_rtt);
  //     last_estimated_round_trip_time_ = safe_rtt;
  //   }
  // }
  
  if (acked_estimator_) UpdateAckedBitrateEstimator(feedback);
  if (probe_controller_) ProcessRealProbeResults(feedback);
  
  DataRate base_fused_rate = prague_estimator_->GetCurrentEstimate();
  ProcessEcnFeedback(feedback, base_fused_rate);
}

double webrtc::L4SNetworkController::CalculateEcnYieldRatio(double raw_ce_ratio, double starvation_ratio, TimeDelta rtt_bloat) const {
  if (raw_ce_ratio <= 0.0) return 0.0;

  if (starvation_ratio < 0.25) {
    double dampened_ratio = raw_ce_ratio * 0.1;
    return dampened_ratio;
  } 
  
  if (rtt_bloat >= TimeDelta::Millis(15)) {
    double dampened_ratio = raw_ce_ratio * 0.25;
    return dampened_ratio;
  }

  return raw_ce_ratio;
}

void webrtc::L4SNetworkController::EnforceHistoricalSafetyFloor() {
  if (!prague_estimator_ || historical_max_capacity_.IsZero()) return;
  DataRate dynamic_floor = historical_max_capacity_ * 0.15;
  dynamic_floor = std::max(dynamic_floor, DataRate::KilobitsPerSec(500));
  
  if (prague_estimator_->GetCurrentEstimate() < dynamic_floor) {
    prague_estimator_->SetCurrentEstimate(dynamic_floor);
  }
}

void webrtc::L4SNetworkController::ProcessEcnFeedback(const TransportPacketsFeedback& feedback, DataRate current_fused_rate) {
  if (feedback.packet_feedbacks.empty()) return;

  TimeDelta window_duration = last_rtt_.IsFinite() && !last_rtt_.IsZero() ? last_rtt_ : TimeDelta::Millis(100);
  int batch_ect_count = 0;
  int batch_ce_count = 0;
  bool probe_caused_congestion = false;

  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.ecn == EcnMarking::kEct0 || packet.ecn == EcnMarking::kEct1) batch_ect_count++;
    if (packet.ecn == EcnMarking::kCe) {
      batch_ce_count++;
      last_congestion_signal_ = feedback.feedback_time;
      if (packet.sent_packet.pacing_info.probe_cluster_id != PacedPacketInfo::kNotAProbe) {
        probe_caused_congestion = true;
      }
      prague_estimator_->ClearProbeConstraint(); 
    }
  }

  // RTC_LOG(LS_INFO) << "L4S: ECN feedback received. Batch ECT count: " << batch_ect_count
  //                   << ", Batch CE count: " << batch_ce_count
  //                   << ", probe caused congestion: " << probe_caused_congestion;

  if (batch_ect_count > 0 || batch_ce_count > 0) {
    ecn_supported_ = true;
    prague_estimator_->UpdateEcnActivity(feedback.feedback_time);
  }

  window_ce_count_ += batch_ce_count;
  window_ect_count_ += batch_ect_count;

  // --- THE SWEEP TRIPWIRE ---
  if (batch_ce_count > 0 && sweep_mode_active_) {
      sweep_mode_active_ = false;
      
      // We found the exact limit of the physical queue. 
      // Anchor the video target to the physical rate that triggered the mark.
      // DataRate ground_truth = std::max(last_actual_bitrate_, last_send_rate_);
      DataRate ground_truth = last_send_rate_;
      
      // Back off exactly 10% from the tripwire to instantly drain the 36-packet queue.
      ground_truth = ground_truth * 0.90;
      
      if (prague_estimator_) {
          prague_estimator_->SetCurrentEstimate(ground_truth);
          prague_estimator_->ExitDiscoveryMode("Paced Sweep tripped by CE mark");
      }
      
      RTC_LOG(LS_WARNING) << "L4S: [Paced Sweep] TRIPWIRE HIT! CE mark received. "
                          << "Anchoring target safely to " << ground_truth.kbps() << " kbps.";
      
      // Absorb the CE mark so the standard continuous Prague engine 
      // doesn't double-punish the connection on the next line.
      window_ce_count_ = 0;
      window_ect_count_ = 0;
      window_start_time_ = feedback.feedback_time;
      last_congestion_signal_ = feedback.feedback_time;
      return; 
  }


  double rtt_s = last_rtt_.IsFinite() ? last_rtt_.seconds<double>() : 0.1;
  double rate_bps = current_fused_rate.bps();
  double pkt_size_bits = 1400.0 * 8.0;

  int dynamic_threshold = std::clamp(static_cast<int>((rate_bps * rtt_s) / pkt_size_bits), 10, 200);
  bool window_expired = (window_start_time_.IsInfinite() || (feedback.feedback_time - window_start_time_) >= window_duration);
  int window_total = window_ce_count_ + window_ect_count_;
  bool immediate_ce_trigger = (batch_ce_count > 0);

  if (window_expired || window_total >= dynamic_threshold || immediate_ce_trigger) {
    int safe_denominator = std::max(window_total, dynamic_threshold);
    double ce_ratio = (safe_denominator > 0) ? static_cast<double>(window_ce_count_) / safe_denominator : 0.0;

    if (window_total >= 3 || immediate_ce_trigger) {
      if (window_ce_count_ > 0 && probe_caused_congestion) {
        prague_estimator_->SetAdditiveHoldUntil(feedback.feedback_time + (last_rtt_ * 2));
      } else {
        DataRate current_target = prague_estimator_->GetCurrentEstimate();
        DataRate physical_traffic = std::min(current_target, last_send_rate_);
        DataRate base_for_cut = current_target;
        
        if (physical_traffic > DataRate::Zero() && physical_traffic < current_target) {
            base_for_cut = physical_traffic;
        }

        TimeDelta rtt_bloat = (last_rtt_.IsFinite() && base_rtt_.IsFinite()) ? (last_rtt_ - base_rtt_) : TimeDelta::PlusInfinity();
        double starvation_ratio = 1.0;
        if (historical_max_capacity_ > DataRate::Zero()) {
            starvation_ratio = current_fused_rate.bps() / static_cast<double>(historical_max_capacity_.bps());
        }

        double effective_ce_ratio = CalculateEcnYieldRatio(ce_ratio, starvation_ratio, rtt_bloat);

        prague_estimator_->UpdateFromCongestionSignal(base_for_cut, effective_ce_ratio, window_total, feedback.feedback_time, historical_max_capacity_);
      }

      EnforceHistoricalSafetyFloor();
    }

    window_ce_count_ = 0;
    window_ect_count_ = 0;
    window_start_time_ = feedback.feedback_time;
  }

  HandleRecoveryDetection(batch_ce_count, feedback.feedback_time);
}

void webrtc::L4SNetworkController::UpdateAckedBitrateEstimator(const TransportPacketsFeedback& feedback) {
  if (feedback.packet_feedbacks.empty() || !acked_estimator_) return;
  acked_estimator_->IncomingPacketFeedbackVector(feedback.SortedByReceiveTime());
  std::optional<DataRate> acked_bitrate = acked_estimator_->bitrate();
  if (acked_bitrate.has_value()) {
      last_acked_bitrate_ = acked_bitrate;
  }
}

void webrtc::L4SNetworkController::ProcessRealProbeResults(const TransportPacketsFeedback& feedback) {
  if (!probe_bitrate_estimator_) return;

  bool probe_packet_has_ce = false;
  for (const auto& packet_feedback : feedback.SortedByReceiveTime()) {
    if (packet_feedback.sent_packet.pacing_info.probe_cluster_id != PacedPacketInfo::kNotAProbe) {
      probe_bitrate_estimator_->HandleProbeAndEstimateBitrate(packet_feedback);
      if (packet_feedback.ecn == EcnMarking::kCe) probe_packet_has_ce = true;
    }
  }

  std::optional<DataRate> measured_probe_rate = GetLastProbeResult();
  if (measured_probe_rate) {
    Timestamp now = feedback.feedback_time;
    recent_probes_window_.emplace_back(now, measured_probe_rate.value());
    
    while (!recent_probes_window_.empty() && (now - recent_probes_window_.front().first) > TimeDelta::Seconds(3)) {
      recent_probes_window_.pop_front();
    }
    
    DataRate effective_probe_rate = DataRate::Zero();
    for (const auto& entry : recent_probes_window_) {
        effective_probe_rate = std::max(effective_probe_rate, entry.second);
    }

    if (last_actual_bitrate_.IsZero() || effective_probe_rate >= last_actual_bitrate_) {
      bool recent_congestion = HasRecentCongestionSignals(now);
      bool is_probe_valid = IsProbeDataValid(now);
      bool block_probe_uplift = probe_packet_has_ce || recent_congestion || !is_probe_valid;

      if (block_probe_uplift && prague_estimator_) {
          prague_estimator_->ClearProbeConstraint();
      }

      if (prague_estimator_ && !block_probe_uplift) {
        DataRate current_prague = prague_estimator_->GetCurrentEstimate();
        if (effective_probe_rate > (current_prague * 1.05)) {
          DataRate max_uplift = current_prague * 1.5;
          DataRate probe_ceiling ;

          if (prague_estimator_->IsDiscoveryModeActive() && probe_trust_counter_ < 5) {
            // --- YOUR THEORY: Blind Trust for the first 5 probes ---
            probe_trust_counter_++;

            // --- INITIATE THE PACED SWEEP ---
            sweep_mode_active_ = true;
            sweep_target_rate_ = effective_probe_rate * 0.95; // Chase 95% of the probe
            sweep_current_padding_rate_ = std::max(current_prague, last_send_rate_);
            sweep_last_update_time_ = now;
            sweep_reached_target_time_ = Timestamp::MinusInfinity();
            
            RTC_LOG(LS_INFO) << "L4S: [Paced Sweep] Initiating sweep towards " 
                             << sweep_target_rate_.kbps() << " kbps.";
                             
            // DO NOT jump the estimator yet. Let the Pacer do the work.
            probe_ceiling = sweep_target_rate_;


            // probe_ceiling = effective_probe_rate; 
            
            // RTC_LOG(LS_INFO) << "L4S: [Blind Trust Phase] Probe " << probe_trust_counter_ 
            //                  << " valid at " << probe_ceiling.kbps() 
            //                  << " kbps. Instant jump executed.";

            // // Instantly jump the actual target rate to the probe result
            // prague_estimator_->SetCurrentEstimate(probe_ceiling);
            // prague_estimator_->SetProbeConstraint(probe_ceiling, now);
            
            // Note: We deliberately do NOT exit discovery mode here. We wait 
            // for the actual DualPI2 CE marks to force us out.
            
          } else {
            // --- Normal Steady-State Bounded Behavior ---
            probe_ceiling = std::min(effective_probe_rate * 0.95, max_uplift);
            prague_estimator_->SetProbeConstraint(probe_ceiling, now);
          }

          
          probe_reject_streak_ = 0;
          next_probe_allowed_at_ = Timestamp::MinusInfinity();
          probe_rate_ceiling_ = probe_ceiling;
          // prague_estimator_->SetProbeConstraint(probe_ceiling, now);
        }
      }
    } else {
      probe_reject_streak_ = std::min(probe_reject_streak_ + 1, 4);
      int64_t backoff_seconds = std::min<int64_t>(12, 3 * (1LL << (probe_reject_streak_ - 1)));
      next_probe_allowed_at_ = feedback.feedback_time + TimeDelta::Seconds(backoff_seconds);
    }
  }
}

std::optional<DataRate> webrtc::L4SNetworkController::GetLastProbeResult() {
  if (!probe_bitrate_estimator_) return std::nullopt;
  return probe_bitrate_estimator_->FetchAndResetLastEstimatedBitrate();
}

void webrtc::L4SNetworkController::HandlePeriodicProbing(Timestamp now, NetworkControlUpdate* update) {
  if (!config_.enable_probing || !probe_controller_) return;
  
  TimeDelta since_last_probe = last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity() : (now - last_probe_time_);
  TimeDelta rtt_bloat = (last_rtt_.IsFinite() && base_rtt_.IsFinite()) ? (last_rtt_ - base_rtt_) : TimeDelta::PlusInfinity();
  
  DataRate current_target = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  DataRate actual_rate = last_actual_bitrate_;

  if (!next_probe_allowed_at_.IsInfinite() && now < next_probe_allowed_at_) return;
  if (!probe_hold_until_.IsInfinite() && now < probe_hold_until_) return;
  if (update && !update->probe_cluster_configs.empty()) return; 

  if (prague_estimator_ && prague_estimator_->HasFreshProbeCeiling(now)) {
    if (probe_rate_ceiling_ > (current_target * 1.25)) return; 
  }
  
  if (HasRecentCongestionSignals(now) || last_loss_fraction_ > 0.02) return; 

  if (prague_estimator_ && prague_estimator_->IsDiscoveryModeActive()) {
      bool has_ceiling = prague_estimator_->HasFreshProbeCeiling(now);
      TimeDelta discovery_interval = has_ceiling ? TimeDelta::Seconds(5) : TimeDelta::Seconds(2);
      bool queue_is_safe = rtt_bloat < TimeDelta::Millis(30);
      
      if (since_last_probe >= discovery_interval && queue_is_safe) {
          InitiateStatefulProbe(now, update, 1.50, "Discovery");
          last_probe_time_ = now;
      }
      return; 
  }

  bool demand_from_actual = !actual_rate.IsZero() && actual_rate > (current_target * 0.85);
  bool delivery_supports_target = !actual_rate.IsZero() && actual_rate > (current_target * 0.75);
  if (demand_from_actual && !IsApplicationLimited()) {
      if (demand_high_since_.IsInfinite()) demand_high_since_ = now;
  } else {
      demand_high_since_ = Timestamp::MinusInfinity();
  }
  bool demand_is_sustained = !demand_high_since_.IsInfinite() && (now - demand_high_since_) >= TimeDelta::Seconds(1);

  if (recovery_mode_active_) {
      TimeDelta recovery_interval = std::max(GetRttScaledInterval(), TimeDelta::Seconds(4));
      bool queue_is_empty = rtt_bloat < TimeDelta::Millis(15);
      
      if (since_last_probe >= recovery_interval && demand_is_sustained && delivery_supports_target && queue_is_empty) {
          InitiateStatefulProbe(now, update, config_.recovery_probe_multiplier, "Recovery");
          last_probe_time_ = now;
      }
      return; 
  }

  TimeDelta stable_interval = config_.probe_interval; 
  bool queue_is_empty = rtt_bloat < TimeDelta::Millis(30);

  if (since_last_probe >= stable_interval && demand_is_sustained && delivery_supports_target && queue_is_empty) {
      if (ShouldProbeNow(now)) { 
          InitiateStatefulProbe(now, update, 1.05, "Stable Micro");
          last_probe_time_ = now;
      }
  }
}

TimeDelta webrtc::L4SNetworkController::GetRttScaledInterval() const {
  TimeDelta effective_rtt = last_rtt_.IsFinite() && !last_rtt_.IsZero()
          ? std::clamp(last_rtt_, TimeDelta::Millis(50), TimeDelta::Millis(300))
          : TimeDelta::Millis(100);
  TimeDelta rtt_scaled = effective_rtt * config_.recovery_probe_rtt_factor;
  return std::max(config_.min_rtt_scaled_interval, rtt_scaled);
}

void webrtc::L4SNetworkController::StartProbeHold(Timestamp now) {
  TimeDelta effective_rtt = last_rtt_.IsFinite() && !last_rtt_.IsZero()
          ? std::clamp(last_rtt_, TimeDelta::Millis(50), TimeDelta::Millis(300))
          : TimeDelta::Millis(100);
  TimeDelta hold = std::max(config_.min_rtt_scaled_interval, effective_rtt * config_.probe_hold_rtt_factor);
  probe_hold_until_ = now + hold;
  if (prague_estimator_) prague_estimator_->SetAdditiveHoldUntil(probe_hold_until_);
}

bool webrtc::L4SNetworkController::ShouldProbeNow(Timestamp now) const {
  if (last_rtt_.IsFinite() && last_rtt_ > TimeDelta::Millis(250)) return false;

  if (last_rtt_.IsFinite() && base_rtt_.IsFinite()) {
      TimeDelta bloat = last_rtt_ - base_rtt_;
      TimeDelta allowed_bloat = TimeDelta::Millis(30); 
      DataRate current_target = target_rate_.value_or(DataRate::Zero());
      
      if (historical_max_capacity_ > DataRate::Zero() && current_target < (historical_max_capacity_ * 0.35)) {
          allowed_bloat = TimeDelta::Millis(50); 
      }
      if (bloat > allowed_bloat) return false;
  } else {
      return false; 
  }

  DataRate current_estimate = target_rate_.value_or(DataRate::Zero());
  if (current_estimate < config_.periodic_probe_min_rate) return false;
  if (HasRecentCongestionSignals(now)) return false;
  
  double ecn_confidence = prague_estimator_->GetConfidence(now);
  bool is_discovery_mode = prague_estimator_->IsDiscoveryModeActive();
  double confidence_threshold = is_discovery_mode ? config_.discovery_probe_block_confidence : config_.steady_probe_block_confidence;
                                    
  if (IsEcnFeedbackFresh(now) && ecn_confidence > confidence_threshold) return false;
  if (last_loss_fraction_ > 0.02) return false;
  
  return true;
}

std::optional<webrtc::ProbeClusterConfig> webrtc::L4SNetworkController::CreateCustomProbe(Timestamp now, DataRate target_rate) {
  if (target_rate <= DataRate::Zero()) return std::nullopt;
  static int32_t next_custom_probe_id = 10000; 

  ProbeClusterConfig custom_probe;
  custom_probe.at_time = now;
  custom_probe.target_data_rate = target_rate;
  custom_probe.target_duration = TimeDelta::Millis(10);
  custom_probe.target_probe_count = 4;
  custom_probe.id = next_custom_probe_id++;
  return custom_probe;
}

bool webrtc::L4SNetworkController::EncoderNeedsMoreHeadroom(double multiplier) const {
  if (!IsApplicationLimited()) return true;
  DataRate current_estimate = target_rate_.value_or(DataRate::Zero());
  DataRate actual_throughput = this->last_actual_bitrate_;
  if (actual_throughput <= DataRate::Zero()) return true; 
  
  DataRate relative_cap = actual_throughput * multiplier;
  DataRate absolute_cap = actual_throughput + DataRate::KilobitsPerSec(1500); 
  DataRate allowed_target = std::max(relative_cap, absolute_cap);
  
  if (current_estimate > allowed_target) return false; 
  return true;
}

webrtc::DataRate webrtc::L4SNetworkController::FuseBandwidthEstimates(Timestamp now) {
  if (!prague_estimator_) return target_rate_.value_or(DataRate::KilobitsPerSec(300));

  if (prague_estimator_->IsDiscoveryModeActive() && ShouldExitDiscoveryMode(now)) {
    prague_estimator_->ExitDiscoveryMode("probe-Prague convergence or fallback threshold");
  }

  DataRate fused_rate = prague_estimator_->GetCurrentEstimate();
  if (min_target_rate_ && fused_rate < *min_target_rate_) fused_rate = *min_target_rate_;
  fused_rate = std::max(fused_rate, DataRate::KilobitsPerSec(20));
  if (max_target_rate_ && fused_rate > *max_target_rate_) fused_rate = *max_target_rate_;
  
  return fused_rate;
}

webrtc::NetworkControlUpdate webrtc::L4SNetworkController::CreateRateUpdate(Timestamp at_time) const {
  NetworkControlUpdate update;
  if (!at_time.IsFinite()) at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());
  
  DataRate current_rate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  if (!current_rate.IsFinite() || current_rate.bps() <= 0) current_rate = DataRate::KilobitsPerSec(300);
  
  update.target_rate = TargetTransferRate();
  update.target_rate->at_time = at_time;
  update.target_rate->network_estimate.at_time = at_time;
  update.target_rate->network_estimate.bandwidth = current_rate;
  update.target_rate->network_estimate.loss_rate_ratio = static_cast<float>(last_loss_fraction_);
  update.target_rate->network_estimate.round_trip_time = last_estimated_round_trip_time_;
  update.target_rate->network_estimate.bwe_period = TimeDelta::Millis(500);

  // Give the VIDEO ENCODER the pushed-back rate so it stops generating heavy frames
  DataRate encoder_target = pushback_target_rate_.IsZero() ? current_rate : pushback_target_rate_;
  update.target_rate->target_rate = encoder_target;
  // update.target_rate->target_rate = current_rate;
  
  // =========================================================
  // 1. THE PACING FACTOR (The Burst Absorber)
  // =========================================================
  // GCC uses 1.5x to 2.5x to quickly drain video frames from the software queue.
  // For L4S, massive bursts hit the shallow 1ms queue hard, so we use a safe 1.5x 
  // to clear frames quickly without triggering excessive CE marks.
  double pacing_factor = 1.5;
  if (prague_estimator_ && prague_estimator_->IsDiscoveryModeActive()) {
      pacing_factor = 2.5; // Allow higher bursting during startup probing
  }
  DataRate pacing_rate = current_rate * pacing_factor;

  // =========================================================
  // 2. THE PADDING RATE (The CBR Smoother)
  // =========================================================
  // Keep the link warm to maintain a smooth RTT and Send Rate. 
  // max_padding_rate_ is provided by WebRTC's BitrateAllocator.
  // We pad up to the max requested, but NEVER pad higher than the L4S safe target.
  // =========================================================
  // 2. THE PADDING RATE (The CBR Smoother)
  // =========================================================
  DataRate padding_rate = max_padding_rate_.value_or(DataRate::Zero());
  padding_rate = std::min(padding_rate, current_rate);

  // --- READ THE PACED SWEEP STATE (NO MATH HERE) ---
  if (sweep_mode_active_) {
      padding_rate = sweep_current_padding_rate_;
      pacing_rate = std::max(pacing_rate, sweep_current_padding_rate_ * 1.05);
  }

  // =========================================================
  // 3. CONFIGURE THE PACER
  // =========================================================
  update.pacer_config = PacerConfig();
  update.pacer_config->at_time = at_time;
  update.pacer_config->time_window = TimeDelta::Seconds(1);
  
  update.pacer_config->data_window = pacing_rate * update.pacer_config->time_window;
  update.pacer_config->pad_window = padding_rate * update.pacer_config->time_window;
  
  return update;
}

void webrtc::L4SNetworkController::MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update, Timestamp at_time) {
  if (!at_time.IsFinite()) at_time = Timestamp::Millis(env_.clock().TimeInMilliseconds());

  // 1. Update the math first!
  UpdatePacedSweepState(at_time);
  
  NetworkControlUpdate rate_update = CreateRateUpdate(at_time);
  
  if (rate_update.pacer_config) update->pacer_config = rate_update.pacer_config;
  
  if (rate_update.target_rate) {
    DataRate target_bitrate = rate_update.target_rate->target_rate;

    // THE HYSTERESIS GATE: Only wake up the heavy Video Encoder if the rate changed > 2%
    bool rate_changed_significantly = 
        !last_emitted_target_rate_.has_value() ||
        (std::abs(target_bitrate.bps() - last_emitted_target_rate_->bps()) > 
         (last_emitted_target_rate_->bps() * 0.02)); 

    if (rate_changed_significantly) {
      update->target_rate = rate_update.target_rate;
      last_emitted_target_rate_ = target_bitrate;

      if (alr_detector_) alr_detector_->SetEstimatedBitrate(target_bitrate.bps());

      if (probe_controller_) {
        bool is_first_report = last_reported_bitrate_to_probe_controller_.IsZero();
        bool changed_for_probe =
            is_first_report ||
            (std::abs(static_cast<int64_t>(target_bitrate.bps()) - static_cast<int64_t>(last_reported_bitrate_to_probe_controller_.bps())) >
             static_cast<int64_t>(0.05 * last_reported_bitrate_to_probe_controller_.bps()));
             
        if (changed_for_probe) {
          BandwidthLimitedCause cause = recovery_mode_active_ ? BandwidthLimitedCause::kLossLimitedBweIncreasing : BandwidthLimitedCause::kDelayBasedLimited;
          auto probes = probe_controller_->SetEstimatedBitrate(target_bitrate, cause, at_time);
          last_reported_bitrate_to_probe_controller_ = target_bitrate;
          if (!probes.empty()) {
            for (const auto& probe : probes) {
              if (!update->probe_cluster_configs.empty()) break;
              TimeDelta since_last_probe = last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity() : (at_time - last_probe_time_);
              if (since_last_probe >= config_.probe_interval) {
                update->probe_cluster_configs.push_back(probe);
                last_probe_time_ = at_time;
                StartProbeHold(at_time);
              }
            }
          }
        }
      }
    }
  }
}



bool webrtc::L4SNetworkController::CanEnterRecoveryState(Timestamp now) const {
  if (!prague_estimator_ || prague_estimator_->IsDiscoveryModeActive() || recovery_mode_active_) return false;
  if (last_congestion_signal_.IsInfinite()) return false;

  TimeDelta effective_rtt = last_rtt_.IsFinite() ? std::max(last_rtt_, TimeDelta::Millis(20)) : TimeDelta::Millis(100);
  return (now - last_congestion_signal_) >= (effective_rtt * kRecoveryCeQuietRttMultiplier);
}

void webrtc::L4SNetworkController::UpdatePacedSweepState(Timestamp at_time) {
  if (!sweep_mode_active_) return;

  TimeDelta elapsed = at_time - sweep_last_update_time_;
  sweep_last_update_time_ = at_time;
  
  if (elapsed > TimeDelta::Zero()) {
      double sweep_step = 5000000.0 * elapsed.seconds<double>();
      sweep_current_padding_rate_ += DataRate::BitsPerSec(sweep_step);
      sweep_current_padding_rate_ = std::min(sweep_current_padding_rate_, sweep_target_rate_);
  }

  // Check if we survived the entire sweep
  if (sweep_current_padding_rate_ >= sweep_target_rate_) {
      if (sweep_reached_target_time_.IsInfinite()) {
          sweep_reached_target_time_ = at_time;
      } else if (at_time - sweep_reached_target_time_ > TimeDelta::Millis(200)) {
          sweep_mode_active_ = false;
          if (prague_estimator_) {
              prague_estimator_->SetCurrentEstimate(sweep_target_rate_);
          }
          RTC_LOG(LS_INFO) << "L4S: [Paced Sweep] Success! Network is clear. Anchoring target at " 
                           << sweep_target_rate_.kbps() << " kbps.";
      }
  }
}

void webrtc::L4SNetworkController::LogStateSnapshot(Timestamp now) {
  if (!now.IsFinite()) now = Timestamp::Millis(env_.clock().TimeInMilliseconds());

  if (!last_state_snapshot_log_.IsInfinite() && (now - last_state_snapshot_log_) < kStateSnapshotLogInterval) return;
  last_state_snapshot_log_ = now;

  const bool discovery_active = prague_estimator_ && prague_estimator_->IsDiscoveryModeActive();
  
  TimeDelta cooldown_left = recovery_cooldown_until_.IsInfinite() || now >= recovery_cooldown_until_ ? TimeDelta::Zero() : (recovery_cooldown_until_ - now);

  RTC_LOG(LS_INFO)
      << "L4S: State snapshot"
      << " | discovery=" << discovery_active
      << " | recovery=" << recovery_mode_active_
      << " | alr=" << IsApplicationLimited()
      << " | cooldown_left_ms=" << cooldown_left.ms()
      << " | target_bps=" << target_rate_.value_or(DataRate::Zero()).bps()
      << " | send_rate_bps=" << last_send_rate_.bps()
      << " | actual_bps=" << last_actual_bitrate_.bps()
      << " | acked_bps=" << last_acked_bitrate_.value_or(DataRate::Zero()).bps()
      << " | loss=" << last_loss_fraction_
      << " | rtt_ms=" << (last_rtt_.IsFinite() ? last_rtt_.ms() : -1);
}

bool webrtc::L4SNetworkController::HasRecentCongestionSignals(Timestamp now) const {
  return !last_congestion_signal_.IsInfinite() && (now - last_congestion_signal_) < TimeDelta::Seconds(2);
}

bool webrtc::L4SNetworkController::IsEcnFeedbackFresh(Timestamp now) const {
  return HasRecentCongestionSignals(now) || (ecn_supported_ && (now - last_congestion_signal_) < TimeDelta::Seconds(5));
}

void webrtc::L4SNetworkController::UpdateThroughputWindow(const TransportPacketsFeedback& feedback) {
  constexpr TimeDelta kThroughputWindow = TimeDelta::Millis(500);
  constexpr TimeDelta kHistoricalWindow = TimeDelta::Seconds(120); 

  for (const auto& packet : feedback.packet_feedbacks) {
    if (packet.receive_time.IsFinite() && packet.sent_packet.send_time.IsFinite()) {
      throughput_window_.emplace_back(packet.receive_time, packet.sent_packet.size.bytes());
    }
  }

  if (!throughput_window_.empty()) {
    Timestamp latest_receive = throughput_window_.back().first;
    while (!throughput_window_.empty() && latest_receive - throughput_window_.front().first > kThroughputWindow) {
      throughput_window_.pop_front();
    }
  }
  
  int64_t window_bytes = 0;
  if (!throughput_window_.empty()) {
    Timestamp window_start = throughput_window_.front().first;
    Timestamp window_end = throughput_window_.back().first;
    for (const auto& entry : throughput_window_) window_bytes += entry.second;
    
    TimeDelta window_interval = window_end - window_start;
    if (window_interval >= TimeDelta::Millis(200)) {
      last_actual_bitrate_ = DataRate::BitsPerSec(static_cast<int64_t>((window_bytes * 8) / window_interval.seconds<double>()));
      historical_capacity_window_.emplace_back(window_end, last_actual_bitrate_);
    } 
  }
  
  if (!historical_capacity_window_.empty()) {
    Timestamp now = feedback.feedback_time;
    while (!historical_capacity_window_.empty() && (now - historical_capacity_window_.front().first) > kHistoricalWindow) {
      historical_capacity_window_.pop_front();
    }

    historical_max_capacity_ = DataRate::Zero();
    for (const auto& entry : historical_capacity_window_) {
      historical_max_capacity_ = std::max(historical_max_capacity_, entry.second);
    }
  }
}

void webrtc::L4SNetworkController::LogPeriodicMetrics(Timestamp at_time) {
  if (!metrics_enabled_ || !metrics_collector_) return;
  if (at_time - metrics_last_logged_ < kMetricsLoggingInterval) return;
  
  metrics_last_logged_ = at_time;
  DataRate target_rate = target_rate_.value_or(DataRate::Zero());
  metrics_collector_->LogBandwidthMetrics(at_time, target_rate, last_actual_bitrate_, last_acked_bitrate_, last_send_rate_);      
  
  static Timestamp last_delay_log_time = Timestamp::MinusInfinity();
  if (last_rtt_.IsFinite() && !last_rtt_.IsZero() && (last_delay_log_time.IsInfinite() || (at_time - last_delay_log_time) >= TimeDelta::Millis(500))) {
    metrics_collector_->LogDelayMetrics(at_time, last_rtt_, last_rtt_ / 2, TimeDelta::Zero());
    last_delay_log_time = at_time;
  }
  
  metrics_collector_->LogLossMetrics(at_time, last_loss_fraction_, last_packets_lost_);
  metrics_collector_->LogPeriodicSummary(at_time);
}

bool webrtc::L4SNetworkController::IsApplicationLimited() const {
  if (!alr_detector_) return false;
  return alr_detector_->GetApplicationLimitedRegionStartTime().has_value();
}

void webrtc::L4SNetworkController::UpdateAlrDetector(const TransportPacketsFeedback& feedback) {
  if (alr_detector_ && probe_controller_) {
    std::optional<int64_t> alr_start_time = alr_detector_->GetApplicationLimitedRegionStartTime();
    if (previously_in_alr_ && !alr_start_time.has_value()) {
      probe_controller_->SetAlrEndedTimeMs(feedback.feedback_time.ms());
      if (acked_estimator_) acked_estimator_->SetAlrEndedTime(feedback.feedback_time);
    }
    previously_in_alr_ = alr_start_time.has_value();
  }
}

bool webrtc::L4SNetworkController::ShouldExitDiscoveryMode(Timestamp now) const {
  if (!prague_estimator_ || !prague_estimator_->IsDiscoveryModeActive()) return false;
  DataRate current_rate = prague_estimator_->GetCurrentEstimate();
  if (current_rate.bps() >= 10000000) return true;
  return false;
}

void webrtc::L4SNetworkController::InitiateStatefulProbe(Timestamp now, NetworkControlUpdate* update, double base_multiplier, const std::string& state_name) {
  if (!EncoderNeedsMoreHeadroom(base_multiplier)) return;
  DataRate current_estimate = target_rate_.value_or(DataRate::KilobitsPerSec(300));
  
  double final_multiplier = base_multiplier;
  if (IsApplicationLimited()) final_multiplier += 0.15; 
  
  DataRate probe_rate = current_estimate * final_multiplier;
  if (max_target_rate_) probe_rate = std::min(probe_rate, *max_target_rate_);
  
  auto custom_probe = CreateCustomProbe(now, probe_rate);
  if (custom_probe) {
    update->probe_cluster_configs.push_back(*custom_probe);
    StartProbeHold(now);
  }
}

bool webrtc::L4SNetworkController::IsProbeDataValid(Timestamp now) const {
  if (last_probe_time_.IsInfinite()) return false;
  if (!last_congestion_signal_.IsInfinite() && last_congestion_signal_ >= last_probe_time_) return false; 
  TimeDelta since_probe = now - last_probe_time_;
  if (since_probe >= (config_.probe_interval * 2)) return false;
  return true;  
}

void webrtc::L4SNetworkController::HandleRecoveryDetection(int ce_count, Timestamp now) {
  if (ce_count > 0) {
    if (recovery_mode_active_) {
      recovery_mode_active_ = false;
      recovery_probe_bootstrapped_ = false;
      recovery_cooldown_until_ = now + config_.recovery_reentry_cooldown;
    }
    return;
  }

  if (recovery_mode_active_) {
    TimeDelta recovery_duration = now - recovery_start_time_;
    bool has_converged = prague_estimator_ && prague_estimator_->HasConvergedWithProbe();
    if (recovery_duration > TimeDelta::Seconds(10) || has_converged) {
      recovery_mode_active_ = false;
      recovery_probe_bootstrapped_ = false;

      if (has_converged) {
        recovery_cooldown_until_ = now + kRecoveryCooldown;
      } else {
        recovery_cooldown_until_ = now + config_.recovery_reentry_cooldown;
      }
    }
    return;
  }

  if (ce_count == 0) {
    if (CanEnterRecoveryState(now)) {
      if (prague_estimator_) prague_estimator_->EnterAdditiveMode(now);
      recovery_mode_active_ = true;
      recovery_probe_bootstrapped_ = false;
      recovery_start_time_ = now;
    }
  }
}

}  // namespace webrtc