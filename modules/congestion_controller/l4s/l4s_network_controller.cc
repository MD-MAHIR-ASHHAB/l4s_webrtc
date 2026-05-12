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




//hold state during reduction, then context-aware AI step calculation with probe constraints and ALR safety, followed by mode escapes and alpha decay logic

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

    // 1. Calculate the dynamic Pipeline Delay
    // This is the time it takes for a rate cut to reach the router, plus the time
    // it takes for the already-marked queue to flush to the receiver.
    TimeDelta queue_bloat = current_rtt_.IsFinite() && baseline_rtt_.IsFinite() 
                            ? (current_rtt_ - baseline_rtt_) 
                            : TimeDelta::Millis(0);
    TimeDelta pipeline_delay = (current_rtt_.IsFinite() ? current_rtt_ : TimeDelta::Millis(100)) + queue_bloat;
    
    // Ensure minimum safe cooldown
    pipeline_delay = std::max(pipeline_delay, TimeDelta::Millis(50));

    // 2. Check the Cooldown Gate
    bool gate_open = last_md_time_.IsInfinite() || 
                     (current_time - last_md_time_ >= pipeline_delay);

    if (gate_open) {
      // 3. Make the Pure L4S Multiplicative Decrease
      direction_flag_ = -1;
      double reduction_factor = 1.0 - (alpha_ / 2.0);

      // Safety bounds (always cut at least 5%, never cut below 20%)
      reduction_factor = std::min(reduction_factor, 0.95);
      reduction_factor = std::max(reduction_factor, 0.20);

      DataRate reduced = std::max(current_rate * reduction_factor, min_target_rate_);
      reduced = std::max(reduced, DataRate::KilobitsPerSec(20));
      congestion_based_estimate_ = reduced;
      
      // 4. Reset the Cooldown Gate
      last_md_time_ = current_time;
      last_ai_update_time_ = current_time; // CRITICAL FIX: Reset AI clock on cut

      RTC_LOG(LS_VERBOSE) << "Prague: Executed Pure L4S Cut (alpha=" << alpha_
                       << ", reduction_factor=" << reduction_factor
                       << "), new rate=" << congestion_based_estimate_.bps() 
                       << " bps. Cooldown locked for " << pipeline_delay.ms() << "ms.";
    } else {
      // 5. During the cooldown, we do nothing to the target rate. 
      // We are waiting for the network to digest the previous cut.
      // (alpha_ is still being updated dynamically at the top of the function).
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

double webrtc::PragueCapacityEstimator::CalculateDiscoveryStep(double current_bps, double elapsed_s) const {
  // --- GCC Multiplicative Increase (Aggressive Discovery) ---
  // 8% growth per second (1.08^t), capped at 1.0s elapsed time per update to prevent wild leaps.
  double growth_factor = 1.08;
  growth_factor = std::pow(growth_factor, std::min(elapsed_s, 1.0)); 
  
  // Enforce a GCC floor of 1000 bps increase so we don't stall at very low rates
  return std::max(current_bps * (growth_factor - 1.0), 1000.0);
}

double webrtc::PragueCapacityEstimator::CalculateRecoveryStep(double current_bps, double target_probe_bps, double elapsed_s) const {
  // --- Accelerated Catch-Up (Recovery) ---
  // We have a proven physical probe ceiling. Smoothly close the gap.
  double gap_bps = target_probe_bps - current_bps;
  
  // Approach the ceiling asymptotically (cover 33% of the remaining gap per second)
  double catch_up_rate_bps_per_s = std::max(gap_bps * 0.33, 40000.0); // floor of 40 kbps/s
  
  // Safety cap: Never grow faster than ~500 kbps per second to prevent pacer micro-bursts
  catch_up_rate_bps_per_s = std::min(catch_up_rate_bps_per_s, 500000.0);
  
  return catch_up_rate_bps_per_s * elapsed_s;
}

double webrtc::PragueCapacityEstimator::CalculateStableStep(double current_bps, double elapsed_s) const {
  // --- Congestion Avoidance (Cautious GCC-Parity Growth) ---
  // Grow by a small percentage (4%) of the current rate per second.
  double increase_rate_bps_per_s = std::max(10000.0, current_bps * 0.04); // floor of 10 kbps/s
  
  // Cap the growth to prevent sudden micro-bursts
  increase_rate_bps_per_s = std::min(increase_rate_bps_per_s, 500000.0);
  
  return increase_rate_bps_per_s * elapsed_s;
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
    // Slightly more permissive queue-clear threshold to avoid stalling growth on
    // moderate jitter while still requiring low queue pressure.
    bool queue_is_clear = rtt_bloat < TimeDelta::Millis(30);
    bool past_hold_time = additive_hold_until_.IsInfinite() ||
                          current_time >= additive_hold_until_;
    double elapsed_s = ai_elapsed.seconds<double>();
    double current_bps = static_cast<double>(congestion_based_estimate_.bps());

    DataRate proposed_rate = congestion_based_estimate_;

    if (!is_app_limited && queue_is_clear && past_hold_time) {
      double final_step_bps = 0.0;
      // Check if a recent probe is actively trying to pull us up
      bool probe_constraint_fresh = !probe_constraint_time_.IsInfinite() &&
                                    (current_time - probe_constraint_time_) < TimeDelta::Seconds(5);
      bool probe_pulling_up = probe_constraint_fresh && 
                              probe_constraint_ > (congestion_based_estimate_ * 1.10);

// --- THE GROWTH POLICY ROUTER ---
      if (discovery_mode_active_) {
        final_step_bps = CalculateDiscoveryStep(current_bps, elapsed_s);
      } 
      else if (probe_pulling_up) {
        final_step_bps = CalculateRecoveryStep(current_bps, static_cast<double>(probe_constraint_.bps()), elapsed_s);
      } 
      else {
        final_step_bps = CalculateStableStep(current_bps, elapsed_s);
      }

      // Smooth the fractional bits using your existing accumulator
      ai_bits_accumulator_ += final_step_bps;
      
      if (ai_bits_accumulator_ >= 1.0) {
        int64_t whole_bits_per_sec = static_cast<int64_t>(ai_bits_accumulator_);
        proposed_rate = congestion_based_estimate_ + DataRate::BitsPerSec(whole_bits_per_sec);
        ai_bits_accumulator_ -= whole_bits_per_sec; 
      }
    } else if (is_app_limited && !discovery_mode_active_ && queue_is_clear &&
               past_hold_time && actual_throughput > DataRate::Zero()) {
      // ALR-safe upward nudge: hold-or-grow only, never reduce outside CE.
      // This addresses long plateaus when the app is limited but still pushing
      // close to target on a clear queue.
      bool near_target_delivery =
          actual_throughput >= (congestion_based_estimate_ * 0.90);
      if (near_target_delivery) {
        constexpr double kAlrNudgeMinBpsPerSec = 2000.0;
        constexpr double kAlrNudgeMaxBpsPerSec = 10000.0;
        constexpr double kAlrNudgeFractionPerSec = 0.002;  // 0.2%/s

        double alr_nudge_bps_per_s = std::clamp(
            current_bps * kAlrNudgeFractionPerSec,
            kAlrNudgeMinBpsPerSec,
            kAlrNudgeMaxBpsPerSec);
        ai_bits_accumulator_ += alr_nudge_bps_per_s * elapsed_s;

        if (ai_bits_accumulator_ >= 1.0) {
          int64_t whole_bits_per_sec =
              static_cast<int64_t>(ai_bits_accumulator_);
          proposed_rate =
              congestion_based_estimate_ + DataRate::BitsPerSec(whole_bits_per_sec);
          ai_bits_accumulator_ -= whole_bits_per_sec;
        }
      }
    }

    // Apply throughput tether even during ALR to avoid target growth freezes.
    // CE-only reduction invariant: never decrease estimate here.
    if (actual_throughput > DataRate::Zero()) {
      DataRate max_allowed = discovery_mode_active_ ? (actual_throughput * 2.0)
                                                    : (actual_throughput * 1.15);
      if (proposed_rate > max_allowed) {
        proposed_rate = std::max(congestion_based_estimate_, max_allowed);
      }
    }

    // --- PROBE CONSTRAINT CHECK ---
    bool probe_constraint_fresh =
        !probe_constraint_time_.IsInfinite() &&
        (current_time - probe_constraint_time_) < TimeDelta::Seconds(10);

    if (probe_constraint_fresh &&
        probe_constraint_confidence_ > 0.0 &&
        probe_constraint_ > congestion_based_estimate_) {
      proposed_rate = std::min(proposed_rate, probe_constraint_);
    }

    congestion_based_estimate_ = proposed_rate;
    congestion_based_estimate_ = std::min(congestion_based_estimate_, max_target_rate_);
  }

  last_ai_update_time_ = current_time; 

  if (alpha_ > 0.0) {
    TimeDelta since_last_ce = current_time - last_congestion_signal_;
    TimeDelta safe_clearance = current_rtt_.IsFinite() ? (current_rtt_ * 2) : TimeDelta::Millis(100);
    if (!last_congestion_signal_.IsInfinite() && since_last_ce > safe_clearance) {
      
      // --- NEW: Faster Penalty Shedding ---
      alpha_ *= 0.80; // Changed from 0.95 to 0.80 to forget congestion 4x faster
      
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


bool webrtc::PragueCapacityEstimator::HasConvergedWithProbe() const {
  // If there is no active probe constraint, we can't converge with it.
  if (probe_constraint_time_.IsInfinite() || probe_constraint_.IsZero()) {
    return false;
  }
  
  // If our current estimate has reached 95% of the target probe ceiling, we are done catching up.
  double rate_ratio = congestion_based_estimate_.bps() / static_cast<double>(probe_constraint_.bps());
  return rate_ratio >= 0.95;
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


  historical_capacity_window_.clear();
  historical_max_capacity_ = DataRate::Zero();
  recent_probes_window_.clear();

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

  // // Growth is feedback-driven via OnTransportPacketsFeedback.
  // // --- PRAGUE DICTATOR MODE ---
  // // Unconditionally push Prague's state to the Fusion Engine every 100ms.
  // // Never hide Prague's estimate just because we are in ALR or Reduction.
  double ecn_confidence = prague_estimator_->GetConfidence(msg.at_time);


// State-owned fusion policy
  DataRate fused_rate = FuseBandwidthEstimates(msg.at_time); // (or msg.at_time)
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
  DataRate fused_rate = FuseBandwidthEstimates(msg.feedback_time); // or msg.feedback_time
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
  DataRate base_fused_rate = prague_estimator_->GetCurrentEstimate();

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
      } else {
       
        // --- PREPARE STATE ---
        TimeDelta rtt_bloat = (last_rtt_.IsFinite() && base_rtt_.IsFinite()) 
                              ? (last_rtt_ - base_rtt_) : TimeDelta::PlusInfinity();
                              
        double starvation_ratio = 1.0;
        if (historical_max_capacity_ > DataRate::Zero()) {
            starvation_ratio = current_fused_rate.bps() / static_cast<double>(historical_max_capacity_.bps());
        }

        // --- APPLY DAMPENERS ---
        if (ce_ratio > 0.0) {
            if (starvation_ratio < 0.25) {
                // 1. SURVIVAL MODE: We are crushed. Yield extremely gently to prevent death.
                ce_ratio = ce_ratio * 0.1;
                RTC_LOG(LS_WARNING) << "L4S: Starvation detected (Ratio: " << starvation_ratio 
                                    << "). Dampening CE to " << ce_ratio;
            } 
            else if (rtt_bloat < TimeDelta::Millis(15)) {
                // 2. GCC PARITY MODE: We are not starving, but getting marked with an empty queue.
                // Cross-traffic is filling the coupled queue. Dampen to maintain a fair share.
                ce_ratio = ce_ratio * 0.25; 
                RTC_LOG(LS_VERBOSE) << "L4S: Cross-traffic bullying (Flat RTT). Dampening CE to " << ce_ratio;
            }
            // 3. NORMAL MODE: If neither is true (we are >25% capacity AND rtt_bloat > 15ms),
            // it means our own L4S traffic is bloating the queue. Let pure Prague math execute.
        }



        prague_estimator_->UpdateFromCongestionSignal(prague_estimator_->GetCurrentEstimate(), ce_ratio, feedback.feedback_time);
      }

      // ---  The Historical Safety Floor ---
      // Prevents the target rate from dropping into the kbps abyss during UDP bursts
      if (historical_max_capacity_ > DataRate::Zero()) {
        DataRate dynamic_floor = historical_max_capacity_ * 0.15; // Universal 15% anchor
        // Ensure absolute minimum viability for video (e.g., 500 kbps absolute floor)
        dynamic_floor = std::max(dynamic_floor, DataRate::KilobitsPerSec(500)); 
        
        if (prague_estimator_->GetCurrentEstimate() < dynamic_floor) {
          prague_estimator_->SetCurrentEstimate(dynamic_floor);
        }
      }
    }

    // Sync confidence with Fusion Engine
    double ecn_confidence = prague_estimator_->GetConfidence(feedback.feedback_time);
    if (prague_estimator_->GetDirectionFlag() == -1) {
      ecn_confidence = std::max(ecn_confidence, 0.95);
    }

    // RESET WINDOW
    window_ce_count_ = 0;
    window_ect_count_ = 0;
    window_start_time_ = feedback.feedback_time;
  }

  // 5. Recovery logic always sees the raw batch info
  HandleRecoveryDetection(batch_ect_count, batch_ce_count, feedback.feedback_time);
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
    Timestamp now = feedback.feedback_time;
    
    // --- PILLAR 3: Smear-Resistant Max Filter ---
    recent_probes_window_.emplace_back(now, measured_probe_rate.value());
    
    // Evict probes older than 3 seconds
    while (!recent_probes_window_.empty() && 
           (now - recent_probes_window_.front().first) > TimeDelta::Seconds(3)) {
      recent_probes_window_.pop_front();
    }
    // Find the max valid probe in the recent window
    DataRate effective_probe_rate = DataRate::Zero();
    for (const auto& entry : recent_probes_window_) {
        effective_probe_rate = std::max(effective_probe_rate, entry.second);
    }

    // Reality Check Floor: Only accept probes that aren't mathematically impossible
    if (last_actual_bitrate_.IsZero() || effective_probe_rate >= last_actual_bitrate_) {
      bool in_reduction = prague_estimator_ && prague_estimator_->GetDirectionFlag() == -1;
      bool recent_congestion = HasRecentCongestionSignals(now);

      bool is_probe_valid = IsProbeDataValid(now);

      bool block_probe_uplift = probe_packet_has_ce || in_reduction || recent_congestion || !is_probe_valid;

      
      if (block_probe_uplift) {
        probe_confidence = 0.0;
        RTC_LOG(LS_VERBOSE)
            << "L4S: Suppressing probe authority during active congestion handling"
            << " (probe_ce=" << probe_packet_has_ce
            << ", reduction=" << in_reduction
            << ", recent_congestion=" << recent_congestion
            << ", probe_valid=" << is_probe_valid << ")";
        if (prague_estimator_) {
          prague_estimator_->ClearProbeConstraint();
        }
      }

      // Probe indicates headroom; additive increase determines ramp speed.
      if (prague_estimator_ && !block_probe_uplift) {
        DataRate current_prague = prague_estimator_->GetCurrentEstimate();

        // Accept the probe as an upper growth indicator only if it proves uplift.
        if (effective_probe_rate > (current_prague * 1.05)) {
          DataRate max_uplift = current_prague * 1.5;
          DataRate probe_ceiling =
              std::min(effective_probe_rate * 0.95, max_uplift);

          probe_reject_streak_ = 0;
          next_probe_allowed_at_ = Timestamp::MinusInfinity();

          RTC_LOG(LS_INFO) << "L4S: Filtered Probe accepted. Max Probe=" << effective_probe_rate.kbps()
                           << "k, additive-growth ceiling=" << probe_ceiling.kbps() << "k.";
          prague_estimator_->SetProbeConstraint(probe_ceiling, probe_confidence, now);
        }
      }
    } else {
      probe_reject_streak_ = std::min(probe_reject_streak_ + 1, 4);
      int64_t backoff_seconds =
          std::min<int64_t>(12, 3 * (1LL << (probe_reject_streak_ - 1)));
      next_probe_allowed_at_ = feedback.feedback_time + TimeDelta::Seconds(backoff_seconds);

      RTC_LOG(LS_INFO) << "L4S: Probe result discarded (physically impossible): "
                       << effective_probe_rate.bps() << " bps < actual throughput "
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

  // ---ALR Emergency Escape ---
  // If we are in ALR, but our rate is severely depressed compared to known history, 
  // the encoder is in a death spiral. Force probing to break out.
  if (app_limited && historical_max_capacity_ > DataRate::Zero()) {
      if (current_target < historical_max_capacity_ * 0.35) { // 65% drop from max capacity
          RTC_LOG(LS_WARNING) << "L4S: ALR Emergency Escape! Bypassing application limit to find headroom.";
          app_limited = false;
      }
  }
  
  // TRIGGER A: Probe demand is delivery-driven (not sender burst-driven).
  // If acked is not available/stable, rely on actual throughput only.
  // Relax demand gate a bit (90% -> 85%) so probing can proceed when delivery
  // is close to target but mildly below due to normal VBR fluctuations.
  bool demand_from_actual = !actual_rate.IsZero() && actual_rate > (current_target * 0.85);
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
      (!actual_rate.IsZero() && actual_rate > (current_target * 0.75)) ||
      (acked_rate > DataRate::Zero() && acked_rate > (current_target * 0.60));
  
  // TRIGGER C: Is the physical queue completely drained?
  // FIX: Fail closed. If we don't know the baseline, assume the queue is bloated.
  TimeDelta rtt_bloat = (last_rtt_.IsFinite() && base_rtt_.IsFinite()) ?
                        (last_rtt_ - base_rtt_) : TimeDelta::PlusInfinity();
  bool queue_is_empty_recovery = rtt_bloat < TimeDelta::Millis(15);
  bool queue_is_empty_periodic = rtt_bloat < TimeDelta::Millis(30);

  TimeDelta since_last_probe = last_probe_time_.IsInfinite() ? TimeDelta::PlusInfinity() : (now - last_probe_time_);

  // --- 1. EVENT-DRIVEN RECOVERY PROBING ---
  if (recovery_mode_active_) {
    TimeDelta recovery_interval = std::max(GetRttScaledInterval(), TimeDelta::Seconds(3));
    
    if (since_last_probe >= recovery_interval) {
        // Probe only if app demand is sustained, delivery supports it, and queue is truly clear.
        if (demand_is_sustained && delivery_supports_target && queue_is_empty_recovery) {
            RTC_LOG(LS_VERBOSE) << "L4S: Event-Driven Recovery Probe Triggered! Send(" 
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
          RTC_LOG(LS_VERBOSE) << "L4S: Event-Driven Periodic Probe Triggered! Send(" 
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

  // --- PILLAR 5: Starvation-Aware Latency Gate ---
  if (last_rtt_.IsFinite() && base_rtt_.IsFinite()) {
      TimeDelta bloat = last_rtt_ - base_rtt_;
      TimeDelta allowed_bloat = TimeDelta::Millis(30); // Normal strict L4S gate for periodic probes
      
      // If we are starving, the competitor is causing the bloat. 
      // We must relax the gate to allow discovery probes to fight back.
      DataRate current_target = target_rate_.value_or(DataRate::Zero());
      if (historical_max_capacity_ > DataRate::Zero() && 
          current_target < (historical_max_capacity_ * 0.35)) {
          allowed_bloat = TimeDelta::Millis(50); // Relaxed starvation gate
      }

      if (bloat > allowed_bloat) {
          RTC_LOG(LS_VERBOSE) << "L4S: Blocking periodic probe due to standing queue bloat (" 
                              << bloat.ms() << "ms > " << allowed_bloat.ms() << "ms).";
          return false;
      }
  } else {
      return false; // Fail closed if no baseline
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
  if (!prague_estimator_) {
    return target_rate_.value_or(DataRate::KilobitsPerSec(300));
  }

  // 1. Manage Discovery State
  // We simply ask Prague if it thinks it is in discovery mode. 
  // No need to check controller_state_ here.
  if (prague_estimator_->IsDiscoveryModeActive() && ShouldExitDiscoveryMode(now)) {
    prague_estimator_->ExitDiscoveryMode("probe-Prague convergence or fallback threshold");
  }

  // 2. Prague is the absolute authority.
  DataRate fused_rate = prague_estimator_->GetCurrentEstimate();
  
  // 3. Apply Hard Constraints
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
  constexpr TimeDelta kHistoricalWindow = TimeDelta::Seconds(15); // 15-second long-term memory

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

          historical_capacity_window_.emplace_back(window_end, last_actual_bitrate_);
    } 
    // If < 200ms, we simply do nothing and retain the existing last_actual_bitrate_.
    // DO NOT set it to Zero here, otherwise the headroom checks will break!
  }
  // Clean up historical window
  if (!historical_capacity_window_.empty()) {
    Timestamp now = feedback.feedback_time;
    while (!historical_capacity_window_.empty() && 
           (now - historical_capacity_window_.front().first) > kHistoricalWindow) {
      historical_capacity_window_.pop_front();
    }

    // Find the max capacity in the last 10 seconds
    historical_max_capacity_ = DataRate::Zero();
    for (const auto& entry : historical_capacity_window_) {
      historical_max_capacity_ = std::max(historical_max_capacity_, entry.second);
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



bool webrtc::L4SNetworkController::ShouldExitDiscoveryMode(Timestamp now) const {
  if (!prague_estimator_ || !prague_estimator_->IsDiscoveryModeActive()) {
    return false;
  }

  // Exit if CE marks detected (already handled in Prague estimator)
  if (prague_estimator_->GetDirectionFlag() == -1) {
    return true;
  }

  // Hard discovery timeout (flicker-proof): exit after fixed time in state.
  // This only changes state mode; it does not reduce rate.
  if (state_entered_at_.IsFinite()) {
    TimeDelta time_in_state = now - state_entered_at_;
    constexpr TimeDelta kDiscoveryHardTimeout = TimeDelta::Seconds(50);
    // constexpr TimeDelta kDiscoverySoftTimeout = TimeDelta::Seconds(25);

    if (time_in_state >= kDiscoveryHardTimeout) {
      RTC_LOG(LS_INFO) << "L4S: Exiting discovery mode - hard timeout "
                       << "(" << time_in_state.seconds<double>() << "s >= "
                       << kDiscoveryHardTimeout.seconds<double>() << "s)";
      return true;
    }
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


bool webrtc::L4SNetworkController::IsProbeDataValid(Timestamp now) const {
  if (last_probe_time_.IsInfinite()) {
    return false;
  }

  // --- 1. CAUSALITY CHECK ---
  // If we received a CE congestion signal AFTER this probe was measured,
  // the network queue has filled. The probe's capacity measurement is dead.
  if (!last_congestion_signal_.IsInfinite() && last_congestion_signal_ >= last_probe_time_) {
    return false; 
  }

  // --- 2. FRESHNESS CHECK ---
  // Without the Fusion engine, we don't need a sliding scale. 
  // We simply trust the probe for up to two full intervals before discarding it as stale.
  TimeDelta since_probe = now - last_probe_time_;
  if (since_probe >= (config_.probe_interval * 2)) {
    return false;
  }
  
  return true;  
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
        // !IsApplicationLimited() && 
        rtt_bloat < TimeDelta::Millis(30)) { // <--- THE LATENCY GATE

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
    bool has_converged = prague_estimator_ && prague_estimator_->HasConvergedWithProbe();
    if (recovery_duration > TimeDelta::Seconds(10) || 
        has_converged) {
      
      recovery_mode_active_ = false;
      recovery_probe_bootstrapped_ = false;
      consecutive_clean_packets_ = 0;
      clean_ect_run_start_ = Timestamp::MinusInfinity();

      if (has_converged) {
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




}  // namespace webrtc
