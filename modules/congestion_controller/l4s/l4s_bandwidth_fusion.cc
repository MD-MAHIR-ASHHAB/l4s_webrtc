#include "modules/congestion_controller/l4s/l4s_network_controller.h"

#include <algorithm>

#include "rtc_base/logging.h"

namespace webrtc {

L4SBandwidthFusion::L4SBandwidthFusion(const L4SControllerConfig& config)
    : config_(config) {}

L4SBandwidthFusion::~L4SBandwidthFusion() = default;

void L4SBandwidthFusion::UpdateEcnEstimate(DataRate estimate,
                                           double confidence,
                                           Timestamp now) {
  // Enforce absolute minimum of 20 kbps to prevent pacer crashes
  DataRate clamped_estimate = std::max(estimate, DataRate::KilobitsPerSec(20));
  RTC_LOG(LS_VERBOSE) << "L4S: Updating ECN estimate to "
                      << clamped_estimate.bps() << " bps with confidence "
                      << confidence;
  sources_.ecn_estimate = clamped_estimate;
  sources_.ecn_confidence = confidence;
  sources_.last_ecn_update = now;
}

void L4SBandwidthFusion::UpdateProbeEstimate(DataRate estimate,
                                             double confidence,
                                             Timestamp now) {
  RTC_LOG(LS_VERBOSE) << "L4S: Updating probe estimate to " << estimate.bps()
                      << " bps with confidence " << confidence;
  sources_.probe_estimate = estimate;
  sources_.probe_confidence = confidence;
  sources_.last_probe_update = now;
}

void L4SBandwidthFusion::UpdateAckedEstimate(DataRate estimate,
                                             double confidence,
                                             Timestamp now) {
  RTC_LOG(LS_VERBOSE) << "L4S: Updating acked estimate to " << estimate.bps()
                      << " bps with confidence " << confidence;
  sources_.acked_estimate = estimate;
  sources_.acked_confidence = confidence;
  sources_.last_acked_update = now;
}

DataRate L4SBandwidthFusion::GetFusedEstimateWithMode(
    Timestamp now,
    bool discovery_mode,
    bool recovery_mode,
    bool in_reduction,
    DataRate actual_rate) const {
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

DataRate L4SBandwidthFusion::GetDiscoveryModeFusedEstimate(
    Timestamp now,
    bool recovery_mode) const {
  (void)recovery_mode;

  DataRate probe_rate = sources_.probe_estimate;
  DataRate prague_rate = sources_.ecn_estimate;

  bool probe_confident =
      sources_.probe_confidence > config_.probe_confidence_threshold &&
      IsRecentlyUpdated(sources_.last_probe_update, now);
  bool prague_confident =
      sources_.ecn_confidence > config_.ecn_confidence_threshold &&
      IsRecentlyUpdated(sources_.last_ecn_update, now);

  DataRate fused_rate = DataRate::KilobitsPerSec(300);

  if (probe_confident) {
    fused_rate = probe_rate;

    if (prague_confident) {
      if (prague_rate < fused_rate) {
        // The Prague Veto: Prague saw CE marks and dictates a lower rate.
        fused_rate = prague_rate;
        RTC_LOG(LS_VERBOSE)
            << "L4S Fusion: Discovery Mode - Prague vetoed probe, using ECN rate: "
            << fused_rate.bps() << " bps";
      } else {
        // Prague is HIGHER than the probe. The probe got smeared by the AQM queue.
        // Hold Prague so we do not rubber-band down on a distorted probe sample.
        fused_rate = prague_rate;
        RTC_LOG(LS_VERBOSE)
            << "L4S Fusion: Discovery Mode - Ignored smeared probe ("
            << probe_rate.bps() << " bps), holding Prague rate: "
            << prague_rate.bps() << " bps";
      }
    } else {
      RTC_LOG(LS_VERBOSE) << "L4S Fusion: Discovery Mode - Trusting probe: "
                          << fused_rate.bps() << " bps";
    }
  } else if (prague_confident) {
    fused_rate = prague_rate;
    RTC_LOG(LS_VERBOSE)
        << "L4S Fusion: Discovery Mode - No valid probe, using Prague AI: "
        << fused_rate.bps() << " bps";
  } else {
    fused_rate = GetMostConfidentEstimate(now);
  }

  // Apply Reality Check Floor
  if (sources_.acked_confidence > 0.3 &&
      IsRecentlyUpdated(sources_.last_acked_update, now)) {
    if (fused_rate < sources_.acked_estimate) {
      fused_rate = sources_.acked_estimate;
    }
  }

  return std::max(fused_rate, DataRate::KilobitsPerSec(20));
}

DataRate L4SBandwidthFusion::GetMostConfidentEstimate(Timestamp now) const {
  DataRate best_estimate = DataRate::KilobitsPerSec(300);
  double best_confidence = 0.0;

  if (sources_.ecn_confidence > best_confidence &&
      IsRecentlyUpdated(sources_.last_ecn_update, now)) {
    best_estimate = sources_.ecn_estimate;
    best_confidence = sources_.ecn_confidence;
  }

  if (sources_.probe_confidence > best_confidence &&
      IsRecentlyUpdated(sources_.last_probe_update, now)) {
    best_estimate = sources_.probe_estimate;
    best_confidence = sources_.probe_confidence;
  }

  if (sources_.acked_confidence > best_confidence &&
      IsRecentlyUpdated(sources_.last_acked_update, now)) {
    best_estimate = sources_.acked_estimate;
    best_confidence = sources_.acked_confidence;
  }

  return best_estimate;
}

DataRate L4SBandwidthFusion::ValidateWithOtherSources(
    DataRate primary_estimate,
    const BandwidthSources& sources) const {
  // Utility method to prevent probe artifacts from causing runaway targets
  DataRate max_alternative = DataRate::Zero();

  if (sources.ecn_confidence > 0.3) {
    max_alternative = std::max(max_alternative, sources.ecn_estimate);
  }
  if (sources.acked_confidence > 0.3) {
    max_alternative = std::max(max_alternative, sources.acked_estimate);
  }

  // If a probe result is more than 2x our other known valid signals, it is likely
  // a DualPI2 scheduling artifact (burst spread over a long window). Cap it.
  if (max_alternative > DataRate::Zero() && primary_estimate > max_alternative * 2.0) {
    return max_alternative * 1.5;
  }

  return primary_estimate;
}

bool L4SBandwidthFusion::IsRecentlyUpdated(Timestamp last_update,
                                           Timestamp now) const {
  if (last_update.IsInfinite() || now.IsInfinite()) {
    return false;
  }
  // Keep probe authority short-lived so stale estimates cannot override Prague.
  return (now - last_update) < TimeDelta::Seconds(10);
}

}  // namespace webrtc
