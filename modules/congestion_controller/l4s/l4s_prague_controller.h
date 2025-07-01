#ifndef MODULES_CONGESTION_CONTROLLER_L4S_L4S_PRAGUE_CONTROLLER_H_
#define MODULES_CONGESTION_CONTROLLER_L4S_L4S_PRAGUE_CONTROLLER_H_

#include <deque>
#include <optional>

#include "api/field_trials_view.h"  // Add this include
#include "api/transport/network_types.h"
#include "api/units/data_rate.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "rtc_base/experiments/field_trial_parser.h"

namespace webrtc {

// Implementation of the L4S Prague congestion control algorithm
// Prague is a specific algorithm for L4S implementation
// See: https://datatracker.ietf.org/doc/draft-ietf-tsvwg-l4s-arch/
class L4SPragueController {
 public:
  explicit L4SPragueController(const FieldTrialsView& field_trials);
  ~L4SPragueController();

  // Updates ECN feedback information
  void UpdateEcnFeedback(const TransportPacketsFeedback& feedback);

  // Updates the RTT estimate
  void UpdateRtt(TimeDelta rtt);

  // Calculates the target send rate based on current rate
  std::optional<DataRate> GetTargetRate(Timestamp now, DataRate current_rate) const;

  // Determines if the controller is active
  bool IsActive() const;

 private:
  // Calculates the congestion window
  DataSize CalculateCongestionWindow() const;

  // Prague algorithm parameters
  FieldTrialParameter<double> alpha_;    // DCTCP-style decrease factor
  FieldTrialParameter<TimeDelta> rtt_filter_time_;  // Time window for RTT calculation
  FieldTrialParameter<TimeDelta> min_rtt_; // Minimum RTT assumed
  FieldTrialParameter<double> init_cwnd_; // Initial congestion window multiplier
  FieldTrialParameter<double> beta_;     // Multiplicative decrease factor

  // Current state
  std::optional<TimeDelta> rtt_;
  std::optional<TimeDelta> min_rtt_estimate_;
  double ecn_ce_ratio_ = 0.0;  // Current CE marking ratio
  bool active_ = false;        // Whether enough ECN feedback has been received

  // Timestamps
  Timestamp last_update_time_ = Timestamp::MinusInfinity();
  
  // Window of ECN feedback for calculating marking ratio
  struct EcnFeedbackPoint {
    Timestamp time;
    size_t ect_count;
    size_t ce_count;
  };
  std::deque<EcnFeedbackPoint> ecn_window_;
  size_t total_ect_packets_ = 0;
  size_t total_ce_packets_ = 0;
};

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_L4S_L4S_PRAGUE_CONTROLLER_H_