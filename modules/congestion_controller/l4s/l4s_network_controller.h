#ifndef MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_
#define MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_

#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include "api/transport/network_control.h"
#include "api/transport/network_types.h"
#include "api/units/data_rate.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/congestion_controller/goog_cc/probe_controller.h"
#include "modules/congestion_controller/l4s/l4s_prague_controller.h"
#include "rtc_base/experiments/field_trial_parser.h"

// GCC-inspired bandwidth estimation integration
#include "modules/congestion_controller/goog_cc/acknowledged_bitrate_estimator.h"
#include "modules/congestion_controller/goog_cc/delay_based_bwe.h"
#include "modules/congestion_controller/goog_cc/send_side_bandwidth_estimation.h"

namespace webrtc {

struct L4SControllerConfig {
  // Whether to fallback to GCC if L4S isn't supported
  bool fallback_to_gcc = true;
  // Whether to use ECT(1) marking for packets
  bool use_ect1_marking = true;
};

// Implementation of Network Controller Interface that uses L4S-based
// congestion control. It can reuse some components from GCC when needed
// and falls back to GCC if L4S (ECN) isn't supported.
class L4SNetworkController : public NetworkControllerInterface {
 public:
  L4SNetworkController(NetworkControllerConfig config,
                      L4SControllerConfig l4s_config);
  ~L4SNetworkController() override;

  // NetworkControllerInterface implementation
  NetworkControlUpdate OnNetworkAvailability(NetworkAvailability msg) override;
  NetworkControlUpdate OnNetworkRouteChange(NetworkRouteChange msg) override;
  NetworkControlUpdate OnProcessInterval(ProcessInterval msg) override;
  NetworkControlUpdate OnRemoteBitrateReport(RemoteBitrateReport msg) override;
  NetworkControlUpdate OnRoundTripTimeUpdate(RoundTripTimeUpdate msg) override;
  NetworkControlUpdate OnSentPacket(SentPacket msg) override;
  NetworkControlUpdate OnReceivedPacket(ReceivedPacket msg) override;
  NetworkControlUpdate OnStreamsConfig(StreamsConfig msg) override;
  NetworkControlUpdate OnTargetRateConstraints(
      TargetRateConstraints msg) override;
  NetworkControlUpdate OnTransportLossReport(TransportLossReport msg) override;
  NetworkControlUpdate OnTransportPacketsFeedback(
      TransportPacketsFeedback msg) override;
  NetworkControlUpdate OnNetworkStateEstimate(
      NetworkStateEstimate msg) override;
      
  bool IsL4SActive() const;


 private:
  NetworkControlUpdate CreateRateUpdate(Timestamp at_time) const;
  void MaybeTriggerOnNetworkChanged(NetworkControlUpdate* update,
                                    Timestamp at_time);
  
  void ProcessEcnFeedback(const TransportPacketsFeedback& feedback);
  void UpdateNetworkCapacityEstimate(const TransportPacketsFeedback& feedback);

  std::optional<Timestamp> last_update_time_;
  TimeDelta update_interval_ = TimeDelta::Millis(25);

  // Environment
  const Environment env_;
  const bool fallback_to_gcc_;
  const bool use_ect1_marking_;

  // L4S-specific controllers
  std::unique_ptr<L4SPragueController> prague_controller_;
  
  // GCC components we can reuse
  std::unique_ptr<ProbeController> probe_controller_;
  
  // Current state
  bool ecn_supported_ = false;
  bool ecn_capable_network_ = false;
  std::optional<DataRate> target_rate_;
  std::optional<DataRate> min_target_rate_;
  std::optional<DataRate> max_target_rate_;
  std::optional<DataRate> starting_rate_;
  
  // Tracking congestion signals
  int ce_count_ = 0;
  int ect_count_ = 0;
  Timestamp last_congestion_signal_ = Timestamp::MinusInfinity();
  
  // For fallback to GCC if needed
  std::unique_ptr<NetworkControllerInterface> gcc_controller_;

  // Add bandwidth estimation integration to the private section
  std::optional<DataRate> estimated_bandwidth_;
  DataRate max_realistic_bandwidth_ = DataRate::KilobitsPerSec(100000); // 100 Mbps default reasonable limit

  // GCC-inspired bandwidth estimation components  
  std::unique_ptr<AcknowledgedBitrateEstimator> acknowledged_bitrate_estimator_;
  std::unique_ptr<DelayBasedBwe> delay_based_bwe_;
  std::unique_ptr<SendSideBandwidthEstimation> bandwidth_estimation_;
  
  // Bandwidth estimation tracking
  DataRate last_acknowledged_rate_ = DataRate::Zero();
  DataRate last_delay_based_estimate_ = DataRate::Zero();
  std::optional<DataRate> last_probe_result_;
  
  // Enhanced RTT tracking (similar to GCC)
  std::deque<int64_t> feedback_max_rtts_;
  TimeDelta last_estimated_round_trip_time_ = TimeDelta::PlusInfinity();
};

/*
 * L4S Network Controller with GCC-Inspired Bandwidth Estimation
 * 
 * Comprehensive Improvements:
 * 1. Integrated GCC's acknowledged bitrate estimator for actual throughput measurement
 * 2. Added delay-based BWE for detecting network congestion through packet delays  
 * 3. Enhanced RTT tracking with moving window analysis (like GCC)
 * 4. Network capacity estimation that learns from actual network behavior
 * 5. BWE-informed rate capping that respects real network conditions
 * 6. Prague controller integration with proper bandwidth constraints
 * 
 * This prevents the 100 Mbps VM network from being overdriven to 2+ Gbps
 * by providing realistic network capacity estimates and proper feedback loops.
 */

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_
