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
};

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_L4S_L4S_NETWORK_CONTROLLER_H_