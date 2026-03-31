#include "modules/congestion_controller/l4s/l4s_network_controller.h"

#include "api/transport/network_types.h"
#include "api/units/data_rate.h"
#include "api/units/timestamp.h"
#include "test/gmock.h"
#include "test/gtest.h"

namespace webrtc {
namespace test {

using ::testing::_;
using ::testing::Field;
using ::testing::Matcher;
using ::testing::Property;

// Matcher for target rate.
Matcher<const TargetTransferRate&> TargetRateCloseTo(DataRate rate) {
  return Field(&TargetTransferRate::target_rate,
               Property(&DataRate::bps, testing::AllOf(
                   testing::Ge(rate.bps() * 0.9),
                   testing::Le(rate.bps() * 1.1))));
}

class L4SNetworkControllerTest : public ::testing::Test {
 protected:
  L4SNetworkControllerTest() {
    NetworkControllerConfig config;
    config.constraints.min_data_rate = DataRate::KilobitsPerSec(10);
    config.constraints.max_data_rate = DataRate::KilobitsPerSec(1500);
    config.constraints.starting_rate = DataRate::KilobitsPerSec(300);
    
    L4SControllerConfig l4s_config;
    l4s_config.fallback_to_gcc = true;
    l4s_config.use_ect1_marking = true;
    
    controller_ = std::make_unique<L4SNetworkController>(config, l4s_config);
  }
  
  std::unique_ptr<L4SNetworkController> controller_;
};

TEST_F(L4SNetworkControllerTest, InitialRateRespectsBounds) {
  ProcessInterval process_interval;
  process_interval.at_time = Timestamp::Millis(100);
  
  NetworkControlUpdate update = controller_->OnProcessInterval(process_interval);
  
  // Rate should be close to the starting rate
  ASSERT_TRUE(update.target_rate.has_value());
  EXPECT_THAT(*update.target_rate, TargetRateCloseTo(DataRate::KilobitsPerSec(300)));
}

TEST_F(L4SNetworkControllerTest, RespondsToEcnFeedback) {
  // First update RTT
  RoundTripTimeUpdate rtt_update;
  rtt_update.receive_time = Timestamp::Millis(100);
  rtt_update.round_trip_time = TimeDelta::Millis(50);
  controller_->OnRoundTripTimeUpdate(rtt_update);
  
  // Process interval to establish baseline
  ProcessInterval process_interval;
  process_interval.at_time = Timestamp::Millis(100);
  NetworkControlUpdate baseline_update = controller_->OnProcessInterval(process_interval);
  
  // Send some ECT(1) feedback to enable L4S
  TransportPacketsFeedback ect_feedback;
  ect_feedback.feedback_time = Timestamp::Millis(200);
  
  for (int i = 0; i < 20; ++i) {
    PacketResult packet_result;
    packet_result.sent_packet.send_time = Timestamp::Millis(150 + i);
    packet_result.sent_packet.sequence_number = 1000 + i;
    packet_result.sent_packet.has_rtp_sequence_number = true;
    packet_result.sent_packet.rtp_sequence_number = 100 + i;
    packet_result.ecn = EcnMarking::kEct1;
    packet_result.receive_time = Timestamp::Millis(180 + i);
    
    ect_feedback.packet_feedbacks.push_back(packet_result);
  }
  
  controller_->OnTransportPacketsFeedback(ect_feedback);
  
  // Now send feedback with some CE marks
  TransportPacketsFeedback ce_feedback;
  ce_feedback.feedback_time = Timestamp::Millis(300);
  
  for (int i = 0; i < 20; ++i) {
    PacketResult packet_result;
    packet_result.sent_packet.send_time = Timestamp::Millis(250 + i);
    packet_result.sent_packet.sequence_number = 1020 + i;
    packet_result.sent_packet.has_rtp_sequence_number = true;
    packet_result.sent_packet.rtp_sequence_number = 120 + i;
    // Mark half the packets as CE
    packet_result.ecn = (i % 2 == 0) ? EcnMarking::kCe : EcnMarking::kEct1;
    packet_result.receive_time = Timestamp::Millis(280 + i);
    
    ce_feedback.packet_feedbacks.push_back(packet_result);
  }
  
  NetworkControlUpdate congestion_update = 
      controller_->OnTransportPacketsFeedback(ce_feedback);
  
  // Rate should be reduced due to CE marks
  ASSERT_TRUE(congestion_update.target_rate.has_value());
  ASSERT_TRUE(baseline_update.target_rate.has_value());
  
  EXPECT_LT(congestion_update.target_rate->target_rate,
            baseline_update.target_rate->target_rate);
}

TEST_F(L4SNetworkControllerTest, FallbackToGccWhenNoEcn) {
  // Process interval with no ECN feedback
  ProcessInterval process_interval;
  process_interval.at_time = Timestamp::Millis(100);
  NetworkControlUpdate initial_update = controller_->OnProcessInterval(process_interval);
  
  // Send feedback with no ECN marks
  TransportPacketsFeedback no_ecn_feedback;
  no_ecn_feedback.feedback_time = Timestamp::Millis(200);
  
  for (int i = 0; i < 20; ++i) {
    PacketResult packet_result;
    packet_result.sent_packet.send_time = Timestamp::Millis(150 + i);
    packet_result.sent_packet.sequence_number = 1000 + i;
    packet_result.sent_packet.has_rtp_sequence_number = true;
    packet_result.sent_packet.rtp_sequence_number = 100 + i;
    packet_result.ecn = EcnMarking::kNotEct;  // No ECN
    packet_result.receive_time = Timestamp::Millis(180 + i);
    
    no_ecn_feedback.packet_feedbacks.push_back(packet_result);
  }
  
  NetworkControlUpdate fallback_update = 
      controller_->OnTransportPacketsFeedback(no_ecn_feedback);
  
  // Should still have a target rate (from GCC fallback)
  ASSERT_TRUE(fallback_update.target_rate.has_value());
}

}  // namespace test
}  // namespace webrtc