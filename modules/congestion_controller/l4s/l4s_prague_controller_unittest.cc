#include "modules/congestion_controller/l4s/l4s_prague_controller.h"

#include "api/transport/network_types.h"
#include "test/gtest.h"

namespace webrtc {
namespace test {

class L4SPragueControllerTest : public ::testing::Test {
 protected:
  L4SPragueControllerTest()
      : controller_(FieldTrialsView()) {}
      
  L4SPragueController controller_;
};

TEST_F(L4SPragueControllerTest, InitiallyInactive) {
  EXPECT_FALSE(controller_.IsActive());
}

TEST_F(L4SPragueControllerTest, BecomesActiveWithEcnFeedback) {
  TransportPacketsFeedback feedback;
  feedback.feedback_time = Timestamp::Millis(1000);
  
  // Add packet with ECN marking
  PacketResult packet_result;
  packet_result.sent_packet.send_time = Timestamp::Millis(900);
  packet_result.sent_packet.has_rtp_sequence_number = true;
  packet_result.sent_packet.rtp_sequence_number = 100;
  packet_result.ecn = EcnMarking::kEct1;
  packet_result.receive_time = Timestamp::Millis(950);
  
  feedback.packet_feedbacks.push_back(packet_result);
  
  controller_.UpdateEcnFeedback(feedback);
  
  EXPECT_TRUE(controller_.IsActive());
}

TEST_F(L4SPragueControllerTest, ReducesRateWithCongestion) {
  // First update RTT
  controller_.UpdateRtt(TimeDelta::Millis(50));
  
  // Send some regular ECT(1) feedback to establish a baseline
  TransportPacketsFeedback baseline_feedback;
  baseline_feedback.feedback_time = Timestamp::Millis(1000);
  
  for (int i = 0; i < 10; ++i) {
    PacketResult packet_result;
    packet_result.sent_packet.send_time = Timestamp::Millis(900 + i);
    packet_result.sent_packet.has_rtp_sequence_number = true;
    packet_result.sent_packet.rtp_sequence_number = 100 + i;
    packet_result.ecn = EcnMarking::kEct1;
    packet_result.receive_time = Timestamp::Millis(950 + i);
    
    baseline_feedback.packet_feedbacks.push_back(packet_result);
  }
  
  controller_.UpdateEcnFeedback(baseline_feedback);
  
  // Get the baseline rate
  auto baseline_rate = controller_.GetTargetRate(Timestamp::Millis(1100));
  ASSERT_TRUE(baseline_rate.has_value());
  
  // Now send feedback with some CE marks
  TransportPacketsFeedback congestion_feedback;
  congestion_feedback.feedback_time = Timestamp::Millis(1200);
  
  for (int i = 0; i < 10; ++i) {
    PacketResult packet_result;
    packet_result.sent_packet.send_time = Timestamp::Millis(1100 + i);
    packet_result.sent_packet.has_rtp_sequence_number = true;
    packet_result.sent_packet.rtp_sequence_number = 110 + i;
    // Mark 3 out of 10 packets as CE
    packet_result.ecn = (i % 3 == 0) ? EcnMarking::kCe : EcnMarking::kEct1;
    packet_result.receive_time = Timestamp::Millis(1150 + i);
    
    congestion_feedback.packet_feedbacks.push_back(packet_result);
  }
  
  controller_.UpdateEcnFeedback(congestion_feedback);
  
  // Get the new rate after congestion
  auto congestion_rate = controller_.GetTargetRate(Timestamp::Millis(1300));
  ASSERT_TRUE(congestion_rate.has_value());
  
  // Rate should be reduced due to CE marks
  EXPECT_LT(*congestion_rate, *baseline_rate);
}

}  // namespace test
}  // namespace webrtc