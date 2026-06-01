#include "modules/congestion_controller/l4s/l4s_network_controller.h"

#include <cstdio>

#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

namespace webrtc {

L4SMetricsCollector::L4SMetricsCollector(test::MetricsLogger* logger,
                                         const std::string& test_case_name)
    : logger_(logger), test_case_name_(test_case_name) {
  RTC_CHECK(logger_);
  RTC_LOG(LS_INFO) << "L4SMetricsCollector initialized for test case: "
                   << test_case_name_;
}

// Custom destructor to guarantee final metrics are written to disk on shutdown
L4SMetricsCollector::~L4SMetricsCollector() {
  RTC_LOG(LS_INFO) << "L4SMetricsCollector shutting down. Exporting final metrics for test case: " 
                   << test_case_name_;
  ExportToJsonFile("l4s_test_1.json");
}

void L4SMetricsCollector::LogBandwidthMetrics(
    Timestamp at_time,
    DataRate target_bitrate,
    DataRate actual_bitrate,
    std::optional<DataRate> acked_bitrate,
    std::optional<DataRate> send_rate) {
  if (at_time - last_bandwidth_log_ < kBandwidthLogInterval) {
    return;
  }
  last_bandwidth_log_ = at_time;

  if (target_bitrate.IsFinite()) {
    logger_->LogSingleValueMetric(
        "target_rate_mbps", test_case_name_, target_bitrate.bps() / 1e6,
        webrtc::test::Unit::kUnitless,
        webrtc::test::ImprovementDirection::kBiggerIsBetter,
        {{"timestamp_ms", std::to_string(at_time.ms())}});
  }

  DataRate tx_rate = send_rate.value_or(DataRate::Zero());
  logger_->LogSingleValueMetric(
      "send_rate_mbps", test_case_name_, tx_rate.bps() / 1e6,
      webrtc::test::Unit::kUnitless,
      webrtc::test::ImprovementDirection::kBiggerIsBetter,
      {{"timestamp_ms", std::to_string(at_time.ms())}});

  logger_->LogSingleValueMetric(
      "actual_rate_mbps", test_case_name_, actual_bitrate.bps() / 1e6,
      webrtc::test::Unit::kUnitless,
      webrtc::test::ImprovementDirection::kBiggerIsBetter,
      {{"timestamp_ms", std::to_string(at_time.ms())}});

  DataRate filtered_acked_rate = acked_bitrate.value_or(DataRate::Zero());
  UpdateThroughputStats(filtered_acked_rate);
  logger_->LogSingleValueMetric(
      "acked_rate_mbps", test_case_name_, filtered_acked_rate.bps() / 1e6,
      webrtc::test::Unit::kUnitless,
      webrtc::test::ImprovementDirection::kBiggerIsBetter,
      {{"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogDelayMetrics(Timestamp at_time,
                                          TimeDelta rtt,
                                          TimeDelta one_way_delay,
                                          TimeDelta jitter) {
  (void)jitter;
  if (at_time - last_delay_log_ < kDelayLogInterval) {
    return;
  }

  last_delay_log_ = at_time;
  UpdateDelayStats(rtt, one_way_delay);

  logger_->LogSingleValueMetric(
      "rtt_ms", test_case_name_, rtt.ms(), webrtc::test::Unit::kMilliseconds,
      webrtc::test::ImprovementDirection::kSmallerIsBetter,
      {{"timestamp_ms", std::to_string(at_time.ms())}});

  if (one_way_delay.IsFinite()) {
    logger_->LogSingleValueMetric(
        "one_way_delay_ms", test_case_name_, one_way_delay.ms(),
        webrtc::test::Unit::kMilliseconds,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"timestamp_ms", std::to_string(at_time.ms())}});
  }
}

void L4SMetricsCollector::LogLossMetrics(Timestamp at_time,
                                         double loss_fraction,
                                         int packets_lost) {
  if (at_time - last_loss_log_ < kLossLogInterval) {
    return;
  }

  last_loss_log_ = at_time;
  UpdateLossStats(loss_fraction);

  logger_->LogSingleValueMetric(
      "packet_loss_fraction", test_case_name_, loss_fraction,
      webrtc::test::Unit::kUnitless,
      webrtc::test::ImprovementDirection::kSmallerIsBetter,
      {{"timestamp_ms", std::to_string(at_time.ms())}});

  logger_->LogSingleValueMetric(
      "packets_lost_count", test_case_name_, packets_lost,
      webrtc::test::Unit::kCount,
      webrtc::test::ImprovementDirection::kSmallerIsBetter,
      {{"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogCongestionMetrics(Timestamp at_time,
                                               int ce_count,
                                               int ect_count,
                                               double congestion_ratio) {
  (void)ect_count;
  logger_->LogSingleValueMetric(
      "congestion_ce_count", test_case_name_, ce_count,
      webrtc::test::Unit::kCount,
      webrtc::test::ImprovementDirection::kSmallerIsBetter,
      {{"timestamp_ms", std::to_string(at_time.ms())}});

  logger_->LogSingleValueMetric(
      "congestion_ratio", test_case_name_, congestion_ratio,
      webrtc::test::Unit::kUnitless,
      webrtc::test::ImprovementDirection::kSmallerIsBetter,
      {{"timestamp_ms", std::to_string(at_time.ms())}});
}

void L4SMetricsCollector::LogPeriodicSummary(Timestamp at_time) {
  if (at_time - last_summary_log_ < kSummaryLogInterval) {
    return;
  }

  last_summary_log_ = at_time;

  if (throughput_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric(
        "acked_rate_avg_mbps", test_case_name_, throughput_stats_.GetAverage() / 1e6,
        webrtc::test::Unit::kUnitless,
        webrtc::test::ImprovementDirection::kBiggerIsBetter,
        {{"stat_type", "average"}, {"metric", "acked_rate"}});
    logger_->LogSingleValueMetric(
        "acked_rate_std_mbps", test_case_name_,
        throughput_stats_.GetStandardDeviation() / 1e6,
        webrtc::test::Unit::kUnitless,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "std_dev"}, {"metric", "acked_rate"}});
  }

  if (rtt_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric(
        "rtt_avg_ms", test_case_name_, rtt_stats_.GetAverage(),
        webrtc::test::Unit::kMilliseconds,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "average"}, {"metric", "rtt"}});
    logger_->LogSingleValueMetric(
        "rtt_std_ms", test_case_name_, rtt_stats_.GetStandardDeviation(),
        webrtc::test::Unit::kMilliseconds,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "std_dev"}, {"metric", "rtt"}});
  }

  if (delay_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric(
        "delay_avg_ms", test_case_name_, delay_stats_.GetAverage(),
        webrtc::test::Unit::kMilliseconds,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "average"}, {"metric", "delay"}});
    logger_->LogSingleValueMetric(
        "delay_std_ms", test_case_name_, delay_stats_.GetStandardDeviation(),
        webrtc::test::Unit::kMilliseconds,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "std_dev"}, {"metric", "delay"}});
  }

  if (loss_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric(
        "packet_loss_avg_fraction", test_case_name_, loss_stats_.GetAverage(),
        webrtc::test::Unit::kUnitless,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "average"}, {"metric", "packet_loss"}});
    logger_->LogSingleValueMetric(
        "packet_loss_std_fraction", test_case_name_,
        loss_stats_.GetStandardDeviation(), webrtc::test::Unit::kUnitless,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "std_dev"}, {"metric", "packet_loss"}});
  }

  ExportToJsonFile("l4s_test_1.json");
}

void L4SMetricsCollector::ExportToJsonFile(const std::string& filename) {
  if (!logger_) {
    return;
  }
  auto metrics = logger_->GetCollectedMetrics();
  FILE* f = fopen(filename.c_str(), "w");
  if (!f) {
    return;
  }

  fprintf(f, "[\n");
  for (size_t i = 0; i < metrics.size(); ++i) {
    const auto& m = metrics[i];
    fprintf(f, "  {\n");
    fprintf(f, "    \"name\": \"%s\",\n", m.name.c_str());
    fprintf(f, "    \"samples\": [");
    for (size_t j = 0; j < m.time_series.samples.size(); ++j) {
      const auto& s = m.time_series.samples[j];
      fprintf(f, "%s{\"timestamp_ms\": %lld, \"value\": %f}",
              (j > 0 ? ", " : ""),
              static_cast<long long>(s.timestamp.ms()),
              s.value);
    }
    fprintf(f, "]\n  }%s\n", (i + 1 < metrics.size()) ? "," : "");
  }
  fprintf(f, "]\n");
  fclose(f);
}

void L4SMetricsCollector::UpdateThroughputStats(DataRate actual_bitrate) {
  throughput_stats_.AddSample(actual_bitrate.bps());
}

void L4SMetricsCollector::UpdateDelayStats(TimeDelta rtt,
                                           TimeDelta one_way_delay) {
  if (rtt.IsFinite()) {
    rtt_stats_.AddSample(rtt.ms());
  }
  if (one_way_delay.IsFinite()) {
    delay_stats_.AddSample(one_way_delay.ms());
  }
}

void L4SMetricsCollector::UpdateLossStats(double loss_fraction) {
  loss_stats_.AddSample(loss_fraction);
}

}  // namespace webrtc