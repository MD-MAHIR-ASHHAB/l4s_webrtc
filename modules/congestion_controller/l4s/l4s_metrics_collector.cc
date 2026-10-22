#include "modules/congestion_controller/l4s/l4s_network_controller.h"

#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "chrono"

#include <cstddef>
#include <cstdint>
#include <string>
#if defined(__linux__)
#include <sys/syscall.h>
#endif
#include "rtc_base/time_utils.h"


// Temporary diagnostic: one timing record per export, emitted after the work.
// Keep this guard before `metrics` so total time includes snapshot destruction.
namespace {
struct ScopedMetricsExportTiming {
  ScopedMetricsExportTiming(const void* owner, const std::string& filename)
      : owner(owner), filename(filename), begin_us(webrtc::TimeMicros()) {
#if defined(__linux__)
    tid = syscall(SYS_gettid);
#endif
  }
  static double DurationMs(int64_t start, int64_t end) {
    return start > 0 && end >= start ? (end - start) / 1000.0 : -1.0;
  }
  ~ScopedMetricsExportTiming() {
    const int64_t end_us = webrtc::TimeMicros();
    RTC_LOG(LS_INFO) << "METRICS_EXPORT_TRACE"
        << " begin_us=" << begin_us << " end_us=" << end_us
        << " tid=" << tid << " owner=" << owner
        << " filename=" << filename << " outcome=" << outcome
        << " entries=" << entries << " samples=" << samples
        << " bytes=" << bytes
        << " collect_ms=" << DurationMs(begin_us, collect_end_us)
        << " open_ms=" << DurationMs(collect_end_us, open_end_us)
        << " serialize_write_ms=" << DurationMs(open_end_us, serialize_end_us)
        << " fflush_ms=" << DurationMs(serialize_end_us, flush_end_us)
        << " fsync_ms=" << DurationMs(flush_end_us, sync_end_us)
        << " fclose_ms=" << DurationMs(sync_end_us, close_end_us)
        << " rename_ms=" << DurationMs(close_end_us, rename_end_us)
        << " tail_ms=" << DurationMs(rename_end_us, end_us)
        << " total_ms=" << DurationMs(begin_us, end_us);
  }
  const void* owner;
  const std::string& filename;
  const int64_t begin_us;
  long tid = 0;
  const char* outcome = "no_logger";
  size_t entries = 0;
  size_t samples = 0;
  long bytes = -1;
  int64_t collect_end_us = 0;
  int64_t open_end_us = 0;
  int64_t serialize_end_us = 0;
  int64_t flush_end_us = 0;
  int64_t sync_end_us = 0;
  int64_t close_end_us = 0;
  int64_t rename_end_us = 0;
};
}  // namespace


namespace {
// Opt in per process while the experiment shutdown protocol is being validated.
bool RequestOnlyMetricsExportEnabled() {
  static const bool enabled = [] {
    const char* value = std::getenv("L4S_EXPORT_ON_REQUEST");
    const bool result = value && std::string(value) == "1";
    RTC_LOG(LS_INFO) << "METRICS_EXPORT_MODE request_only=" << (result ? 1 : 0);
    return result;
  }();
  return enabled;
}

std::string ReadMetricsExportRequest(const std::string& filename) {
  const std::string request_path = filename + ".export_request";
  FILE* request = fopen(request_path.c_str(), "r");
  if (!request) return {};
  char buffer[64];
  const size_t count = fread(buffer, 1, sizeof(buffer), request);
  const bool failed = ferror(request) != 0;
  fclose(request);
  if (failed || count != 32) return {};
  for (size_t i = 0; i < count; ++i) {
    if (!((buffer[i] >= '0' && buffer[i] <= '9') ||
          (buffer[i] >= 'a' && buffer[i] <= 'f'))) return {};
  }
  return std::string(buffer, count);
}

bool AcknowledgeMetricsExport(const std::string& filename,
                              const std::string& token) {
  const std::string ack_path = filename + ".export_ack";
  const std::string ack_temp = ack_path + ".tmp";
  FILE* ack = fopen(ack_temp.c_str(), "w");
  if (!ack) return false;
  bool ok = fprintf(ack, "%s\n", token.c_str()) > 0;
  if (fflush(ack) != 0) ok = false;
  if (fsync(fileno(ack)) != 0) ok = false;
  if (fclose(ack) != 0) ok = false;
  if (!ok || rename(ack_temp.c_str(), ack_path.c_str()) != 0) {
    remove(ack_temp.c_str());
    return false;
  }
  remove((filename + ".export_request").c_str());
  return true;
}
}  // namespace

namespace webrtc {

L4SMetricsCollector::L4SMetricsCollector(test::MetricsLogger* logger,
                                         const std::string& test_case_name)
    : logger_(logger), test_case_name_(test_case_name) {
  RTC_CHECK(logger_);
  RTC_LOG(LS_INFO) << "L4SMetricsCollector initialized for test case: "
                   << test_case_name_;
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

  // Grab the globally synced UTC Epoch time specifically for the plot data
  int64_t global_utc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()
).count();

  if (target_bitrate.IsFinite()) {
    logger_->LogSingleValueMetric(
        "target_rate_mbps", test_case_name_, target_bitrate.bps() / 1e6,
        webrtc::test::Unit::kUnitless,
        webrtc::test::ImprovementDirection::kBiggerIsBetter,
        {{"timestamp_ms", std::to_string(global_utc_ms)}}); // Swapped here
  }

  DataRate tx_rate = send_rate.value_or(DataRate::Zero());
  logger_->LogSingleValueMetric(
      "send_rate_mbps", test_case_name_, tx_rate.bps() / 1e6,
      webrtc::test::Unit::kUnitless,
      webrtc::test::ImprovementDirection::kBiggerIsBetter,
      {{"timestamp_ms", std::to_string(global_utc_ms)}}); // Swapped here

  logger_->LogSingleValueMetric(
      "actual_rate_mbps", test_case_name_, actual_bitrate.bps() / 1e6,
      webrtc::test::Unit::kUnitless,
      webrtc::test::ImprovementDirection::kBiggerIsBetter,
      {{"timestamp_ms", std::to_string(global_utc_ms)}}); // Swapped here

  DataRate filtered_acked_rate = acked_bitrate.value_or(DataRate::Zero());
  UpdateThroughputStats(filtered_acked_rate);
  logger_->LogSingleValueMetric(
      "acked_rate_mbps", test_case_name_, filtered_acked_rate.bps() / 1e6,
      webrtc::test::Unit::kUnitless,
      webrtc::test::ImprovementDirection::kBiggerIsBetter,
      {{"timestamp_ms", std::to_string(global_utc_ms)}}); // Swapped here
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

  // Grab the globally synced UTC Epoch time specifically for the plot data
  int64_t global_utc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()
).count();


  logger_->LogSingleValueMetric(
      "rtt_ms", test_case_name_, rtt.ms(), webrtc::test::Unit::kMilliseconds,
      webrtc::test::ImprovementDirection::kSmallerIsBetter,
      {{"timestamp_ms", std::to_string(global_utc_ms)}});

  if (one_way_delay.IsFinite()) {
    logger_->LogSingleValueMetric(
        "one_way_delay_ms", test_case_name_, one_way_delay.ms(),
        webrtc::test::Unit::kMilliseconds,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"timestamp_ms", std::to_string(global_utc_ms)}});
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



  // Grab the globally synced UTC Epoch time specifically for the plot data
  int64_t global_utc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();

  logger_->LogSingleValueMetric(
      "packet_loss_fraction", test_case_name_, loss_fraction,
      webrtc::test::Unit::kUnitless,
      webrtc::test::ImprovementDirection::kSmallerIsBetter,
      {{"timestamp_ms", std::to_string(global_utc_ms)}});

  logger_->LogSingleValueMetric(
      "packets_lost_count", test_case_name_, packets_lost,
      webrtc::test::Unit::kCount,
      webrtc::test::ImprovementDirection::kSmallerIsBetter,
      {{"timestamp_ms", std::to_string(global_utc_ms)}});
}

void L4SMetricsCollector::LogCongestionMetrics(Timestamp at_time,
                                               int ce_count,
                                               int ect_count,
                                               double congestion_ratio) {


  // Grab the globally synced UTC Epoch time specifically for the plot data
  int64_t global_utc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();

    
  (void)ect_count;
  logger_->LogSingleValueMetric(
      "congestion_ce_count", test_case_name_, ce_count,
      webrtc::test::Unit::kCount,
      webrtc::test::ImprovementDirection::kSmallerIsBetter,
      {{"timestamp_ms", std::to_string(global_utc_ms)}});

  logger_->LogSingleValueMetric(
      "congestion_ratio", test_case_name_, congestion_ratio,
      webrtc::test::Unit::kUnitless,
      webrtc::test::ImprovementDirection::kSmallerIsBetter,
      {{"timestamp_ms", std::to_string(global_utc_ms)}});
}

void L4SMetricsCollector::LogPeriodicSummary(Timestamp at_time) {
  if (at_time - last_summary_log_ < kSummaryLogInterval) {
    return;
  }

  last_summary_log_ = at_time;

    // Grab the globally synced UTC Epoch time specifically for the plot data
  int64_t global_utc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();


  if (throughput_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric(
        "acked_rate_avg_mbps", test_case_name_, throughput_stats_.GetAverage() / 1e6,
        webrtc::test::Unit::kUnitless,
        webrtc::test::ImprovementDirection::kBiggerIsBetter,
        {{"stat_type", "average"}, {"metric", "acked_rate"},{"timestamp_ms", std::to_string(global_utc_ms)}});
    logger_->LogSingleValueMetric(
        "acked_rate_std_mbps", test_case_name_,
        throughput_stats_.GetStandardDeviation() / 1e6,
        webrtc::test::Unit::kUnitless,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "std_dev"}, {"metric", "acked_rate"},{"timestamp_ms", std::to_string(global_utc_ms)}});
  }

  if (rtt_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric(
        "rtt_avg_ms", test_case_name_, rtt_stats_.GetAverage(),
        webrtc::test::Unit::kMilliseconds,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "average"}, {"metric", "rtt"},{"timestamp_ms", std::to_string(global_utc_ms)}});
    logger_->LogSingleValueMetric(
        "rtt_std_ms", test_case_name_, rtt_stats_.GetStandardDeviation(),
        webrtc::test::Unit::kMilliseconds,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "std_dev"}, {"metric", "rtt"},{"timestamp_ms", std::to_string(global_utc_ms)}});
  }

  if (delay_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric(
        "delay_avg_ms", test_case_name_, delay_stats_.GetAverage(),
        webrtc::test::Unit::kMilliseconds,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "average"}, {"metric", "delay"},{"timestamp_ms", std::to_string(global_utc_ms)}});
    logger_->LogSingleValueMetric(
        "delay_std_ms", test_case_name_, delay_stats_.GetStandardDeviation(),
        webrtc::test::Unit::kMilliseconds,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "std_dev"}, {"metric", "delay"},{"timestamp_ms", std::to_string(global_utc_ms)}});
  }

  if (loss_stats_.NumSamples() > 0) {
    logger_->LogSingleValueMetric(
        "packet_loss_avg_fraction", test_case_name_, loss_stats_.GetAverage(),
        webrtc::test::Unit::kUnitless,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "average"}, {"metric", "packet_loss"},{"timestamp_ms", std::to_string(global_utc_ms)}});
    logger_->LogSingleValueMetric(
        "packet_loss_std_fraction", test_case_name_,
        loss_stats_.GetStandardDeviation(), webrtc::test::Unit::kUnitless,
        webrtc::test::ImprovementDirection::kSmallerIsBetter,
        {{"stat_type", "std_dev"}, {"metric", "packet_loss"},{"timestamp_ms", std::to_string(global_utc_ms)}});
  }

  // Keep recording samples, but avoid full-history copies and disk work
  // during a request-only experiment. Handle the final request on this thread.
  const std::string filename = "l4s_test_c6.json";
  if (!RequestOnlyMetricsExportEnabled() ||
      !ReadMetricsExportRequest(filename).empty()) {
    ExportToJsonFile(filename);
  }
}


L4SMetricsCollector::~L4SMetricsCollector() {
  RTC_LOG(LS_INFO) << "Test complete. Safely exporting metrics to JSON...";
  ExportToJsonFile("l4s_test_c6.json");
}


void L4SMetricsCollector::ExportToJsonFile(const std::string& filename) {
  ScopedMetricsExportTiming export_timing(this, filename);
  if (!logger_) {
    return;
  }

  const std::string export_token = ReadMetricsExportRequest(filename);
  auto metrics = logger_->GetCollectedMetrics();
  export_timing.collect_end_us = TimeMicros();
  export_timing.entries = metrics.size();

  // Write to a temporary file first.
  std::string temp_filename = filename + ".tmp";

  FILE* f = fopen(temp_filename.c_str(), "w");
  export_timing.open_end_us = TimeMicros();
  export_timing.outcome = "open_failed";
  if (!f) {
    RTC_LOG(LS_ERROR)
        << "Failed to open temporary metrics file: "
        << temp_filename;
    return;
  }

  fprintf(f, "[\n");

  for (size_t i = 0; i < metrics.size(); ++i) {
    const auto& m = metrics[i];
    export_timing.samples += m.time_series.samples.size();

    fprintf(f, "  {\n");
    fprintf(f, "    \"name\": \"%s\",\n", m.name.c_str());
    fprintf(f, "    \"samples\": [");

    for (size_t j = 0; j < m.time_series.samples.size(); ++j) {
      const auto& s = m.time_series.samples[j];

      fprintf(
          f,
          "%s{\"timestamp_ms\": %lld, \"value\": %f}",
          (j > 0 ? ", " : ""),
          static_cast<long long>(s.timestamp.ms()),
          s.value);
    }

    fprintf(
        f,
        "]\n"
        "  }%s\n",
        (i + 1 < metrics.size()) ? "," : "");
  }

  fprintf(f, "]\n");

  // Do not acknowledge a snapshot after an earlier buffered write error.
  if (ferror(f) != 0) {
    export_timing.outcome = "write_failed";
    fclose(f);
    return;
  }
  export_timing.serialize_end_us = TimeMicros();
  export_timing.bytes = ftell(f);

  // Flush stdio buffers.
  const int trace_flush_result = fflush(f);
  export_timing.flush_end_us = TimeMicros();
  if (trace_flush_result != 0) {
    export_timing.outcome = "fflush_failed";
    RTC_LOG(LS_ERROR)
        << "fflush failed for temporary metrics file: "
        << temp_filename;
    fclose(f);
    return;
  }

  // Flush kernel buffers to disk.
  const int trace_sync_result = fsync(fileno(f));
  export_timing.sync_end_us = TimeMicros();
  if (trace_sync_result != 0) {
    export_timing.outcome = "fsync_failed";
    RTC_LOG(LS_ERROR)
        << "fsync failed for temporary metrics file: "
        << temp_filename;
    fclose(f);
    return;
  }

  // Close the file.
  const int trace_close_result = fclose(f);
  export_timing.close_end_us = TimeMicros();
  if (trace_close_result != 0) {
    export_timing.outcome = "fclose_failed";
    RTC_LOG(LS_ERROR)
        << "Failed to close temporary metrics file: "
        << temp_filename;
    return;
  }

  // Atomically replace the old file with the new one.
  const int trace_rename_result = rename(temp_filename.c_str(), filename.c_str());
  export_timing.rename_end_us = TimeMicros();
  if (trace_rename_result != 0) {
    export_timing.outcome = "rename_failed";
    RTC_LOG(LS_ERROR)
        << "Failed to rename "
        << temp_filename
        << " -> "
        << filename;

    remove(temp_filename.c_str());
    return;
  }

  export_timing.outcome = "ok";
  // Only acknowledge after the JSON was flushed, synced, closed and renamed.
  if (!export_token.empty()) {
    if (!AcknowledgeMetricsExport(filename, export_token)) {
      export_timing.outcome = "ack_failed";
      RTC_LOG(LS_ERROR) << "METRICS_FINAL_EXPORT acknowledgement failed";
      return;
    }
    RTC_LOG(LS_INFO) << "METRICS_FINAL_EXPORT token=" << export_token
                     << " filename=" << filename;
  }

  // Heartbeat logging.
  static int export_call_count = 0;
  ++export_call_count;

  if (export_call_count % 100 == 0) {
    RTC_LOG(LS_INFO)
        << "[Metrics Heartbeat] JSON export completed successfully "
        << export_call_count
        << " times. Safely written to "
        << filename;
  }
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