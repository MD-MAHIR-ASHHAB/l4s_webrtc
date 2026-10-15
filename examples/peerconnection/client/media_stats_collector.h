// Application-level video statistics collector for the supplied WebRTC API.
// Add this header beside conductor.h. No encoder/controller edits required.
#ifndef EXAMPLES_PEERCONNECTION_CLIENT_MEDIA_STATS_COLLECTOR_H_
#define EXAMPLES_PEERCONNECTION_CLIENT_MEDIA_STATS_COLLECTOR_H_

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "api/make_ref_counted.h"
#include "api/peer_connection_interface.h"
#include "api/scoped_refptr.h"
#include "api/stats/rtc_stats_collector_callback.h"
#include "api/stats/rtc_stats_report.h"
#include "api/stats/rtcstats_objects.h"
#include "api/units/time_delta.h"
#include "rtc_base/logging.h"
#include "rtc_base/thread.h"

class MediaStatsCollector {
 public:
  // signaling_thread must be the live thread used by this PeerConnection.
  // Destroy/Stop this collector BEFORE destroying that thread or connection.
  static std::unique_ptr<MediaStatsCollector> Start(
      webrtc::scoped_refptr<webrtc::PeerConnectionInterface> peer_connection,
      webrtc::Thread* signaling_thread) {
    if (!peer_connection || !signaling_thread) {
      RTC_LOG(LS_ERROR) << "Media stats: missing PeerConnection/signaling thread";
      return nullptr;
    }
    auto state = std::make_shared<State>();
    state->connection = std::move(peer_connection);
    state->log = ProcessLog();
    if (!state->log->IsOpen()) {
      return nullptr;
    }
    state->session_id = state->log->NewSession();
    state->log->Write(Event("session_start", state->session_id));
    signaling_thread->PostTask([state, signaling_thread] {
      Poll(state, signaling_thread);
    });
    return std::unique_ptr<MediaStatsCollector>(new MediaStatsCollector(state));
  }

  ~MediaStatsCollector() { Stop(); }
  MediaStatsCollector(const MediaStatsCollector&) = delete;
  MediaStatsCollector& operator=(const MediaStatsCollector&) = delete;

  void Stop() {
    if (!state_) {
      return;
    }
    // Do not join a worker thread or wait for a stats callback here.
    // In-flight callbacks own State, never Conductor, and discard late reports.
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (!state_->active) {
      return;
    }
    state_->active = false;
    state_->connection = nullptr;
    state_->log->Write(Event("session_stop", state_->session_id));
  }

 private:
  static int64_t UtcMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  static int64_t MonotonicMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  // One output stream per application process. Opening in "w" clears an older
  // process's output ONCE, while reconnects append new sessions to the same file.
  // As in your experiments, run only one peerconnection process per output path.
  class Log {
   public:
    Log() {
      const char* configured_path = std::getenv("MEDIA_STATS_FILE");
      const char* path = configured_path && configured_path[0]
                             ? configured_path
                             : "media_stats.jsonl";
      file_ = std::fopen(path, "w");
      if (!file_) {
        RTC_LOG(LS_ERROR) << "Cannot open media statistics file: " << path;
      }
    }
    ~Log() {
      if (file_) {
        std::fclose(file_);
      }
    }
    bool IsOpen() const { return file_ != nullptr; }
    uint64_t NewSession() {
      std::lock_guard<std::mutex> lock(mutex_);
      return ++session_number_;
    }
    void Write(const std::string& json) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!file_) {
        return;
      }
      const bool failed =
          std::fwrite(json.data(), 1, json.size(), file_) != json.size() ||
          std::fputc('\n', file_) == EOF || std::fflush(file_) != 0;
      if (failed) {
        RTC_LOG(LS_ERROR) << "Writing media statistics failed";
      }
    }

   private:
    FILE* file_ = nullptr;
    std::mutex mutex_;
    uint64_t session_number_ = 0;
  };

  struct State {
    std::mutex mutex;
    bool active = true;
    bool in_flight = false;
    uint64_t sequence = 0;
    uint64_t session_id = 0;
    webrtc::scoped_refptr<webrtc::PeerConnectionInterface> connection;
    std::shared_ptr<Log> log;
  };

  static std::shared_ptr<Log> ProcessLog() {
    static const auto log = std::make_shared<Log>();
    return log;
  }

  static std::string Event(const char* event, uint64_t session_id) {
    return "{\"schema_version\":1,\"event\":\"" + std::string(event) +
           "\",\"session_id\":" + std::to_string(session_id) +
           ",\"timestamp_ms\":" + std::to_string(UtcMs()) +
           ",\"monotonic_ms\":" + std::to_string(MonotonicMs()) + "}";
  }

  static bool IsVideo(const std::optional<std::string>& kind) {
    return kind.has_value() && *kind == "video";
  }

  static bool Keep(const webrtc::RTCStats& stats) {
    const std::string type = stats.type();
    if (type == "inbound-rtp") {
      return IsVideo(stats.cast_to<webrtc::RTCInboundRtpStreamStats>().kind);
    }
    if (type == "outbound-rtp") {
      return IsVideo(stats.cast_to<webrtc::RTCOutboundRtpStreamStats>().kind);
    }
    if (type == "media-source") {
      return IsVideo(stats.cast_to<webrtc::RTCMediaSourceStats>().kind);
    }
    if (type == "codec") {
      const auto& mime = stats.cast_to<webrtc::RTCCodecStats>().mime_type;
      return mime.has_value() && mime->compare(0, 6, "video/") == 0;
    }
    return false;
  }

  // make_ref_counted supplies the callback's AddRef/Release implementation.
  // This class must not be marked final.
  class Callback : public webrtc::RTCStatsCollectorCallback {
   public:
    Callback(std::shared_ptr<State> state, uint64_t sequence,
             int64_t request_utc_ms, int64_t request_monotonic_ms)
        : state_(std::move(state)),
          sequence_(sequence),
          request_utc_ms_(request_utc_ms),
          request_monotonic_ms_(request_monotonic_ms) {}

    void OnStatsDelivered(
        const webrtc::scoped_refptr<const webrtc::RTCStatsReport>& report)
        override {
      const int64_t callback_utc_ms = UtcMs();
      const int64_t callback_monotonic_ms = MonotonicMs();
      std::lock_guard<std::mutex> lock(state_->mutex);
      state_->in_flight = false;
      if (!state_->active || !report) {
        return;
      }
      std::string objects = "[";
      bool first = true;
      for (const auto& stats : *report) {
        if (!Keep(stats)) {
          continue;
        }
        if (!first) {
          objects += ",";
        }
        first = false;
        // Native serialization retains IDs, SSRC, dimensions, raw counters,
        // optional fields, and the original per-stat timestamp and units.
        objects += stats.ToJson();
      }
      objects += "]";
      state_->log->Write(
          "{\"schema_version\":1,\"event\":\"sample\",\"session_id\":" +
          std::to_string(state_->session_id) + ",\"sequence\":" +
          std::to_string(sequence_) + ",\"timestamp_ms\":" +
          std::to_string(callback_utc_ms) + ",\"request_utc_ms\":" +
          std::to_string(request_utc_ms_) + ",\"request_monotonic_ms\":" +
          std::to_string(request_monotonic_ms_) +
          ",\"callback_monotonic_ms\":" +
          std::to_string(callback_monotonic_ms) + ",\"stats\":" + objects + "}");
    }

   protected:
    ~Callback() override = default;

   private:
    std::shared_ptr<State> state_;
    uint64_t sequence_;
    int64_t request_utc_ms_;
    int64_t request_monotonic_ms_;
  };

  static void Poll(const std::shared_ptr<State>& state,
                   webrtc::Thread* signaling_thread) {
    webrtc::scoped_refptr<webrtc::PeerConnectionInterface> connection;
    uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->active) {
        return;
      }
      if (!state->in_flight) {
        state->in_flight = true;
        connection = state->connection;
        sequence = ++state->sequence;
      }
    }
    if (connection) {
      auto callback = webrtc::make_ref_counted<Callback>(
          state, sequence, UtcMs(), MonotonicMs());
      // Your supplied API takes a raw pointer and retains the ref-counted callback.
      connection->GetStats(callback.get());
    }
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->active) {
        return;
      }
    }
    // This task is executing on signaling_thread, which must outlive the collector.
    // A delayed task captures State only, not Conductor or a PeerConnection.
    signaling_thread->PostDelayedTask(
        [state, signaling_thread] { Poll(state, signaling_thread); },
        webrtc::TimeDelta::Millis(1000));
  }

  explicit MediaStatsCollector(std::shared_ptr<State> state)
      : state_(std::move(state)) {}
  std::shared_ptr<State> state_;
};

#endif  // EXAMPLES_PEERCONNECTION_CLIENT_MEDIA_STATS_COLLECTOR_H_
