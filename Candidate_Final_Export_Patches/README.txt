Clean requested-final-export patches for five candidate branches

C6 was already patched separately. Each patch here targets exactly one source
file from collector_sources.zip and preserves that candidate's JSON filename.
No congestion-control algorithms, headers, BUILD.gn, or diagnostic tracing change.
The C4/C5 exports of l4s_network_metrics.json are destructor-only and unchanged.

Apply from your repository root after extracting this folder there. Keep the
patch folder untracked; stage only the indicated source file. The following
subshell stops at the first failure. It refuses pre-existing tracked changes.
It creates local commits; pushing is a separate step below.

(
    set -e
    git diff --quiet
    git diff --cached --quiet
    for branch in webrtc_fresh gcc_ce_loss_c2 gcc_ecn_c3 l4s_loss_c4 l4s_prague_c5; do
        git switch "$branch"
        case "$branch" in
            webrtc_fresh|gcc_ce_loss_c2|gcc_ecn_c3)
                source_file=modules/congestion_controller/goog_cc/goog_cc_network_control.cc ;;
            *) source_file=modules/congestion_controller/l4s/l4s_metrics_collector.cc ;;
        esac
        patch_file="Candidate_Final_Export_Patches/${branch}_final_export.patch"
        git apply --check "$patch_file"
        git apply "$patch_file"
        git add -- "$source_file"
        git commit -m "Support requested metrics export without periodic full-history writes"
    done
    git switch l4s_c6
)

If all five commits succeeded, push them:

(
    set -e
    for branch in webrtc_fresh gcc_ce_loss_c2 gcc_ecn_c3 l4s_loss_c4 l4s_prague_c5; do
        git push origin "$branch"
    done
)

If a command fails, stop and inspect its output. Do not rerun the whole loop:
patches already committed on earlier branches cannot be applied twice.

RUNTIME INTEGRATION IS REQUIRED

Set L4S_EXPORT_ON_REQUEST=1 in the environment of EACH remote WebRTC client
process, not just the local FABRIC Python process. Without it, legacy periodic
export remains enabled. All samples and periodic summary samples still collect
in memory in request-only mode. No background writer is introduced.

Before starting a new run, remove stale JSON export request/ack files while the
old client is stopped. Use one client process per working directory.

After the measurement window, publish a fresh UUID hex token (exactly 32 lowercase
hex characters, with no newline) atomically to <json>.export_request in the client
working directory. The next LogPeriodicSummary call performs export on the
collector thread, outside the measurement window. Wait for the SAME token in
<json>.export_ack (which ends with a newline) before stopping that client.

Candidate output files:
  webrtc_fresh    gcc_test_c1.json
  gcc_ce_loss_c2  gcc_test_c2.json
  gcc_ecn_c3     gcc_test_c3.json
  l4s_loss_c4    l4s_test_c4.json
  l4s_prague_c5  l4s_test_c5.json
  l4s_c6        l4s_test_c6.json (already patched separately)

The acknowledgement follows successful JSON writing, fflush, fsync, fclose,
and rename. Never treat a previous acknowledgement as completion for a new run.
On timeout, preserve the live client and report failure rather than killing it
and losing its in-memory data. The existing final-export C6 diagnostic runner
implements the protocol for C6 only; shared suite helpers still need integration
for all primary and competing endpoints.

Validation completed locally:
- Each patch passes git apply --check and produces the intended source exactly.
- All other source content is unchanged.
- Isolated C++17 harness builds with -Wall -Wextra -Werror for each collector.
- Request-only mode does not snapshot without a valid request.
- Legacy mode still exports periodically.
- Token validation, request consumption, matching ack, failed-open/no-ack tested.
- JSON output matches original serializer byte-for-byte for harness samples.
Full WebRTC builds and FABRIC integration are not tested here.
