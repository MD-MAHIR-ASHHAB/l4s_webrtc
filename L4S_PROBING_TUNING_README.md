# L4S Probing Tuning Plan and Test Matrix

## Objective

Tune L4S probing behavior for CE-sensitive AQMs (DualPI2 style) so probing can discover capacity without repeatedly triggering CE bursts and rate collapses.

## Working Hypothesis

- Bursty probing can exceed the shallow marking threshold and cause CE spikes.
- CE spikes can then dominate fusion and trigger unnecessary reductions.
- Probing must be slower, less aggressive, and less dominant in fusion.
- Recovery probing must avoid repeated startup-style reset loops.

## Plan

1. Reduce probe pressure in steady state and recovery.
2. Tighten probe gating when ECN confidence is high.
3. Reduce probe confidence and fusion weight to prevent single-sample dominance.
4. Prevent repeated reset to startup probing during recovery loops.
5. Validate with focused scenarios (steady CE sensitivity, ALR transitions, recovery, RTT sensitivity).

## Parameter Table (Baseline Profile)

| Parameter | Previous | Baseline | Why |
|---|---:|---:|---|
| `probe_interval` | 5 s | 8 s | Fewer periodic probe bursts |
| `recovery_probe_interval` | 2 s (effective in code path) | 5 s | Reduce CE burst cadence in recovery |
| `probe_multiplier` | 1.5 | 1.2 | Gentler periodic probing |
| `alr_probe_multiplier` | 2.0 | 1.35 | Avoid aggressive ALR spikes |
| `recovery_probe_multiplier` | 2.5 | 1.4 | Safer recovery probing |
| `recovery_alr_probe_multiplier` | 3.0 | 1.7 | Safer recovery+ALR probing |
| `discovery_probe_block_confidence` | 0.99 | 0.92 | Block probes sooner during strong ECN confidence |
| `steady_probe_block_confidence` | 0.95 | 0.88 | Block probes sooner in steady state |
| `probe_confidence_fresh` | 0.95 | 0.75 | Lower immediate influence of fresh probe |
| `probe_confidence_recent` | 0.80 | 0.55 | Lower medium-term probe influence |
| Discovery/recovery `probe_weight` | 0.60 / 0.65 | 0.45 / 0.50 | Reduce probe dominance in fusion |

## Implementation Summary

### Files

- `modules/congestion_controller/l4s/l4s_network_controller.h`
- `modules/congestion_controller/l4s/l4s_network_controller.cc`

### Key logic changes

1. Added explicit probe tuning parameters to `L4SControllerConfig` for interval, multipliers, gating, and confidence.
2. Switched hardcoded thresholds in probing decisions to config-driven values.
3. Reduced probe confidence outputs in `CalculateProbeConfidence`.
4. Reduced probe weighting in discovery/recovery fusion path.
5. Added recovery bootstrap state (`recovery_probe_bootstrapped_`) so recovery does not repeatedly reset probe controller and retrigger startup-like probing on every cycle.

## Test Scenarios

### 1) CE-sensitive steady state

- Setup: DualPI2-like shallow queue, fixed bottleneck, continuous video.
- Metrics:
  - CE ratio over time
  - Target bitrate oscillation amplitude
  - Probe event count per minute
- Pass criteria:
  - Lower CE spike frequency than previous baseline
  - Fewer large target-rate drops immediately after probes

### 2) ALR entry/exit

- Setup: periodic application-limited phases, then return to full load.
- Metrics:
  - Probes around ALR transitions
  - CE marks within 2-5 seconds after ALR end
- Pass criteria:
  - No large CE burst right after ALR exit
  - Smooth return of target rate

### 3) Recovery mode stress

- Setup: induce CE burst, then clean ECT sequence.
- Metrics:
  - Recovery probe cadence
  - Recovery duration
  - Number of restart-style probe sequences
- Pass criteria:
  - Recovery probes no more frequent than configured interval
  - No repeated reset-to-startup behavior during a single recovery episode

### 4) Probe artifact defense

- Setup: receiver-side scheduling jitter to spread probe receive window.
- Metrics:
  - Accepted vs discarded probe results
  - Rate jumps following accepted probes
- Pass criteria:
  - Artifact-like low probe results are filtered
  - No abrupt collapse from one anomalous probe sample

### 5) RTT sensitivity

- Setup: run same load across low RTT and medium RTT profiles.
- Metrics:
  - Probe-triggered CE rates by RTT
  - Convergence time
- Pass criteria:
  - Low RTT profile no longer disproportionately punished by probing

## Suggested A/B Profiles

| Profile | `probe_interval` | `probe_multiplier` | `recovery_probe_interval` | `recovery_probe_multiplier` | Use when |
|---|---:|---:|---:|---:|---|
| Conservative | 10 s | 1.15 | 6 s | 1.30 | Very CE-sensitive, low queue headroom |
| Baseline (current) | 8 s | 1.20 | 5 s | 1.40 | General first-pass validation |
| Aggressive | 6 s | 1.30 | 4 s | 1.60 | Better headroom, faster exploration needed |

## Logging Checklist

Capture these for each run:

- Probe schedule and cluster count per minute
- Probe target rates and accepted measured probe rates
- CE ratio timeline
- Target bitrate timeline
- Acked bitrate timeline
- Recovery mode enter/exit events and duration

## Acceptance Checklist

A change is considered successful when all are true:

1. CE spike frequency decreases vs previous behavior.
2. Probe events do not cause repeated rate collapses.
3. Recovery exits are stable (no immediate re-entry loop).
4. Throughput remains competitive without sustained queue growth.

## Next Iteration Rules

- If CE spikes still occur after probes, increase `probe_interval` and lower `probe_multiplier`.
- If capacity seeking is too slow, adjust `probe_multiplier` upward in small increments (+0.05).
- If probe artifacts still dominate, lower probe confidence and fusion weight further.
- Change one group of knobs at a time and rerun the same scenario set.

## Competing-Flow Guardrails (v2)

These were added after observing recovery-mode re-entry loops and low-rate probe artifacts under competing flow.

| Knob | Value | Purpose |
|---|---:|---|
| `periodic_probe_min_rate` | 400 kbps | Block periodic probes in very low-rate/noisy region |
| `recovery_probe_min_rate` | 600 kbps | Do not run recovery probes when target rate is too low |
| `min_useful_probe_uplift` | 1.05 | Require probe target to be at least +5% above current estimate |
| `recovery_min_clean_packets` | 80 | Raise entry floor for recovery mode |
| `recovery_min_clean_duration` | 2 s | Require sustained clean period before recovery |
| `recovery_reentry_cooldown` | 8 s | Prevent immediate re-entry after timeout/CE exit |

### Expected log signatures after v2

- More frequent lines like:
  - `Skipping recovery probe due to low target rate`
  - `Skipping recovery probe due to insufficient uplift`
  - `Blocking probe due to low target rate`
- Fewer lines like:
  - `Entering recovery mode ... threshold=20`
  - immediate `Exiting recovery mode - timeout` followed by quick re-entry
- Reduced count of probe bursts in sub-500 kbps regimes.

## Conservative-5RTT Control Profile (v3)

This profile implements the agreed balance policy:

1. Conservative probe uplift.
2. Recovery probing on a 5-RTT cadence.
3. Post-probe freeze for 5 RTT with no additive increase.
4. CE hysteresis and floor-based protection unchanged from v2.

### Policy mapping

| Policy | Runtime behavior |
|---|---|
| Recovery probe cadence = 5 RTT | Recovery interval uses RTT-scaled timing (`recovery_probe_rtt_factor=5.0`) with minimum interval guard |
| Freeze for 5 RTT after any probe | Probe hold window is started when probe clusters are emitted |
| No additive increase during hold | Prague additive increase path exits early while hold is active |
| Normal probing remains conservative | `probe_multiplier` and low-rate/uplift guards from v2 are kept |

### Additional v3 knobs

| Knob | Value | Purpose |
|---|---:|---|
| `recovery_probe_rtt_factor` | 5.0 | RTT-scaled recovery probe spacing |
| `probe_hold_rtt_factor` | 5.0 | Hold duration scaling after probe |
| `min_rtt_scaled_interval` | 1 s | Prevents too-fast actions on low RTT paths |

### Expected v3 log signatures

- `Probe hold active, skipping probe scheduling`
- `Prague: Additive increase paused during probe hold window`
- Fewer immediate probe->up->CE->down cycles compared to v2 under competing flow.
