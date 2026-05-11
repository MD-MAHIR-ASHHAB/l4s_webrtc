

Here is a comprehensive, paste-ready README summarizing the battlefield of the last few days.

***

# WebRTC L4S Network Controller: Troubleshooting & Architecture Evolution

This document tracks the evolution of the L4S Network Controller. Integrating a purely mathematical, time-based capacity estimator (Prague) with WebRTC’s aggressive, delay-based video pacing engine revealed several critical edge cases. Below is the chronological progression of the problems we encountered and the architectural fixes applied to synchronize the math with the physical network.

### 1. The Phantom Budget (Runaway Target Rate)
* **The Symptom:** The controller's `target_bps` would grow infinitely (e.g., to 50+ Mbps), even when the video encoder was only pushing 2 Mbps.
* **The Cause:** The Prague estimator was blindly applying Additive Increase (AI) based on time passing. Because the network was idle (the encoder wasn't filling the pipe), no CE marks were generated. Prague assumed the lack of congestion meant infinite capacity.
* **The Fix:** **ALR Integration.** We wired up WebRTC’s `AlrDetector` (Application Limited Region). We updated Prague's logic to strictly freeze the Additive Increase whenever `alr=1`. Growth now only happens when the application is actively filling the current pipe.

### 2. The Catastrophic Rate Crash (MD Collapse)
* **The Symptom:** A single CE mark would cause the `target_bps` to instantly crash from 30 Mbps down to ~1.5 Mbps.
* **The Cause:** When `ProcessEcnFeedback` received a CE mark, it applied the 10-20% Multiplicative Decrease (MD) to the *physical acked throughput* (which was hovering around 2 Mbps) instead of Prague's internal target. 
* **The Fix:** **Proportional MD.** We modified the feedback loop so that Prague applies its reduction factor exclusively to its *own internal estimate*, preventing the catastrophic collapse to the noise floor.

### 3. The Micro-Batch Filter (Ignored CE Marks)
* **The Symptom:** We successfully triggered immediate RTCP feedback upon receiving a CE mark, but the controller ignored it and never entered reduction mode.
* **The Cause:** A legacy WebRTC noise filter inside `ProcessEcnFeedback` was hardcoded to discard any feedback batch containing fewer than 3 packets (`if (batch_total < 3) return;`). 
* **The Fix:** **Filter Removal.** Since we process CE marks over an RTT window, the micro-batch filter was obsolete. Removing it allowed instantaneous 1-packet CE reports to correctly trigger Prague's reduction mode.

### 4. The ALR Deadlock (The 3 Mbps Trap)
* **The Symptom:** The controller became permanently trapped at ~3 Mbps. The video encoder's QP oscillated wildly, but the resolution never adapted up to 1080p.
* **The Cause:** The strict ALR freeze created a deadlock. The encoder wouldn't push more traffic until it had a bigger target, but Prague wouldn't increase the target because the encoder wasn't pushing traffic (`alr=1`). 
* **The Fix:** **The Safe Escalation Handshake.** We introduced a mechanism where clean, successful network Probes act as "scouts." If a probe physically proves the path has more capacity (and contains no CE marks), it safely "nudges" Prague upward by 50% of the difference. This breaks the deadlock, offering the encoder a higher ceiling to grow into without relying on phantom math.

### 5. The 5% ALR Decay Treadmill
* **The Symptom:** Even with the Safe Escalation Handshake, the encoder struggled to adapt its resolution upwards, constantly panicking and raising its QP.
* **The Cause:** While in ALR, we froze Prague's Additive Increase, but we forgot to freeze its *Time-Based Decay*. The controller was automatically slashing the budget by 5% every decay interval simply because time was passing. The encoder was on a treadmill, constantly having its headroom stolen.
* **The Fix:** **Total ALR Freeze.** Updated Prague to halt *all* autonomous movement (both growth and 5% decay) while `alr=1`. This provides a perfectly flat, stable floor for the WebRTC QualityScaler to lower its QP and comfortably adapt up.

### 6. The Integrator Gap (Desync)
* **The Symptom:** Prague's math and the actual Pacer rate were desynchronizing, causing reductions to fail to clear physical congestion.
* **The Cause:** The Fusion Engine occasionally clamped the final target rate (e.g., capping it to a probe limit of 5 Mbps), but Prague's internal state was unaware, continuing to calculate its next steps from 6 Mbps. 
* **The Fix:** **Closed the Loop.** Added synchronization in `OnProcessInterval` to force `prague_estimator_->SetCurrentEstimate(fused_rate)` every tick. The "Driver" (Prague) now accurately steers from the actual enforced limit.



### 7. The Smeared Probe Crash & RTT Math Explosion (The Rubber-Band)
* **The Symptom:** After exiting recovery, the target rate would randomly crash to 1.3 Mbps, and then instantly explode to 44+ Mbps, creating massive bufferbloat and 600ms RTT spikes.
* **The Cause:** 1. **The Crash:** DualPI2 AQM routers natively "smear" high-speed bursts to protect queues. Legacy WebRTC delay-based math saw this smeared probe, assumed the link was 1.3 Mbps, and the Fusion engine blindly trusted it.
    2. **The Explosion:** Raw, instantaneous 1ms/2ms RTT samples were being fed directly into Prague's Additive Increase formula. When RTT is squared in the denominator, a 2ms sample causes the math to instantly calculate tens of megabits of growth.
* **The Fix:** * **The Prague Veto:** Rewrote `GetDiscoveryModeFusedEstimate`. Probes are now only allowed to pull the limit *up*. If a probe reports a low rate but Prague (monitoring ECN) sees no congestion, we veto the probe, ignoring the AQM smearing.
    * **RTT Smoothing:** Implemented an Exponential Moving Average (80/20) for the RTT fed to Prague, alongside a hard 20ms safety floor. This mathematically guarantees the Additive Increase cannot explosively rubber-band.


***8. The 720p Yo-Yo (Bitrate Boundary Trap)
The Symptom: The controller held a perfectly stable target bitrate (e.g., 3.5 Mbps), but the WebRTC Quality Scaler continuously oscillated. It would adapt up to 720p, immediately panic due to a high QP (60-70), adapt down to 480p, drop the QP to 20, and repeat the cycle infinitely.

The Cause: 3.5 Mbps is a "tweener" boundary. It is vast for 480p but extremely tight for 720p. The Safe Escalation Handshake was failing to nudge the ceiling higher because the math required the receiver to measure a probe at > 1.10x the current rate. However, because probes were sent at exactly 1.10x, network jitter and packet headers caused the measured rate to always fall slightly short (e.g., 1.08x), silently failing the handshake.

The Fix: Widen the Handshake Window. Increased the micro-probe multiplier during Application-Limited Regions (ALR) to 1.20x, but lowered the acceptance threshold to 1.05x. This guaranteed that successful probes could easily clear the mathematical threshold, nudging Prague up to 5-6 Mbps and giving the 720p encoder the headroom it needed to stabilize its QP.

9. The Convergence Trap (The 10-Second Heartbeat)
The Symptom: The controller became stuck in an infinite loop, entering and exiting Recovery Mode every 10 seconds, while the target rate permanently flatlined (e.g., at 4.2 Mbps).

The Cause: When the link was perfectly clean, the controller confidently entered Recovery Mode to probe for the ceiling. However, because the application was underutilizing the link (alr=1), the AQM router easily absorbed and smeared the probe burst. The controller saw the smeared probe (~4.0 Mbps), noticed it matched Prague's current frozen rate (~4.2 Mbps), and falsely declared "Prague-Probe Convergence." It exited Recovery immediately, waited out the 10-second cooldown, and repeated the loop.

The Fix: ALR Recovery Block. Added a strict guard (!IsApplicationLimited()) to HandleRecoveryDetection. The controller is now forbidden from probing for the network ceiling if the application is currently the bottleneck, completely killing the infinite heartbeat loop.

10. The Lucky Jitter Lottery (Full-Duplex Asymmetry)
The Symptom: In a symmetric, full-duplex session running identical code, one client would successfully climb to 6 Mbps while the other was permanently trapped at 3.5 Mbps.

The Cause: With a tight 1.10x probe threshold, a client could only pass the Safe Escalation Handshake if microscopic network jitter accidentally compressed its probe packets in transit, causing the receiver to measure an artificially inflated rate (e.g., 1.12x). The "lucky" client escalated; the "smooth" client failed the check and stayed trapped.

The Fix: Solved simultaneously by the wider Handshake Window in Fix #8. By ensuring the physical probe size significantly exceeded the mathematical acceptance floor, we removed network jitter as a variable for capacity escalation.

***11. The Stubborn Manager (ProbeController Bypass)
The Symptom: Logs showed Initiating periodic probe! but were never followed by Probing successful. The network remained "dark," and the target bitrate never moved, leaving the encoder trapped.

The Cause: We were requesting probes through WebRTC’s legacy ProbeController. This component was designed for GCC’s violent 300% spikes and contains hardcoded safety guards (timers, ALR checks, and field trials). It was silently dropping our L4S Micro-Probe requests because they didn't meet legacy GCC criteria.

The Fix: Direct Pacer Injection. We refactored the probing logic to bypass the ProbeController entirely. We implemented a unified CreateCustomProbe helper that manually constructs a ProbeClusterConfig and injects it directly into the NetworkControlUpdate. This guarantees that every L4S probe request physically leaves the machine and reaches the pacer.

12. The 2.5 Mbps Resolution Cage
The Symptom: Even after the network controller successfully proved 20+ Mbps of capacity, the video encoder remained stuck at 720p (921600 pixels) and refused to scale up to 1080p.

The Cause: WebRTC decouples Network BWE from Video Track limits. By default, video tracks are often capped at a max_bitrate_bps of 2.5 Mbps. WebRTC’s internal lookup tables dictate that 1080p is "illegal" at 2.5 Mbps to prevent poor visual quality. The encoder was ignoring the 20 Mbps "speed limit" because the "car" (the track configuration) was electronically limited to 2.5 Mbps.

The Fix: Application-Layer Unleashing. To cross the 1080p threshold, the max_bitrate_bps must be explicitly raised in the RtpParameters of the video sender, or via the WebRTC-VideoRateControl field trial. This allows the encoder to see the large L4S budget and finally allocate the ~5 Mbps required for 1080p.

13. The Weaponized Prober (Phantom Budget 2.0)
The Symptom: With the new probing fix, the target_bps began climbing infinitely (30, 40, 50 Mbps) while the encoder was sitting idle at 2 Mbps.

The Cause: The "Safe Escalation Handshake" was too successful. Since the network was empty, every probe cleared the 1.05x threshold, nudging Prague upward indefinitely. This recreated the "Phantom Budget" problem, creating a latent "latency bomb" if the encoder were to suddenly surge.

The Fix: The Headroom Cap. Introduced EncoderNeedsMoreHeadroom(). This logic stops all probing (Periodic and Recovery) once the target_bps provides 2.0x (200%) of the application's actual current throughput. The controller now provides a stable "porch" for the encoder to grow into but refuses to drift into "fantasy land" bitrates that the application isn't even close to using.

14. Reverse-Path Creep (Asymmetric Bufferbloat)
The Symptom: In full-duplex runs, the RTT would gradually creep from 170ms up to 210ms or higher as the session continued, despite the forward path being clean.

The Cause: This is "Reverse-Path Queuing." As the other client successfully increases its bitrate, it fills the return path. The tiny RTCP feedback packets (needed to measure RTT) get stuck in the return queue behind the other client's video frames.

The Result: This confirmed that our RTT Smoothing (EMA) and 20ms Floor are vital; they allow the controller to remain stable even when return-path congestion artificially inflates the RTT measurements.


***15. The Hydraulic Shock (Unconstrained Encoder)
The Symptom: After raising the max_bitrate_bps to 30 Mbps, the system became violently erratic. CE marks were constant, and the session stayed in a state of permanent congestion.

The Cause: Removing the bitrate cap allowed the video engine to attempt massive quality jumps. Instantaneous 1080p keyframes would "shock" the network, instantly saturating the DualPI2 queue. Furthermore, high max limits often trigger WebRTC's internal padding mechanisms, which blast the network with non-video data to "test" the 30 Mbps limit, overwhelming the L4S controller's ability to react smoothly.

The Lesson: In L4S, bandwidth must be "pulled" by the network controller, not "pushed" by the encoder. Conservative application-layer caps (e.g., 5-8 Mbps) are necessary to prevent the video engine from over-reacting to transient capacity increases.


Here is a fully expanded, comprehensive update for your `README.md`. It breaks down every single major architectural flaw, mathematical bug, and "mini-problem" we solved together to get the L4S controller testbed-ready.

---

## Comprehensive Architectural Upgrades & Fixes

### 1. The "Vacuum Cleaner" Reality Anchor Bug
**The Problem:** The Reality Anchor was designed to snap the target rate down to the physical throughput (`Actual * 1.1`) if the target floated too high. However, it was running unconditionally every time the throughput window expired. Even on perfectly clean networks without congestion, it was vacuuming up all the additive growth and resetting the rate downward, artificially starving the video encoder.
**The Solution:** Wrapped the Reality Anchor inside an `if (window_ce_count_ > 0)` check. It now acts strictly as a pre-processor that only triggers when physical congestion is actively detected.

### 2. The RTT Gate State Lockout
**The Problem:** Even when the Reality Anchor correctly calculated a snapped target rate (e.g., from 4986k down to 2224k), the underlying Prague estimator was ignoring it. Prague's internal RTT gate saw that a congestion cut had recently occurred and silently rejected the new input, keeping the internal state stuck at the bloated ceiling.
**The Solution:** Implemented a forced state override (`prague_estimator_->SetCurrentEstimate()`) immediately after calculating the snapped rate. This forces Prague to adopt the physical reality *before* the RTT gates evaluate the next mathematical cut.

### 3. The Stale Variable Reduction Bug
**The Problem:** Inside the `UpdateFromCongestionSignal` reduction block, the multiplicative decrease math was multiplying the reduction factor against `congestion_based_estimate_` (the old, un-snapped internal state) instead of the securely anchored `current_rate`. This mathematically bypassed the Reality Anchor completely.
**The Solution:** Refactored the math to strictly execute against `current_rate`: `DataRate reduced = std::max(current_rate * reduction_factor, min_target_rate_);` ensuring that all cuts are grounded in the actual physical throughput.

### 4. The RTCP RTT Race Condition
**The Problem:** The controller tracked latency using an Exponential Moving Average (EMA) when processing high-resolution Transport Feedback. However, legacy WebRTC RTCP Receiver Reports trigger asynchronously via `OnRoundTripTimeUpdate`. This method lacked the EMA logic and was blindly overwriting `last_rtt_` with raw, noisy data, destabilizing the controller's time-scaled math.
**The Solution:** Unified the RTT architecture. Ported the identical EMA smoothing equation `(last_rtt_ * 0.8) + (msg.round_trip_time * 0.2)` into the RTCP update method and enforced a strict 20ms safety floor to prevent division-by-zero errors in the Additive Increase phase.

### 5. Pacer Padding Starvation (The Ghost Probes)
**The Problem:** During ALR (Application Limited Region) periods, WebRTC attempts to probe the network to find higher capacity. However, the L4S controller explicitly hardcoded the hardware WebRTC Pacer's padding limit to zero (`pad_window = DataSize::Zero()`). If Prague set a target of 5 Mbps but the encoder pushed 1 Mbps, the physical network never saw the 5 Mbps probe because the hardware was forbidden from generating padding.
**The Solution:** Re-enabled dynamic padding by capturing `max_padding_rate_` from `OnStreamsConfig`. The Pacer is now allowed to pad up to the encoder's limit, strictly capped by the current L4S target rate, ensuring physical probes actually reach the wire.

### 6. The Pacing Multiplier Micro-Burst Trap
**The Problem:** Legacy Google Congestion Control (GCC) uses a `2.5x` pacing multiplier, intentionally blasting video frames into the network at 250% speed to clear the encoder queue. In an L4S shallow-buffer network, a 2.5x micro-burst instantly overflows the threshold, generating a wall of CE marks and forcing Prague into a perpetual cycle of self-inflicted congestion cuts.
**The Solution:** Hardened the pacing configuration to strictly use a `1.0x` multiplier (`current_rate * pacer_config->time_window`). This enforces perfectly smooth packet pacing, allowing the controller to cleanly read the network's steady-state CE signals.



### 7. The Encoder Protection Trap (The 7-RTT Staircase)
**The Problem:** To protect the video encoder from crashing, Prague was mathematically forbidden from cutting the bitrate by more than 20% in a single step (`std::max(reduction_factor, 0.80)`). However, during a massive 50x network collapse (e.g., 100 Mbps dropping to 2 Mbps), it took 7 full round-trips for the 20% math to scale the target down to safety. This 1-second delay completely shattered the drop-tail buffer, causing severe packet loss spikes.
**The Solution:** Implemented a Delay-Gradient Circuit Breaker. By tracking a `baseline_rtt_`, the controller detects when the queue is rapidly expanding. If latency spikes beyond safety limits, the controller temporarily overrides the 20% protection floor, allowing deep, violent queue-draining cuts.

### 8. Full-Duplex Reverse-Path Starvation
**The Problem:** In full-duplex bottlenecks, the peer's media stream suffocates the reverse queue, delaying the RTCP feedback packets carrying the CE marks back to the controller. This "blind spot" caused the controller to react hundreds of milliseconds too late to severe congestion.
**The Solution:** The integration of the Delay-Gradient Circuit Breaker natively solves this. By the time the delayed CE marks arrive, the RTT has already spiked to 300ms+. The Circuit Breaker sees the bloated RTT and instantly applies a massive compensation cut, recovering the queue immediately.

### 9. Continuous Circuit Breaker vs. Hardcoded Tiers
**The Problem:** Initial solutions to the full-duplex collapse relied on hardcoded latency tiers (e.g., a 50% cut at 75ms, an 80% cut at 150ms). These "magic numbers" are brittle and fail to adapt to networks with inherently different base latencies or jitter profiles.
**The Solution:** Replaced the discrete tiers with a **Continuous Mathematical Penalty Function**. 
The controller now calculates a fluid `penalty_factor` based on the exact proportion of the RTT bloat: `dynamic_floor = 0.80 - (0.60 * penalty_factor)`. As latency increases, the maximum allowable cut smoothly slides from 20% down to 80%, perfectly mirroring the physical severity of the queue buildup without relying on arbitrary step thresholds.



---