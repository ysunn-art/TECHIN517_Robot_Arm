# Quantitative Results — Bimanual Blackjack Dealer

**Team 3 · TECHIN 517**

A bimanual SO-101 rig deals and plays a hand of blackjack end-to-end: it deals the
opening cards, runs the player/dealer decision loop, reads every card with a YOLO
detector, and settles the bet by paying out or collecting a chip with a learned
visuomotor (VLA) policy. We evaluate the system as **three states (phases)**, each
exercising a distinct sub-system, with **10 trials per phase (30 total)**.

| Phase (state) | Sub-system | Trials | Success | Mean time (success) | Std |
|---|---|---|---|---|---|
| **I — Initial Deal** | FK waypoints + YOLO | 10 | **8 / 10 (80%)** | 105.6 s | 2.5 s |
| **II — Decision** | FK loop + YOLO | 10 | **9 / 10 (90%)** | 48.3 s | 20.9 s |
| **III — Settlement** | SmolVLA policy (VLA) | 10 | **10 / 10 (100%)** | 30.0 s | 1.7 s |
| **Overall** | — | 30 | **27 / 30 (90%)** | — | — |

Timing means/standard deviations are computed **over successful trials only**.

---

## Experiment design

### States tested (≥ 3, 10 trials each)
1. **Phase I — Initial Deal** (FK): right arm deals two cards to the player, left
   arm deals two to the dealer (second card face-down as the hole card); the wrist
   cameras + YOLO must read every face-up rank correctly.
2. **Phase II — Decision** (FK): the player hit/stand loop plus the automatic dealer
   (hit ≤ 16, stand ≥ 17), reading each newly drawn card. Round length **scales
   with the number of player hits**.
3. **Phase III — Settlement** (VLA): based on the outcome, the right arm runs a
   trained SmolVLA policy — **pay out a chip** (player wins) or **collect a chip**
   (dealer wins) — driven directly from camera observations.

### Definition of success (binary, per phase)
- **Phase I:** all four dealt cards are physically placed in their zones **and** the
  detector reports the correct rank/value for every face-up card (the hole card
  stays hidden). Failure if any face-up card is misread or misplaced.
- **Phase II:** every hit/stand and dealer draw executes, each newly drawn card is
  read correctly, and the hand reaches a correct terminal total (player/dealer
  win/bust/tie). Failure if any card is misread or the wrong total is reported.
- **Phase III:** the policy grasps the chip and places it in the correct location
  (pay-out → player tray; collect → house tray) within the episode window. Failure
  if the chip is dropped, missed, or placed in the wrong location.

### Termination criteria (when a trial is declared a failure)
- A card is read with the wrong rank/value (detection error).
- A card or chip is dropped or knocked out of its zone.
- The policy episode times out (Phase III, 13 s) without completing the placement.
- Operator safety-stop (none triggered across the 30 trials).

### Starting state
- Bringup launched in JTC mode; both arms calibrated (`gix-follower4` =
  `shoulder_pan -1233`/retrained-cal frame), parked at their home pose.
- Shuffled deck in the right feeder, chips staged in the house tray, cameras
  confirmed via `system_check.py` (right wrist = `/dev/video2`, left wrist =
  `/dev/video0`, overhead RealSense = `/dev/video8`).
- Dashboard running (serves the wrist-cam vision hub + UI).

### Reset procedure (between trials)
1. Collect dealt cards, re-square the deck, return it to the feeder.
2. Return chips to the house tray.
3. Return both arms to home (Phase III also relaunches JTC bringup after the policy
   tears it down and resets the RealSense via the USB `authorized` toggle).
4. Clear the prior hand's state in the UI; confirm cameras streaming before the
   next trial.

---

## Results

### Success rate by condition
| Phase | Success rate | Failures |
|---|---|---|
| I — Initial Deal | 80% (8/10) | 2 |
| II — Decision | 90% (9/10) | 1 |
| III — Settlement | 100% (10/10) | 0 |
| **Overall** | **90% (27/30)** | **3** |

### Timing (successful trials)
| Phase | Mean | Std | Range |
|---|---|---|---|
| I — Initial Deal | 105.6 s (~1:46) | 2.5 s | 102–110 s |
| II — Decision | 48.3 s | 20.9 s | 20–83 s |
| III — Settlement | 30.0 s | 1.7 s | 27–33 s |

Phase II's large spread is expected and meaningful: the round length **scales with
the number of player hits** (0 hits ≈ 20 s, 4 hits ≈ 83 s). Phase III's 30 s breaks
down as **~13 s of policy execution + ~17 s of per-trial init** (bringup teardown,
RealSense reset, model load).

### Failure-mode analysis
All **3 failures share a single root cause: card placement.** When a freshly dealt
card physically **overlaps** a previously placed card, YOLO cannot cleanly separate
the two and misreads the rank (e.g. an **8 read as a 6**). No failures came from the
arms, grippers, the decision logic, or the VLA settlement policy.

| Failure mode | Count | Phase(s) | Cause |
|---|---|---|---|
| Card placement (overlap misread) | 3 | I (×2), II (×1) | Overlapping cards confuse YOLO rank classification |
| Arm / grasp / motion | 0 | — | — |
| Policy (settlement) | 0 | — | — |
| Timeout / safety-stop | 0 | — | — |

Mitigations already in place that reduced this mode: a per-deal **winner-take-all
margin** in the detector (suppresses a look-alike rank unless it dominates by 2×),
a **`right_scan` recovery** pass that repositions the wrist camera and re-scans when
a deal yields no confident read, and a post-settle dwell (`extra_frames`) so a card
is confirmed after the arm clears the frame.

---

## Variance & trade-offs

- **Phases I–II (FK) — speed vs. accuracy.** The FK waypoint phases *can* complete a
  round in under a minute, but pushing the deal motions faster reduces the number of
  camera frames over each card and drops detection reliability. We deliberately
  **traded speed for reliable detection** (0.3 s/waypoint + post-settle dwell), which
  is why the residual failures are all detection-side, not motion-side.
- **Phase III (VLA) — robustness across conditions.** The settlement policy was the
  meaningful variance axis: the 10 Phase III trials were run across **tray positions**
  (left/center/right), **chip counts** (1–3), and **lighting** (dim/normal/bright).
  It succeeded in **10/10**, indicating the learned policy generalizes across those
  conditions where a scripted pick-and-place would need re-tuning.
