# TODO / Future Work

## HIGH PRIORITY — QCar 2 lane filtering + overtake abort (deadline 2026-08-30)

**Added 2026-08-28** after a supervisor meeting. Two working days. Everything
in this section happens in **`~/ros2_ws_sami`** on QCar 2 (`ros2_ws_izhan` is
the untouched clean fallback — verified byte-different, keep it that way).

**Supervisor's ask, verbatim intent:** implement the *lane filtering* scenario
from the IDEAM paper on QCar 2, and have it check ROSbot 3's location from the
V2V link at the same time. He demonstrated the current failure himself: he
stood in the second lane while QCar 2 was in the overtaking state, and the car
**stopped** rather than returning to its original lane, even though that lane
was clear.

**Strategy decision: extend the existing stack, do NOT reimplement IDEAM from
scratch.** Rationale in "Explicitly out of scope" below. A from-scratch
reconstruction is a 2-3 week job; the existing stack already carries weeks of
hard-won tuning (Issues 14/15/16, `v_curve_min`, track-error limiting, startup
ramp) plus all of the 2026-08-27/28 fixes, and throwing that away two days
before a deadline is the wrong trade.

---

### BLOCKERS — resolve before building anything on top of the V2V gap

- [x] **B1. The V2V gap number is wrong by roughly 2x, cause unknown.**
      Measured on hardware 2026-08-28: dashboard/`/v2v/gap` read **1.051 m**
      while a tape measure between chassis read **57 cm** (~1.84x). This is
      *the* blocker: the supervisor explicitly wants the V2V location check,
      and gap-magnitude filtering (D1 below) is built directly on this number.
      A filter fed a 2x-wrong distance makes confidently wrong decisions.
      **Root cause found and fixed 2026-08-31**: `signed_gap_along()` reports
      `base_link`-to-`base_link` distance, but real-world measurements are
      bumper-to-bumper. Both robots' `base_link` sits at their chassis
      center, so the gap was inflated by each vehicle's own half-length —
      `qcar2_overhang_m` (0.2125) + `rosbot3_overhang_m` (0.0985) = ~31cm.
      `v2v_receiver_node.py` now subtracts this (sign preserved). Live
      test: a 25cm physical gap that previously read `-0.481` now reads
      `-0.049` — much closer to real life, per direct comparison. The
      remaining ~20cm is the separate `frame_tx/ty/tyaw` calibration error
      (see the P1-P5 landmark section in `qcar2_side/memory.md` and this
      repo's `memory.md` — a 5-point least-squares fit gave 19.8cm RMS,
      not worth writing over the current ICP transform).
      Already ruled out, do not re-investigate:
    - **Not a curvature/arc-length artifact.** Verified the reference path is
      effectively straight between the two indices involved — chord distance
      and along-path distance agreed to 3 decimal places (1.051 m both ways).
    - **Not a stale-recording mismatch.** ROSbot 3's trajectory (Aug 25) and
      the calibration (Aug 26) are chronologically consistent; QCar 2's
      `track_map_new.pbstream` / `track_run_cartographer_final.npy` (Aug 17-18)
      were untouched by the 2026-08-27/28 cartographer retuning, which only
      changed matching thresholds, not the frozen map.
    - **Not QCar 2's own localization.** User confirmed live that QCar 2's
      position on its own dashboard track matches where it physically is.
    - **Next diagnostic step (~15 min, do this first):** does ROSbot 3's own
      dashboard show *its* position matching where it physically is? That one
      check isolates ROSbot 3's AMCL from the `frame_tx/ty/tyaw` calibration
      (currently `1.3814 / 0.4986 / -3.030697`, ICP-derived 2026-08-26 at
      4.7 cm RMS — see `memory/v2v-map-frame-calibration.md`). Note 4.7 cm RMS
      alone cannot explain a 48 cm error, so if ROSbot 3's own position checks
      out too, the calibration becomes the prime remaining suspect and should
      be re-derived by loop-to-loop ICP.

- [x] **B2. Along-path gap is structurally blind to lateral offset — known,
      affects design not correctness.** A *second*, separate discrepancy was
      seen during an actual pass: `/v2v/gap` 0.661 m vs 19 cm physical
      (~3.48x). This one **is** explained: `signed_gap_along()` measures
      progress along the centerline only, so during a 0.70 m lateral swerve
      two vehicles can be nearly touching while the along-path number still
      looks comfortable. Do **not** use `/v2v/gap` as a proximity check during
      the pass itself — use the LiDAR side sectors (`right_min`/`right_count`)
      for lateral clearance. Different mechanism from B1; fixing B1 will not
      fix this.
      **Verified 2026-08-31, no code change needed**: audited every use of
      `/v2v/gap` in `lidar_overtake_node.py` and `overtake_state_machine.py`.
      Lateral-clearance decisions during `OVERTAKE_LEFT`/`RETURN_RIGHT`
      already come entirely from LiDAR side sectors (`left_clear`/
      `right_clear`/`right_count` in `overtake_state_machine.py`) — `/v2v/gap`
      never feeds them. The only place `/v2v/gap` is used
      (`commit_check_distance` in `lidar_overtake_node.py`) is a different,
      legitimate purpose: deciding whether there's enough *longitudinal*
      runway to commit to *starting* an overtake, not a lateral safety check
      during the pass itself. The codebase already follows this item's own
      guidance.

- [x] **B3. `path_mpc` has no staleness watchdog on `/v2v/follow_speed_cap`.**
      If `v2v_receiver` dies (it crashed once on 2026-08-28), the last cap
      value freezes in place indefinitely instead of falling back to "no
      restriction". The crash itself is fixed, but the missing watchdog is
      not. Low effort, worth doing before relying on V2V for decisions.
      **DONE 2026-08-31** in `path_mpc_node.py`: `/v2v/follow_speed_cap` is
      `Float32` (no header/stamp), so freshness is tracked the same way
      `v2v_receiver_node` judges its own UDP link — arrival time on a
      monotonic clock, recorded in `v2v_follow_speed_cap_callback`. If more
      than `v2v_follow_speed_cap_stale_sec` (1.0s) has passed since the last
      message, `update_v2v_follow_cap_filter()` now treats the cap as -1.0
      (no restriction) instead of reusing the frozen last value. Built and
      deployed to the car; `path_mpc` restarted cleanly, `Localization
      stable. MPC enabled.` confirmed. Not yet tested against an actual
      `v2v_receiver` kill on hardware.

---

### THE BUG THE SUPERVISOR DEMOED (highest value, ~10 lines)

- [x] **A1. Add an abort-to-return path to the overtake state machine.**
      **WRITTEN + BUILT 2026-08-28 into `ros2_ws_sami` — hardware-tested
      2026-08-30 with QCar 2 alone (ROSbot 3 not running) and confirmed
      working; independently corroborated live, the same day's driving
      log showed `abort_return=True` latched in `lidar_overtake`'s
      output. HARDWARE-TESTED 2026-08-31 with ROSbot 3 also running and
      confirmed working** — the supervisor's demoed scenario (a real
      two-vehicle overtake) has now been exercised with both vehicles
      live.
      Overtake lane blocked + original lane empty (`right_clear and
      right_count == 0`) now transitions to `RETURN` instead of `WAIT`;
      `WAIT` is reserved for when both lanes are blocked. Deliberately not
      gated on `min_overtake_steps` or `sufficient_lead_to_return` (an abort
      is an escape, not a completed pass), nor on the 3-tick
      `confirmed_right_clear` counter — falling through to `WAIT` would trap
      the car, since `WAIT` cannot re-enter `OVERTAKE` while the left lane
      is still blocked. Added `last_return_was_abort` + an `abort_return=`
      field in the `lidar_overtake` log so an abort is distinguishable from
      a normal completion. Verified offline against the supervisor's exact
      scenario (4 cases incl. a regression that normal passes still work).
      **To test:** stand in the overtake lane mid-pass — expect
      `state=LC_RIGHT` (renamed from `RETURN_RIGHT` by D3, 2026-08-31),
      `offset=0.00`, `motion=True`, `abort_return=True` instead of a stop.
      In `src/qcar_science_night_pkg/qcar_science_night_pkg/overtake_state_machine.py`,
      the `OVERTAKE` branch currently has exactly one escape when the overtake
      lane becomes blocked:

      ```python
      if not status.left_clear and status.left_count > 0:
          self.state = self.WAIT
          return OvertakeDecision(self.state, 999.0, False)   # full stop
      ```

      There is no path anywhere in the machine that goes "overtake lane
      blocked + original lane clear -> return to original lane". `WAIT` is a
      hard stop (offset 999.0 sentinel, `motion_enabled=False`), regardless of
      how clear the lane it came from is. **Confirmed pre-existing** in the
      upstream `izhan` branch — this is not a regression from the 2026-08-27/28
      work.
    - Fix: overtake lane blocked **and** original lane clear -> `RETURN`.
      Reserve `WAIT` for when *both* lanes are blocked.
    - Reuse the existing `right_clear` / `right_count` confirm-counters for
      the "original lane is clear" test rather than inventing a new signal.
    - **Framing worth using in the write-up:** the paper's whole argument is
      that a passive safety mode is inferior to active probing — the ego
      should *"proactively explore opportunities for merging"* rather than
      *"merely adopting a passive safety mode in response to the environment"*
      (Sec. V-A-2). The current `WAIT_FOR_CLEAR` hard-stop **is** the passive
      baseline the paper argues against. The supervisor demonstrated the
      paper's own motivating problem on our robot — say so explicitly.

---

### WHAT MAPS CLEANLY FROM IDEAM (build these)

| Paper concept | Our stack today | Work needed |
|---|---|---|
| **LK** (lane-keeping, Eq. 31) | `LK` (renamed from `DRIVE`, D3 done 2026-08-31) | done |
| **LP** (lane-probing, Eq. 32) | *missing* — we jump straight to full commit or full stop | the real gap, see D2 |
| **LC** (lane-changing, Eq. 33) | `LC_LEFT` / `LC_RIGHT` (renamed from `OVERTAKE_LEFT`/`RETURN_RIGHT`, D3 done 2026-08-31) | done |
| **Gap Magnitude Judge** (Alg. 2 line 6 — filter candidate gaps lacking space) | partial: `enough_distance_to_overtake` | extend with V2V, see D1 |

- [ ] **D1. Gap-magnitude filtering using the V2V location (the supervisor's
      "lane filtering").** Make the candidate-gap evaluation an explicit,
      named filter step rather than the current inline boolean: evaluate the
      target-lane gap, filter it out if it lacks sufficient space, and let the
      decision fall through to an alternative (probe / abort / return) instead
      of collapsing to a stop. Feed it the V2V gap **and** the LiDAR side
      sectors, per B2. Blocked by B1.
- [ ] **D2. Explicit LP (lane-probing) state between `DRIVE` and
      `OVERTAKE_LEFT`.** Probe alongside, evaluate the gap, then commit or
      abort — instead of today's binary commit-or-stop. Note the follow law
      (`cap = leader_speed + gain*(gap - target)`) is already a crude LP: it
      holds station behind ROSbot 3. What's missing is the explicit state, the
      filtering, and the abort path (A1).
- [x] **D3. Rename states to LK / LP / LC** so the implementation reads as an
      IDEAM reconstruction rather than an ad-hoc FSM. Cheap, and it is what
      makes the deliverable legible to the supervisor.
      **DONE 2026-08-31, partial by design** — no `LP` yet since D2 (the
      actual lane-probing state) isn't built. Renamed in
      `overtake_state_machine.py`, `path_mpc_node.py`, and
      `lidar_overtake_node.py`: `DRIVE`→`LK`, `OVERTAKE_LEFT`→`LC_LEFT`,
      `RETURN_RIGHT`→`LC_RIGHT`. `WAIT_FOR_CLEAR`/`EMERGENCY_STOP` kept
      their own names (no paper analog). Full grep across the repo
      confirmed the only other file referencing these strings
      (`qcar_lane_pkg/mpc_controller_2.py`) is dead/unlaunched legacy code
      — left untouched, out of scope.
      **Bonus find while auditing the rename's blast radius**: `path_mpc_
      node.py` had three checks against `"OBSTACLE_SLOW"`, a state that
      turned out to be **dead code** — nothing in `overtake_state_machine.py`
      or `lidar_overtake_node.py` has ever published it (confirmed via a
      full grep of every `/drive_state` producer). One of those checks was
      the exact gate my B3 fix (same session) had just been built on top
      of; traced through and confirmed it still behaved correctly by
      accident (the dead condition was equivalent to "never", same as the
      intended "not during LC_LEFT/LC_RIGHT"). Removed the dead branch and
      re-pointed that gate at the real state (`LK`) instead, which is
      behavior-preserving (redundant-but-harmless during `LK`, since
      `apply_state_machine_reference()` doesn't touch `target_v` there) but
      makes the guard actually mean what its comment says, instead of
      being a no-op that happened to have the right effect.
      Built and deployed to the car; not yet hardware-tested with an actual
      overtake since the rename (stack was up and stationary at the time).

---

### TWO-DAY PLAN

Estimated ~1.5 days of work, leaving real buffer. Hardware integration is
where time actually goes — 2026-08-27/28 is the evidence.

- [ ] **Step 0 — B1 diagnostic (~15 min).** Check ROSbot 3's own dashboard
      position against physical. Everything else that touches the gap number
      depends on this.
- [x] **Step 1 — A1 abort-to-return (~1-2 h incl. hardware test).**
      Fixes the exact scenario the supervisor demonstrated. QCar-2-alone
      hardware test passed 2026-08-30; full test with ROSbot 3 running
      passed 2026-08-31 (see A1 above). Was started first, ahead of B1,
      since it never depended on gap accuracy.
- [ ] **Step 2 — D1 gap-magnitude filtering (~3-4 h).** Blocked by B1.
- [ ] **Step 3 — D2 explicit LP state (~4-5 h).**
- [ ] **Step 4 — Hardware integration + tuning (~4-6 h).** Budget generously.
- [ ] **Step 5 — Write-up + before/after video of the supervisor's exact
      scenario (~2 h).** Map our LK/LP/LC states to the paper's Eqs. 31-33,
      state the disclosed deviations (below) plainly rather than hiding them.

**Deployment reminder for every step:** `lidar_overtake` and `v2v_receiver`
are started once in `launch_stack()` and are **not** restarted by the script's
relaunch (`l`) — a `colcon build` alone does nothing to the running process.
Kill and relaunch manually (`pkill -2 -f "qcar_science_night_pkg/<node>"`,
then `ros2 run ...`), which can be done live without disrupting driving.
Non-interactive SSH also needs `source install/setup.bash` on top of
`/opt/ros/humble/setup.bash`.

---

### EXPLICITLY OUT OF SCOPE (disclose as deviations, do not attempt in 2 days)

- **LSGM / C-DFS graph search (Alg. 1-2, Fig. 4-5).** Needs six vehicle
  groups (L1,L2,C1,C2,R1,R2) in dense multi-lane traffic. We have *one* other
  robot. `LevelCheck` and `RiskAssessment` over a two-node graph is theater,
  not function. (Note this contradicts the more optimistic 2-lane/4-node
  assessment in the older "PAPER Implementation" section further down this
  file — that assessment assumed more surrounding traffic than we actually
  run.)
- **DCBF/DHOCBF convex-QP motion planner (Eqs. 14-19, 25-33).** Replacing the
  existing CasADi NLP wholesale, then re-tuning on hardware. Weeks, not days.
- **Paper's parameter set (Tables I-II).** Paper is a 3.5 m car at 18 m/s with
  `d_0=5 m`, `d_lat=2.1 m`, `l_diag=3.7 m`. QCar 2 is ~0.4 m at 0.75 m/s.
  Every value needs re-derivation; none transfer directly.

---

### CONFIRM WITH SUPERVISOR BEFORE BUILDING

- [ ] **"Lane filtering" is not verbatim paper terminology.** Best reading is
      the **Gap Magnitude Judgement** step (Alg. 2 line 6; "Filtered nodes" in
      Fig. 1) — filtering candidate lane-gaps that lack sufficient space.
      That reading fits both halves of his instruction (filtering + the V2V
      location check) and explains the failure he demonstrated. But it could
      also mean motorcycle-style lane filtering, which is a completely
      different behaviour. **One message before building — it changes the
      deliverable.**

---

## Next major feature: V2V communication with Quanser QCar 2

**Goal:** ROSbot 3 and a Quanser QCar 2 (a different vendor's platform — a
"heterogeneous" pair, not two ROSbots) exchange vehicle-to-vehicle state over
the network in real time, so behavior on one (e.g. `trajectory_follower_node.py`'s
pure pursuit / obstacle logic) can react to the other's position, heading,
speed, and intent.

This is a design breakdown to work from, not a locked spec — the "Open
Questions" section below has to be resolved with actual QCar 2 hardware
access before the architecture is final.

> **UPDATE 2026-08-04 — a teammate has already built and validated a real
> implementation.** Pulled from `github.com/6hammad9/qcar2-lane-keeping`,
> branch `fix/lidar-curve-corridor`. Everything below this point (Open
> Questions through Phased Workflow) was written *before* this was found and
> is now largely superseded — kept for historical context, but read
> "## Actual V2V Implementation (pulled from teammate's branch)" further
> down first; it answers most of the open questions here directly:
> - **Transport:** confirmed raw UDP + JSON, not native ROS 2 topics — for
>   the exact DDS-crash reason predicted in Option A/B's risk column below,
>   now confirmed to have actually happened twice on the lab Wi-Fi.
> - **Who runs the reconstructed IDEAM algorithm (§ hardware-reality-check
>   in the PAPER Implementation section):** answered — **QCar 2 only**. The
>   ROSbot is a broadcaster + a command-gated follower, never runs its own
>   MPC/DCBF. This sidesteps the differential-drive-vs-Ackermann dynamics
>   mismatch entirely, exactly as option (a) proposed there.
> - **Shared coordinate frame (§4 below):** resolved by assumption, not
>   measurement — both robots are assumed to localize against the *same*
>   map (transform = identity). Only needs real calibration
>   (`frame_tx/ty/tyaw`) if that stops being true.

---

### 0. Open questions to resolve first (blocks real architecture decisions)

- [ ] **What does QCar 2 actually run?** Quanser ships QCar 2 with its own
      ROS 2 packages (Jetson-based) — confirm which ROS 2 distro
      (Humble/Jazzy/other) and which `rmw` implementation it uses. This repo
      is on **ROS 2 Jazzy + `rmw_fastrtps_cpp`**; a distro/rmw mismatch is a
      real risk (we already fought DDS discovery issues between just two
      Jazzy machines in this project — see `memory.md` — a genuine
      cross-vendor, possibly cross-distro link will be worse).
- [ ] Does QCar 2 expose its state over native ROS 2 topics, or only through
      Quanser's own SDK/QLabs Python API? If the latter, native ROS 2
      pub/sub between the two vehicles isn't an option — need a bridge.
- [ ] What's the actual network topology at test time? Same LAN/WiFi AP,
      same subnet, or does this cross a router (multicast likely won't
      survive that — same class of problem as the laptop↔robot discovery
      issue already hit in this project)?
- [ ] Latency/rate requirements — is this informational (e.g. 1–5 Hz "here's
      roughly where I am") or safety-critical (e.g. collision avoidance
      needing <100ms and guaranteed delivery)? This changes the transport
      choice significantly (see below).
- [ ] Any authentication/security requirement, or is this a closed test-bench
      network where trust isn't a concern for v1?

---

### 1. Communication design — options to evaluate

| Option | How it'd work | Pros | Cons |
|---|---|---|---|
| **A. Shared ROS 2 domain, native topics** | Both vehicles on the same `ROS_DOMAIN_ID`, custom `v2v_msgs` package, plain `rclpy` pub/sub | Simplest to write, reuses everything already in this repo (TF, `rclpy` patterns) | Only works if QCar 2 is ROS 2 and distro/rmw-compatible; DDS discovery across real WiFi has already bitten us once this project — fragile over an AP that filters multicast |
| **B. `rmw_zenoh` / Zenoh bridge** | Route ROS 2 topics over Zenoh instead of raw DDS; `zenoh-bridge-ros2dds` can also bridge two *different* ROS 2 domains/distros, or bridge a non-ROS2 peer | Built for exactly this — cross-vendor, unreliable-link, WAN-friendly; actively maintained, native option in ROS 2 Jazzy+ | New piece of infra to learn/deploy; still needs both sides speaking something Zenoh can bridge |
| **C. Transport-agnostic (UDP broadcast / MQTT), JSON or Protobuf payload** | A `v2v_node` on each vehicle serializes its own state and broadcasts/publishes it independently of ROS 2 message generation entirely | Fully vendor-agnostic — works even if QCar 2's side is pure Python/Quanser SDK with no ROS 2 involved at all; easiest to reason about across a flaky link | You own framing/versioning/reliability yourself; no free RViz/`ros2 topic echo` introspection on the wire format |

**Recommendation for v1 prototyping:** start with **Option C** (plain UDP
broadcast, JSON payload) specifically *because* QCar 2's ROS 2 compatibility
isn't confirmed yet (see Open Questions above) — it de-risks the "can these
two systems talk at all" problem from the "are they DDS-compatible" problem,
which are separate concerns. Revisit **Option B (Zenoh)** once basic
comms are proven, if bidirectional low-latency ROS 2-native topics turn out
to be worth the infra investment (e.g. once trajectory data itself needs to
flow, not just pose/intent summaries).

---

### 2. Proposed system architecture (v1, Option C)

```
   ROSbot 3 (this repo)                       Quanser QCar 2
  ┌─────────────────────────┐                ┌─────────────────────────┐
  │ trajectory_follower_node│                │  QCar 2 control stack   │
  │           │             │                │           │             │
  │           ▼             │                │           ▼             │
  │      v2v_node.py        │◄──UDP/JSON────►│      v2v_node.py        │
  │  (peer_registry,        │  broadcast     │  (peer_registry,        │
  │   safety_monitor)       │  or Zenoh      │   safety_monitor)       │
  └─────────────────────────┘  (phase 2)     └─────────────────────────┘
```

Each vehicle runs its own `v2v_node`:
- **Broadcasts** its own state (pose, velocity, heading, timestamp, a coarse
  "intent" enum) at a fixed rate.
- **Listens** for peer broadcasts, keeps a `peer_registry` (drops peers whose
  last-seen timestamp is too stale — a heartbeat timeout).
- Exposes the latest peer state to the rest of the vehicle's own stack (e.g.
  `trajectory_follower_node.py`'s obstacle-check logic) via a normal ROS 2
  topic *internally* — the UDP/Zenoh boundary is only between vehicles, the
  ROSbot-internal consumers still just subscribe to a local ROS 2 topic like
  `/v2v/peers`.

This keeps the "weird cross-vendor transport" code isolated to one package,
and everything downstream of it (pure pursuit, obstacle logic) just
consumes a normal local ROS 2 topic — no need to touch
`trajectory_follower_node.py`'s core control loop to plug this in later,
just add a subscriber.

---

### 3. Proposed folder structure

New top-level ROS 2 package, sibling to `rosbot_lane/` — kept separate
since it's a different concern (networking/comms vs. vehicle control) and
so it can eventually be built/run independently of the lane package:

```
rosbot3/
├── rosbot_lane/            (existing)
├── v2v_msgs/                # NEW — ament_cmake interface package
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── msg/
│       ├── VehicleState.msg
│       └── PeerList.msg
└── v2v_comm/                 # NEW — ament_python node package
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── v2v_comm
    ├── config/
    │   └── v2v_params.yaml
    ├── test/
    │   ├── test_copyright.py
    │   ├── test_flake8.py
    │   └── test_pep257.py
    └── v2v_comm/
        ├── __init__.py
        ├── v2v_node.py              # main node: owns broadcast + listen loop
        └── core/
            ├── __init__.py
            ├── config.py             # load v2v_params.yaml (mirror rosbot_lane/core/config.py pattern)
            ├── transport_udp.py      # Option C: raw UDP broadcast send/recv
            ├── transport_zenoh.py    # Option B: Zenoh pub/sub (phase 2, stub for now)
            ├── message_codec.py      # encode/decode VehicleState <-> JSON/bytes
            ├── peer_registry.py      # track known peers, heartbeat timeout/eviction
            └── safety_monitor.py     # conflict detection (e.g. peer too close + closing speed)
```

**Why a separate `v2v_msgs` package:** `rosbot_lane` is `ament_python`, and
`ament_python` packages can't cleanly generate custom `.msg` interfaces
themselves — the standard ROS 2 pattern is a small `ament_cmake` package
that only holds interface definitions, imported by whichever Python
packages need the generated message types. Worth doing this even for
Option C (UDP/JSON) — keep `VehicleState` as a real ROS 2 message for the
*local* topic (`/v2v/peers`) that `trajectory_follower_node.py` will
eventually subscribe to, even though the *inter-vehicle* wire format is
plain JSON, not this message type directly.

---

### 4. Suggested message fields (`VehicleState.msg` draft)

```
std_msgs/Header header
string vehicle_id          # e.g. "rosbot3", "qcar2"
geometry_msgs/Pose2D pose  # x, y, theta — in a locally-agreed shared frame (see open question below)
float32 speed               # m/s
float32 heading_rate        # rad/s, for closing-speed estimation
uint8 intent                 # enum: UNKNOWN=0, HOLD=1, PROCEED=2, YIELDING=3, ...
float32 seconds_since_seen   # populated locally by peer_registry, not sent over wire
```

Open sub-question worth flagging early: pose is meaningless across vehicles
unless both agree on a shared coordinate frame (or you fall back to
GPS/UWB/some shared external reference) — the ROSbot's `map` frame from its
own SLAM run has no relationship to QCar 2's own localization frame unless
you explicitly establish one (e.g. both localized against a shared/known
track geometry, or a shared external positioning system). This needs an
answer before pose sharing is meaningful, not just wire format.

---

### 5. Phased workflow

- [ ] **Phase 0 — Research & bench access.** Resolve the Open Questions
      (§0) with actual QCar 2 access. Confirm its ROS 2 distro/SDK situation
      before writing more than a stub.
- [ ] **Phase 1 — One-way broadcast, same LAN.** `v2v_node` on the ROSbot
      broadcasts `VehicleState` as UDP/JSON at ~2–5 Hz; a throwaway script on
      the QCar 2 side (or a laptop) just logs what it receives. Prove the
      link works before building anything that consumes it.
- [ ] **Phase 2 — Bidirectional + local ROS 2 topic.** Both vehicles run
      `v2v_node`, each publishes received peer state onto its own local
      `/v2v/peers` topic. No behavior change yet — just visibility
      (`ros2 topic echo /v2v/peers`).
- [ ] **Phase 3 — Consume it.** Add a `/v2v/peers` subscriber into
      `trajectory_follower_node.py`'s obstacle-check path (`_check_obstacle`
      currently only reasons about LIDAR — extend it to also factor in a
      known peer's reported pose/speed).
- [ ] **Phase 4 — Shared coordinate frame.** Solve the cross-vehicle pose
      alignment problem (§4) — needed before peer *position* is trustworthy,
      not just peer *presence*.
- [ ] **Phase 5 — Evaluate Option B (Zenoh).** If Phase 1–4 prove the
      concept but latency/reliability over UDP broadcast is a limiting
      factor, swap `transport_udp.py` for `transport_zenoh.py` behind the
      same `v2v_node` interface — this is why the transport is isolated
      into its own module from the start.
- [ ] **Phase 6 — Field test on the actual track**, with both vehicles
      moving, before trusting this for anything safety-relevant.

---

## Actual V2V Implementation (pulled from teammate's branch)

**Source:** `github.com/6hammad9/qcar2-lane-keeping`, branch
`fix/lidar-curve-corridor`, pulled 2026-08-04. Full design writeup lives in
that repo's `V2V_README.md` (308 lines — read it in full before touching
this, only summarized here). Two files pulled directly into this repo:
`rosbot_lane/rosbot_v2v_broadcaster.py` and `rosbot_lane/rosbot_v2v_gate.py`
— both standalone, zero imports from the rest of `rosbot_lane`, by design
(copy-and-run on the robot, `trajectory_follower_node.py` stays untouched).

**Before doing anything else with this:** the `detection.py`/`geometry.py`/
`config.py`/`lane_keeping_node.py`/`lane_params.yaml` copies in the
teammate's `rosbot_opta2/` mirror are an **older snapshot that predates
every fix from this session** (still has the `768×432` resolution bug,
`kp: 0.30`, none of the CL/RL detection fixes) — confirmed by diff, not
assumed. Don't pull those files in; this repo's copies are strictly ahead.
Only the two `rosbot_v2v_*.py` files and the reference docs were new.

### Architecture

Deliberately **asymmetric**, not the symmetric peer-to-peer design
originally sketched in this file:
- **ROSbot 3** (`rosbot_v2v_broadcaster.py`): broadcasts pose + speed +
  a predicted future trajectory (rolled forward along its own recorded
  `smoothed_trajectory.csv` at measured speed — exact, not IDM-style
  guessing, since the robot's future *is* the recorded path) + obstacle/
  detour-intent, over UDP at 10Hz. Never receives or reacts to anything.
- **QCar 2**: the only vehicle that optimizes. Receives, validates, and
  reacts — slows, follows, stops, or overtakes — via an MPC with a DCBF
  safety layer (elliptical keep-out around the ROSbot's predicted pose,
  directly implementing the IDEAM paper's ellipse-constraint DHOCBF
  concept from Eqs. 25-30, cited explicitly in their README) plus a speed
  governor. Can send HOLD/PROCEED commands back for the narrow case where
  it needs the ROSbot to physically stop moving (passing maneuvers the
  ROSbot's own blind-spot-limited overtake logic can't see coming) —
  `rosbot_v2v_gate.py` is what receives and enforces that hold, gating
  `trajectory_follower_node.py`'s output rather than modifying it.

**Transport is raw UDP + JSON, not ROS 2 topics or DDS** — deliberately,
because (per their README, and matching this session's own
`memory.md` history) this lab's Wi-Fi has *twice* crashed the entire ROS
graph via corrupted Fast DDS discovery when mixed ROS 2 distros shared a
domain. Both robots keep independent, private ROS graphs
(`ROS_LOCALHOST_ONLY=1` or distinct `ROS_DOMAIN_ID`s) and V2V rides a
socket DDS discovery never touches. This directly validates Option C from
this file's own earlier communication-design comparison table.

**Fail-safe contract** (the property to actually test before trusting this
on track): no broadcaster / stale packets / malformed data / ROSbot not
localized all degrade to "QCar behaves exactly as it did before V2V
existed" — never silently read as "clear to proceed."

### Deployment (ROSbot side — QCar side lives in the teammate's own repo)

```bash
# after tf_relay + SLAM/AMCL are up, same as any other run in this repo:
python3 rosbot_lane/rosbot_v2v_broadcaster.py --ros-args \
  -p target_ip:=192.168.0.53 \
  -p trajectory_csv:=$(pwd)/config/smoothed_trajectory.csv

# if the QCar needs to hold this robot during a pass, in a separate terminal:
ros2 run rosbot_lane trajectory_follower_node --ros-args \
  -r /rosbot3/cmd_vel:=/rosbot3/cmd_vel_raw
python3 rosbot_lane/rosbot_v2v_gate.py --ros-args -p bind_port:=47101
```

Verify the link from the QCar side before any driving:
`ros2 topic echo /v2v/stats` (rx counting up, parse_errors 0),
`/v2v/alive` (true), `/v2v/gap` (sensible along-path meters).

### Findings from their work, relevant to this repo specifically

- **A general lesson that applies directly to our own `kp` fight today:**
  they found the QCar 2's default `max_steer=0.50` made it geometrically
  *unable* to follow a path recorded by the ROSbot (a differential-drive
  robot that can pivot in place) — the path demanded a 0.456m turning
  radius, the QCar's default steering only allowed 0.469m minimum.
  Tracking error was 0.474m until they corrected `max_steer`/`max_speed`
  to the vehicle's real limits, dropping it to 0.024m. Same *class* of bug
  as this session's `lane_keeping_node.py` `kp` issue — a controller
  physically incapable of the turn radius the path demands, silently
  producing bad tracking instead of an obvious error. Worth remembering
  as a checklist item any time a recorded path gets driven by a different
  vehicle than the one that recorded it.
- Their sim testing (Gazebo, real map, 2026-08-01) caught a real
  `PathProjector.closest_idx()` bug: a global nearest-waypoint search
  flipped across the track because their recorded path happens to pass
  within 7mm of itself — a reminder that path self-intersection (or
  near-intersection) is a real hazard class for any nearest-waypoint
  logic, including `find_closest_waypoint` in this repo's own
  `core/trajectory.py` if a recorded loop ever comes close to crossing
  itself.

### QCar 2 side pulled locally + bench-tested (2026-08-04)

The full teammate repo (not just the two ROSbot-side files) is now also
available at **`~/qcar2_side/`** (sibling to this repo, its own git remote
pointed at `github.com/6hammad9/qcar2-lane-keeping`, branch
`fix/lidar-curve-corridor` — `git pull` there directly for their updates).
Contains `src/qcar_science_night_pkg/` (the V2V receiver + MPC/DCBF +
overtake state machine), `src/qcar2_nodes/` (C++ hardware driver, needs
real QCar hardware/Quanser SDK — won't build here), `src/qcar_lane_pkg/`
(an earlier/alternate QCar lane-keeping package), `sim/`, and `utils/`
(including `v2v_fake_rosbot.py` and `v2v_selftest.py`, their own
bench-testing tools).

**Ran their documented "bench test without hardware" workflow for real,
on `rosbot-server`, both sides at once — not just import-checked:**

```bash
# Terminal A — QCar-side receiver (no hardware/Quanser SDK needed, pure rclpy)
cd ~/qcar2_side/src/qcar_science_night_pkg
python3 -m qcar_science_night_pkg.v2v_receiver_node --ros-args \
  -p trajectory_file:="$(pwd)/config/recorded_path_amcl_final.npy"

# Terminal B — the REAL ROSbot-side broadcaster (not their fake stand-in)
cd ~/rosbot3
python3 rosbot_lane/rosbot_v2v_broadcaster.py --ros-args \
  -p target_ip:=127.0.0.1 \
  -p trajectory_csv:=config/smoothed_trajectory.csv

# Terminal C — watch it
ros2 topic echo /v2v/stats
ros2 topic echo /v2v/alive
```

**Result: genuinely confirmed working**, not just "imports cleanly":
- `rx: 624, parse_errors: 0` on the receiver — real schema-2 packets from
  the actual `rosbot_v2v_broadcaster.py` (not the fake tool) parsing
  correctly.
- `/v2v/alive = false`, correctly — no SLAM/AMCL was running on
  `rosbot-server` at test time, so the broadcaster had no live TF and
  correctly sent "not localized" heartbeats (`loc: 0`) per its own design;
  the receiver correctly reported that as *not alive*, never as "clear."
  This is the documented fail-safe contract actually observed working,
  not just read about.
- Also separately validated with their own `v2v_fake_rosbot.py` tool
  (schema 1, backward-compatible) replaying our real
  `config/smoothed_trajectory.csv` — `/v2v/rosbot_speed` correctly showed
  `0.25` matching the `--speed 0.25` passed to it. `/v2v/gap`/`on_path`
  came back `NaN`/`false` in that run, which is *correct*, not a bug — the
  fake tool was replaying the ROSbot's own track against the receiver's
  QCar-path reference (`recorded_path_amcl_final.npy`), two genuinely
  different physical tracks/coordinate frames, so "gap along the QCar's
  path" is legitimately undefined for that combination. A real field test
  (both vehicles on the same physical map) would need matching paths — see
  open items below.
- Test processes were stopped after validation; nothing is left running.

### Open items

- [ ] **Fix ROSbot 3's overtake logic — unreliable, gets stuck.**
      `rosbot_lane/trajectory_follower_node.py`'s overtake state machine
      generates a detour when it hits an obstacle but then gets stuck
      indefinitely in "Obstacle ahead — holding position" instead of
      completing the maneuver, even after the obstacle (QCar 2, parked)
      was moved clear. One earlier overtake near the same spot DID
      succeed, so this is intermittent, not a total break. Unrelated to
      V2V — this is ROSbot 3's own standalone LiDAR/overtake logic.
      Investigate: does the detour path get regenerated/re-evaluated
      while holding, or does it keep retrying the same stale 21-point
      detour from before the hold started? Also log the actual LiDAR
      range at the stuck moment vs. whatever corridor/threshold it's
      being compared against. Full context in the "Staged joint-driving
      test" section's 2026-08-05 pause note below.
- [ ] Confirm which named person(s) `6hammad9` corresponds to relative to
      Qais's supervisor guideline (Mr. Afreed / Mr. Sharjeel own
      perception/localization/actuation; unclear if this is one of them
      under a different GitHub handle, or a third collaborator) — matters
      for the "at least 2 members on paper reconstruction" requirement.
- [ ] The `path_mpc_node.py` / DCBF work described in their README lives
      on the QCar 2 side, outside this repo — if it substantially
      reconstructs IDEAM's motion-planner math already, that's directly
      relevant to the "PAPER Implementation and Supervisor Guideline"
      section above and worth reviewing together rather than duplicating.
- [x] **Bench communication link confirmed** — see "Live cross-machine
      test" below. Full real-hardware validation done 2026-08-04.
- [x] **RESOLVED 2026-08-05** — landmark data collected, transform
      computed and written. See "CRITICAL CORRECTION" and
      "Landmark-transform data collection" sections below for the full
      story (round 1 was against the wrong QCar 2 workspace; round 2
      against the correct `~/qcar_v2v_ws` worked, 3.7cm residual).
- [x] **Map-frame alignment between the two robots — DONE 2026-08-05.**
      > **SUPERSEDED 2026-08-26 — DO NOT USE THE VALUES BELOW.** The
      > landmark-transform result in this item was replaced by a
      > loop-to-loop ICP solve (4.7 cm RMS). The transform **actually
      > live on QCar 2** — verified on the robot 2026-08-28 in *both*
      > `ros2_ws_izhan` and `ros2_ws_sami` — is:
      > `frame_tx=1.3814, frame_ty=0.4986, frame_tyaw=-3.030697`.
      > See `memory/v2v-map-frame-calibration.md`, and the local copy at
      > `qcar2/src/qcar_science_night_pkg/config/v2v_params.yaml`.
      > Restoring the old numbers below would silently break `gap`/`on_path`.
      > Note also that the two-landmark method itself was **abandoned as
      > inconsistent** (P1->P2 measured 4.370 m in ROSbot 3's map vs
      > 3.460 m in QCar 2's — impossible for a rigid transform); the
      > `?` left in `~/Desktop/v2v_calibration/landmark_measurements_INCONSISTENT.txt`
      > is a dead end, not outstanding work.

      Decision made 2026-08-04: going with the landmark-transform
      approach, not switching to a shared map. Result:
      `frame_tx=-4.679956, frame_ty=-3.745196, frame_tyaw=-0.705757`,
      written to QCar 2's `v2v_params.yaml`, 3.7cm residual — see
      "Landmark-transform data collection" section above for the full
      story (round 1 was against the wrong QCar 2 workspace and produced
      garbage; round 2 against the correct `~/qcar_v2v_ws` worked).
      `v2v_params.yaml`'s `frame_tx/ty/tyaw` (rigid transform between the
      two robots' map frames) defaults to identity, which their own
      README says assumes both robots localize against the *same* map.
      They don't: ROSbot 3 uses `config/track_map.yaml` (origin
      `[-9.603, -2.732, 0]`), QCar 2's launch loads a *different* map,
      `track_map_new.yaml` (origin `[-6.96, -10, 0]`) — two independently
      recorded SLAM sessions of the same physical track, with no shared
      absolute coordinate reference. Until `frame_tx/ty/tyaw` is actually
      measured (their README: "measure two shared landmarks in both
      frames and set frame_tx/ty/tyaw"), `on_path`/`gap` will not be
      meaningful even with both vehicles genuinely driving the real
      track — this is a prerequisite for a real field test, not just for
      today's desk-side bench test.
    - **Transform convention** (from `v2v_common.py`'s `se2_apply`):
      maps a ROSbot 3 pose `(x,y,yaw)` into QCar 2's frame via
      `x' = tx + cos(tyaw)*x - sin(tyaw)*y`,
      `y' = ty + sin(tyaw)*x + cos(tyaw)*y`, `yaw' = yaw + tyaw`.
    - **Procedure:** pick 2 distinct, precisely identifiable physical
      points on the real track. For each: position ROSbot 3 exactly at
      the point, read its converged `/amcl_pose` (x,y); do the same for
      QCar 2 at the same physical point, read its own `/amcl_pose`.
      Solve the 2-point rigid transform (rotation from the angle between
      the two landmark vectors in each frame, then translation) for
      `tx/ty/tyaw`, write into `config/v2v_params.yaml` on the QCar 2
      side (`frame_tx`/`frame_ty`/`frame_tyaw`).
- [ ] **Staged joint-driving test — decision made 2026-08-04, IN
      PROGRESS 2026-08-05.**
      **Step 0 (fix stale v2v_params.yaml paths): DONE.**
      **Step 1 (both stacks up, stationary, V2V off): DONE.**
      **Step 2 (V2V link up, both stationary, no motion capability armed):
      DONE, SUCCESS.** `v2v_receiver` (on `~/qcar_v2v_ws`, correct
      workspace) + `rosbot_v2v_broadcaster.py` (on ROSbot 3) brought up
      together. First real end-to-end validation of the calibrated
      transform: `/v2v/alive=true`, `/v2v/gap=17.07m` (sane, not
      garbage), **`/v2v/on_path=true`** (ROSbot 3's broadcast position
      correctly registers as being on QCar 2's route — direct proof the
      landmark transform is correct), stats clean (6167 rx, 0 parse
      errors, 0 seq drops, age_s=0.088). Took a long detour debugging a
      false alarm: `ros2 node list`/`ros2 topic echo <topic>` (no
      explicit type) returned empty/failed even though everything was
      actually fine — those tools look up the topic's type via the graph
      first and give up fast under discovery latency, unlike
      `tf2_echo`/type-explicit `topic echo <topic> <type>` which don't
      need that lookup. **Lesson for future sessions:** if
      `ros2 node list` or bare `ros2 topic echo <topic>` come back empty
      but you have reason to believe things are actually running, don't
      assume the graph is broken — retry with an explicit type
      (`ros2 topic echo <topic> <type_string> --once`) or use `tf2_echo`
      style tools before concluding there's a real connectivity problem.
      **Step 3 (ROSbot 3 drives from Landmark 2, QCar 2 parked at
      Landmark 1, watching `/v2v/gap`/`/v2v/on_path` live): IN
      PROGRESS 2026-08-05, working as expected.** ROSbot 3's own LiDAR
      safety stack correctly paused it for an obstacle mid-drive
      (independent of V2V). `on_path` flipped `false` at that point —
      investigated via the `v2v_rx_log.csv` raw log (confirmed the
      step-0 path fix also fixed logging): `lat_offset=-0.451m`, just
      over the `lane_half_width=0.35m` threshold. **Not a calibration
      bug** — a transform with 3.7cm landmark residual doesn't produce a
      45cm error; this is genuine physical divergence between ROSbot 3's
      recorded route and QCar 2's specific reference lane at that point
      on the track (expected between two independently-recorded paths).
      `blocked=true` confirmed to come directly from ROSbot 3's own
      broadcast packet (its own pause state relayed correctly), not
      computed by QCar 2. Both fields behaving exactly as designed.
      **Useful troubleshooting note for future sessions:** the raw
      `v2v_rx_log.csv` (columns:
      `t_rx,seq,age_gap_ms,x,y,yaw,v,localized,moving,qcar_idx,rosbot_idx,gap,lat_offset,on_path`)
      is the fastest way to sanity-check `on_path`/`gap`/`blocked`
      against real numbers rather than guessing from the boolean topics
      alone.
    - **`gap` sign-flip investigated and explained (2026-08-05).** Saw
      `gap` jump ~38.77m in one 100ms tick (e.g. `19.368 → -19.399`)
      while `rosbot_idx` ticked forward by exactly 1 and ROSbot 3's raw
      x/y stayed perfectly continuous — confirmed via `v2v_rx_log.csv`.
      Root cause: `gap` is signed "shortest way around the loop"; the
      jump magnitude matches the recorded route's total loop length
      (38.80m per HANDOFF.md) almost exactly, meaning this happens right
      when the two vehicles are close to *half a loop length apart* —
      genuinely ambiguous which direction is shorter, so the sign flips.
      **Not a calibration/projection bug** — inherent to any signed
      loop-gap metric, and it's exactly the geometry this test set up
      (QCar 2 parked at Landmark 1, ROSbot 3 looping the long way from
      Landmark 2). **Caveat for later, not yet checked:** if QCar 2's
      MPC/governor ever consumes raw signed `gap` while actually driving
      (stage 4/5), a flip at the loop's halfway point would look like a
      sudden ~38m gap change — worth confirming the governor code uses
      `abs(gap)` or hysteresis around that crossover before trusting it
      in a real joint-driving pass.
    - **ROSbot 3's own (V2V-independent) obstacle avoidance encountered
      QCar 2 itself, parked in its path near Landmark 1.** First
      encounter: overtake detour succeeded and rejoined normally. Second
      lap, same spot (`rejoin at WP 461` both times): overtake generated
      but repeatedly got stuck at 0.39-0.40m unable to complete, holding
      position 40+ seconds real time — the detour path still passed too
      close to QCar 2's physical footprint. Resolved by moving QCar 2
      further clear of ROSbot 3's driving corridor. Good real-world
      confirmation that ROSbot 3's stock obstacle avoidance treats QCar 2
      as an ordinary obstacle (expected — its autonomy stack is
      completely unmodified/standalone per the V2V design), not a V2V
      bug.
    - **PAUSED HERE 2026-08-05, resume next session.** V2V link + step 3
      (one vehicle moving, one parked) both validated working correctly.
      Steps 4 (QCar 2 drives, ROSbot 3 parked) and 5 (both moving) not
      yet attempted — don't skip straight to step 5. **Plan: user is
      doing step 4 next** (QCar 2 drives via `path_mpc`/`lidar_overtake`
      per RUNBOOK.md's three-terminal sequence, ROSbot 3 stays parked) —
      confirms QCar 2's own driving is unaffected by V2V being enabled
      with no real conflict to react to. Still keep in mind the `gap`
      loop-halfway sign-flip caveat above if the governor logs anything
      that looks like a sudden ~38m gap jump while QCar 2 is moving.
      **Separate, not blocking step 4: ROSbot 3's own overtake maneuver
      (`trajectory_follower_node.py`, unrelated to V2V) is unreliable —
      tracked as its own item below, fix it whenever, doesn't need to
      happen before step 4 since ROSbot 3 stays parked for that step.**
      Confirmed happening again at end of this session —
      generates a detour but gets stuck holding position indefinitely
      ("Obstacle ahead — holding position", repeating every ~1s) instead
      of completing it, even after the QCar 2 obstacle was moved clear
      once already. Worth digging into
      `rosbot_lane/trajectory_follower_node.py`'s overtake state machine
      (detour generation / rejoin logic) directly — first lap's overtake
      DID succeed near the same spot, so this looks like an intermittent
      failure mode, not a total break. Check: does the detour path
      itself get regenerated/re-evaluated while holding, or does it keep
      retrying the same (now-invalid) 21-point detour from before? Also
      worth logging exactly what LiDAR range is being read at the stuck
      moment vs. the corridor/threshold it's compared against.
      **To resume:** re-verify both localization stacks are still up and
      converged (don't assume readings are still valid after a time
      gap — same caution as always), confirm ROSbot 3's `.venv`/
      `ROS_DOMAIN_ID` and QCar 2's per-terminal sourcing (`cd
      ~/qcar_v2v_ws`, `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`) since
      neither persists across terminals/reboots, then dig into the
      overtake bug before attempting steps 4/5.
      Once frame alignment is in place, running
      `trajectory_follower_node.py` on ROSbot 3 and QCar 2's own driving
      stack at the same time is a real step up in physical risk from
      everything tested so far (all prior tests were stationary/desk-side).
      Plan: stage it — e.g. one vehicle moving while the other stays
      parked first, confirm `gap`/`on_path`/the governor behave sanely
      from that alone, before letting both move autonomously at once.
      E-stop/manual override should be ready throughout.
- [x] **`rosbot_v2v_gate.py`'s HOLD/PROCEED path — tested directly on
      real hardware, 2026-08-04.** Gate run on `rosbot-server`, fed a
      synthetic `TwistStamped` (angular.z=0.05 only, no linear motion) on
      `/rosbot3/cmd_vel_raw`, commands sent via raw UDP to `127.0.0.1:47101`
      matching the real wire schema. All 5 behaviors confirmed on the real
      `/rosbot3/cmd_vel` topic: passthrough when not held; `HOLD` zeroes
      output; TTL auto-expiry releases on its own without needing a
      `PROCEED` (bounded lease, never a latch — `gated_messages: 50` at
      10Hz over the 5s TTL, exact); an out-of-range TTL (`30.0`, over the
      `MAX_COMMAND_TTL_SEC=5.0` cap) is correctly rejected as a
      `parse_error` rather than accepted; and an explicit `PROCEED`
      correctly releases an active hold before its TTL would have expired
      on its own. One minor unexplained quirk, not functionally
      significant: the gate's own "hold released" log line for the
      explicit-PROCEED case was timestamped ~5s after "engaged" (matching
      the TTL) rather than immediately after the PROCEED was sent (~2s in),
      even though the actual `/rosbot3/cmd_vel` topic data directly
      confirmed the release happened around 2s, well before TTL expiry —
      worth a closer look sometime, doesn't affect the conclusion since
      the real gating behavior was checked directly.

### CRITICAL CORRECTION (2026-08-05): wrong QCar 2 workspace used all session

Discovered by reading `qcar2_side/HANDOFF.md` and `RUNBOOK.md` (dated
2026-08-02/03) — user confirmed **`~/qcar_v2v_ws` is the actual current
setup** on the physical QCar 2. Everything QCar-2-side done in this
session up to this point (the "QCar 2 side pulled locally + bench-tested"
work, the "Live cross-machine test", and all 4 landmark readings above)
used the **wrong workspace**: `~/ros2_ws`, `ROS_DOMAIN_ID=1`, AMCL +
`science_night_slam.launch.py` + `track_map_new.yaml`. The real one:

| | Correct (`~/qcar_v2v_ws`) | Wrong, used all session (`~/ros2_ws`) |
|---|---|---|
| `ROS_DOMAIN_ID` | **42** | 1 |
| Localization | **Cartographer pure-localization**, no `/amcl_pose` topic | AMCL, has `/amcl_pose` |
| Map | `mapping_output/qcar_real_20260802-014755.yaml` | `track_map_new.yaml` |
| Bring-up | `ros2 launch qcar2_nodes qcar2_cartographer_launch.py state_filename:=... configuration_basename:=qcar2_2d_localization.lua resolution:=0.05` (RUNBOOK.md §1 Terminal 1) | `science_night_slam.launch.py` |
| Pose check | **`ros2 run tf2_ros tf2_echo map base_link`** (no `/amcl_pose` exists) | `ros2 topic echo /amcl_pose` |

**Consequences:**
- The 4 landmark `/amcl_pose` readings above are against the wrong map —
  **must be redone** using `tf2_echo map base_link` against
  `~/qcar_v2v_ws`'s Cartographer localization once it's brought up.
- The V2V bench test / live cross-machine test protocol-level results
  (packet delivery, fail-safe contract, alive-flag behavior) are still
  probably valid as UDP-link tests (V2V rides raw UDP, domain-agnostic by
  design per `V2V_README.md`), but were not exercised against the actual
  production localization/map — worth re-running once `~/qcar_v2v_ws` is
  up, not urgent.
- `enable_v2v`/`v2v_fusion_enable` default `false` in `~/qcar_v2v_ws`'s
  known-good `path_mpc`/`lidar_overtake` launch params (RUNBOOK.md §1 T2/T3)
  — V2V receiver code exists there but is disabled by default, consistent
  with what we built.
- The earlier ~3.9x landmark-distance-mismatch investigation (map
  resolution ruled out, AMCL-seeding theory) is likely moot — it was
  chasing an artifact of using a possibly-stale `track_map_new.yaml`
  rather than a real transform problem. Don't carry that theory forward;
  just redo the readings clean against the correct workspace.

### Landmark-transform data collection (started 2026-08-05)

Raw `/amcl_pose` readings for the 2-point rigid transform (see procedure
above). Recording each as it's collected so nothing gets lost again.

- **QCar 2 @ Landmark 1** (converged, cov x=0.0073 y=0.0022 yaw=0.0055):
  `x=-3.080313121212298, y=-3.078642304680784`,
  `orientation.z=0.061260711418388424, w=0.9981217987983796`
  → yaw = 2*atan2(z, w) ≈ 0.1225 rad
- **QCar 2 @ Landmark 2** (converged, cov x=0.0017 y=0.0070 yaw=0.0080):
  `x=-2.858151435455781, y=-2.1164484116334163`,
  `orientation.z=0.7567008508985412, w=0.6537612884298241`
  → yaw = 2*atan2(z, w) ≈ 1.716 rad
- **ROSbot 3 @ Landmark 1** (re-collected 2026-08-05, converged, cov
  x=0.0040 y=0.0189 yaw=0.0122 — original 2026-08-04 reading's exact
  x/y/yaw values were lost in a context-compaction gap, so re-collected
  fresh rather than reused): `x=-0.9781308487815419, y=3.730390478637725`,
  `orientation.z=-0.7089744073725912, w=0.7052342091040984`
  → yaw = 2*atan2(z, w) ≈ -1.576 rad
- **ROSbot 3 @ Landmark 2** (converged, cov x=0.0249 y=0.0042 yaw=0.0188):
  `x=0.008356154748831, y=-0.003330947331620`,
  `orientation.z=0.005646527349004966, w=0.9999840582373786`
  → yaw = 2*atan2(z, w) ≈ 0.0113 rad
- **Physical landmark identities** (for reference): Landmark 1 = middle of
  the map entrance. Landmark 2 = near the heater section, 2nd block line.
- **SUPERSEDED — see "CRITICAL CORRECTION" section above.** The first
  round of 4 readings (below, struck through in spirit not literally) was
  collected against the wrong QCar 2 workspace (`~/ros2_ws`/AMCL/
  `track_map_new.yaml`) and produced a ~3.9x landmark-distance mismatch
  that was never conclusively explained — moot now, redoing clean against
  `~/qcar_v2v_ws`/Cartographer instead. Old readings kept above for
  reference only, not to be used.

#### Round 2 — correct workspace (`~/qcar_v2v_ws`, Cartographer, 2026-08-05)

- **QCar 2 @ Landmark 1** (`tf2_echo map base_link`, stable across
  multiple ticks, mm-level jitter): `x=-2.927, y=-0.368`, yaw≈-1.510 rad
  (-86.5°).
- **ROSbot 3 @ Landmark 1** (same physical spot QCar 2 was just standing
  on, converged cov x=0.0032 y=0.0253 yaw=0.0158):
  `x=-0.850646289126626, y=3.6713109290438415`,
  `orientation.z=-0.6937356453830356, w=0.7202297233001309`
  → yaw = 2*atan2(z, w) ≈ -1.535 rad
- **QCar 2 @ Landmark 2** (`tf2_echo map base_link`, stable across
  multiple ticks): `x=-3.500, y=-1.300`, yaw≈-1.486 rad (-85.2°).
- **ROSbot 3 @ Landmark 2** (same spot QCar 2 was just at; took 3 tries —
  first attempt was still essentially at Landmark 1 (~20cm away, rejected),
  second attempt didn't converge (y var 0.24-0.25, likely a corridor/
  along-axis ambiguity near this spot), third attempt after driving
  forward/back converged cleanly: cov x=0.0030 y=0.0285 yaw=0.0196):
  `x=-0.6935182068190987, y=2.6625484196337523`,
  `orientation.z=-0.678511926607521, w=0.7345893856103218`
  → yaw = 2*atan2(z, w) ≈ -1.491 rad
- **All 4 round-2 readings collected. Transform computed and written —
  DONE 2026-08-05.**
  `distance(L1,L2)`: ROSbot 3 frame 1.0209 m vs QCar 2 frame 1.0941 m
  (ratio 1.07, plausible — footprint/placement tolerance, not a scale
  bug). Solved via `se2_apply` 2-point rigid transform:
  **`frame_tx=-4.679956, frame_ty=-3.745196, frame_tyaw=-0.705757`**
  (rad, ≈-40.4°). Residual when predicting each landmark from the other:
  **3.7 cm on both L1 and L2** — solid, self-consistent this time.
  Written into
  `~/qcar_v2v_ws/src/qcar_science_night_pkg/config/v2v_params.yaml` on
  QCar 2 (confirmed via `grep frame_t`). That file is a full
  symlink-install chain (`install → build → src`), so no rebuild needed —
  editing `src/` was sufficient.
  **Fixed 2026-08-05:** `trajectory_file`/`log_file`/`path_spacing` in
  that same file were stale, pointing at `~/ros2_ws` and the old 0.03
  spacing — updated to `~/qcar_v2v_ws/mapping_output/my_route_loop.npy`,
  `~/qcar_v2v_ws/v2v_rx_log.csv`, and `0.05` (matching RUNBOOK.md's
  `path_mpc`/`lidar_overtake` params) so `gap`/`on_path` will actually be
  meaningful once V2V is enabled.

### Live cross-machine test — real hardware, both robots powered on (2026-08-04)

Full stack brought up on both real machines simultaneously (not a bench
test/localhost — genuine `192.168.0.110` ↔ `192.168.0.53` over the lab
WiFi):

- **ROSbot 3** (`rosbot-server`): `tf_relay` + AMCL (localized against
  `config/track_map.yaml`) + the real `rosbot_v2v_broadcaster.py`
  targeting QCar 2's IP.
- **QCar 2**: full `science_night_slam.launch.py` (hardware + AMCL against
  `track_map_new.yaml`) + `v2v_receiver`.

**Result, watched live from QCar 2's own `/v2v/stats`/`/v2v/alive`:**
1000+ packets, **0 parse errors, 0 sequence drops**, `age_s` staying
~0.03–0.14s (well under the 0.6s stale threshold). `/v2v/alive` correctly
tracked ROSbot 3's actual localization state throughout (`false` while
unlocalized, flipped to `true` once AMCL converged) — the documented
fail-safe contract observed working on real hardware, not just in a bench
test. `blocked: true` / `blocked_distance: 0.566m` and `on_path: false` /
`gap: null` were both confirmed *correct* for the physical setup at test
time (both robots sitting close together on a desk, not driving the
track — see the map-frame-alignment open item above for why `gap`/`on_path`
specifically aren't meaningful yet regardless).

**Two unrelated infrastructure problems hit and fixed/worked around during
this test, neither caused by V2V code:**
- **Every new terminal on QCar 2 needs its own `ROS_DOMAIN_ID`/sourcing —
  learned the hard way, repeatedly**, same class of lesson as ROSbot 3's
  own `.venv`/`ROS_DOMAIN_ID` non-persistence issues logged earlier this
  session. Root cause each time: a new shell doesn't inherit env vars set
  in a different terminal. Recommended fix (not yet done): add
  `export ROS_DOMAIN_ID=42` to QCar 2's own `~/.bashrc`.
- **`ros2 launch nav2_bringup localization_launch.py` on `rosbot-server`
  now crashes its `lifecycle_manager` process outright** —
  `undefined symbol` involving `diagnostic_updater::Updater`, a C++ ABI
  mismatch: `ros-jazzy-diagnostic-updater` is at a newer build
  (`4.2.7`, built 2026-06-15) than `ros-jazzy-nav2-lifecycle-manager`
  (`1.3.11`, built 2026-04-12) — likely from a partial system package
  update (an `apt` history entry from 2026-08-02 on this shared machine).
  Not caused by anything in this repo, and no sudo available to fix the
  package mismatch itself. **Workaround that works fine:** `map_server`
  and `amcl` themselves start and run correctly even when
  `lifecycle_manager` crashes — just skip it and activate both manually
  (`ros2 lifecycle set /map_server configure`, `activate`, same for
  `/amcl`), exactly the same commands already used throughout this
  session's `guide.md` workflow. No functional loss, just one extra
  manual step until the shared machine's package mismatch gets fixed by
  someone with sudo.

---

---

## PAPER Implementation and Supervisor Guideline

**Source paper:** Shu, Zhou & Zhang, "Agile Decision-Making and Safety-Critical
Motion Planning for Emergency Autonomous Vehicles" (IDEAM), IEEE Trans.
Intell. Transp. Syst., Vol. 26, No. 9, Sep. 2025. Code:
https://github.com/YimingShuteay/IDEAM.git

**Supervisor instruction (Qais, verbatim guideline — see `memory.md` for the
full email):** reconstruct the attached paper "with all its details (without
adding new or removing any part from it)... Use the same tools as they use,
same methods same equations, same experiments" and reproduce the same
results at the next meeting. Team split into 6 modules; **this group (V2V)
owns Module 5 — Decision-Making and Cooperative Behaviors — specifically by
reconstructing this paper.** Modules 1/2/3/4/6 are inherited from
Mr. Afreed's/Mr. Sharjeel's existing implementation (perception,
localization, comms middleware, V2X message layer, actuation) — see the
module table below. At least 2 group members must work on the paper
reconstruction, in parallel.

### Supervisor's 6-module breakdown, mapped to this project

| # | Module | Supervisor's instruction | Applies here as |
|---|---|---|---|
| 1 | Perception Layer | Take as-is from Afreed/Sharjeel | Not this group's build — integrate their output |
| 2 | Localization & State Estimation | Take as-is from Afreed/Sharjeel | AMCL/SLAM pipeline already in this repo likely qualifies — confirm with them whether to reuse this repo's `guide.md` Part 1-3 pipeline or theirs |
| 3 | Communication Middleware (ROS 2 + V2X radio) | Use a suitable ready-to-use ROS 2 topic | **Supersedes the custom transport design in section 1 above** — supervisor wants plain existing ROS 2 topics, not a bespoke UDP/JSON or Zenoh bridge. Re-evaluate Option A (shared ROS 2 domain, native topics) as the actual mandated choice, not just the fallback; the custom `transport_udp.py`/`transport_zenoh.py` design in this file may be more infrastructure than the assignment wants |
| 4 | V2X Message Layer (ETSI/SAE) | Use a suitable ready-to-use ROS 2 topic | Same note — look for an existing ROS 2 message package matching ETSI/SAE V2X conventions (e.g. CAM/BSM-like state messages) before hand-rolling `VehicleState.msg` from scratch; only design a custom message if nothing suitable exists |
| 5 | **Decision-Making and Cooperative Behaviors** | **Reconstruct the attached paper, exactly** | **This group's actual deliverable — see breakdown below** |
| 6 | Actuation Layer (Motion Control) | Take as-is from Afreed/Sharjeel | `trajectory_follower_node.py` / QCar 2's equivalent — not being rebuilt, just the thing Module 5's output commands |

### Hardware-reality check before implementing (raise with Qais, don't silently deviate)

- **Vehicle dynamics mismatch:** the paper's chassis model (Eqs. 4-10) is a
  front-steered bicycle model with Pacejka tire forces — fits **QCar 2**
  (Ackermann-steered) close to as-is. **ROSbot 3 is differential-drive** —
  it has no steering angle `δ`, so Eqs. (4)-(12) cannot be applied to it
  unmodified. Needs an explicit decision: does ROSbot 3 get its own
  MPC-DCBF instance with a substituted diff-drive kinematic model (a
  disclosed deviation from the paper), or does it instead play the role of
  a moving obstacle/HDV that QCar 2's LSGM+MPC reacts to (avoids the
  mismatch entirely, but means only QCar 2 runs the reconstructed
  algorithm)?
- **LSGM's graph is a 3-lane, 6-node structure** (Fig. 1/4/5 — `L1/L2`,
  `C1/C2`, `R1/R2`) requiring multiple simultaneous surrounding vehicles on
  a marked multi-lane road. **Correction from an earlier pass of this
  section:** the physical track is *not* a single unmarked loop — it's a
  black mat with solid left and right lane boundary lines and a center
  **dotted** line splitting two lanes, plus an **intersection** and a
  **roundabout** elsewhere on the mat. That changes the feasibility picture:
    - **Two real lanes** means a genuine (not artificially collapsed) 2-lane
      subset of the paper's per-lane structure is achievable: 2 lanes ×
      2 vehicle groups/lane = 4 nodes (vs. the paper's 6 for 3 lanes), not
      the fallback "2-node current/adjacent" reduction floated earlier.
      Much closer to "same methods, same equations" than initially assessed.
    - **Only 2 physical vehicles is still fine** — the paper's own gap
      magnitude judge (Section IV-A) explicitly excludes nodes without
      sufficient space *before* the graph search runs, so an otherwise
      sparsely-populated graph (most nodes empty, only ego + one other
      vehicle actually present) is within the paper's own design, not a
      deviation. Don't need 4+ vehicles to have a legitimate 2-lane LSGM
      instance — just need the gap-judgment step wired up honestly.
    - **The center line being dotted (legal to cross) vs. what would be a
      solid line matters for whether lane-changing is even the correct
      behavior to model there** — worth confirming aloud with Qais that
      the dotted center line is intentionally the LC/LP-eligible boundary,
      matching the paper's assumption that lane-changing is generally
      permitted between adjacent lanes.
    - **Neither the intersection nor the roundabout has any counterpart in
      this paper** — IDEAM is continuous multi-lane traffic with no
      intersections, roundabouts, stop logic, or right-of-way reasoning of
      any kind. If either is meant to be part of the Module 5
      reconstruction, that would be *adding* something not in the paper —
      flag this explicitly and get an explicit answer on whether they're
      in-scope for Module 5 at all, or purely there for another
      module/experiment (e.g. a V2X right-of-way scenario, which is a
      different problem than what this paper solves).
    - **Existing asset worth reusing:** `rosbot_lane/lane_keeping_node.py`
      already does exactly this kind of lane geometry extraction — HSV
      segmentation + Hough-line fitting for a **center dashed line (CL)**
      and **right solid line (RL)** from the camera (see `structure.md`).
      That's a plausible source for the lane-centerline curvature `κ(s)`
      and lateral deviation `e_y` the paper's Frenet kinematics (Eqs. 1-3)
      need, rather than building lane-geometry extraction from scratch —
      worth evaluating before writing a new perception path for this,
      even though Module 1 (Perception) is nominally "take as-is from
      Afreed/Sharjeel."
- **Perception input for risk assessment (Eq. 20) is ground-truth in the
  paper's simulation.** On real hardware, the natural source is exactly
  the V2V broadcast this group is building — the other vehicle's
  broadcast pose/speed stands in for the "surrounding vehicle" state the
  paper's `d_risk` math needs. This is a genuine point of synergy between
  Module 3/4 (comms) and Module 5 (decision-making), not just a gap.
- **Scale:** the paper's scenario is highway-scale (curvature up to ~0.1,
  desired speed 18 m/s). This project's track is lab/indoor scale. Metrics
  (below) are still directly reproducible at any scale; raw distance/speed
  numbers won't match the paper's, only the *relative* behavior (IDEAM
  beats baselines) should be expected to reproduce.

### Concrete equations/algorithms to reconstruct (Module 5 checklist)

- [ ] Frenet kinematics — Eqs. (1)-(3)
- [ ] Chassis dynamics + Pacejka tire model — Eqs. (4)-(10) (QCar 2 only, per mismatch above)
- [ ] Model linearization for MPC — Eqs. (11)-(12)
- [ ] Discrete Control Barrier Function (DCBF) — Def. 1, Eqs. (14)-(15)
- [ ] Discrete High-Order CBF (DHOCBF) — Def. 2, Eqs. (16)-(17)
- [ ] Linearized DHOCBF — Eqs. (18)-(19)
- [ ] C-DFS algorithm — Algorithm 1, with `UpdateMaxLevelVisited`/`LevelCheck`/`RiskAssessment`
- [ ] Risk assessment `d_risk` — Eq. (20), Fig. 4
- [ ] Long/short-term efficiency group selectors — Section IV-C
- [ ] LSGM top-level algorithm — Algorithm 2
- [ ] Longitudinal / lateral / safety-boundary / actuator constraints — Eqs. (21)-(24), (28)-(30)
- [ ] Ellipse (DHOCBF) constraint derivation via Lagrange multipliers — Eqs. (25)-(27)
- [ ] Three MPC formulations: LK (31), LP (32), LC (33)
- [ ] Same solver stack: `cvxpy` + ECOS solver, Python

### Experiments/metrics to reproduce (Module 5 checklist)

- [ ] Progress at 20s/40s + max progress (Table III cols 1-3)
- [ ] Avg/max velocity (Table III cols 4-5)
- [ ] Avg-min / min safety distance `S_o` (Table III cols 6-7)
- [ ] Max/avg acceleration, avg jerk (Table III cols 8-10)
- [ ] Lane-change intention success count, avg/max (Table IV)
- [ ] Computation time breakdown: solver / LSGM / C-DFS (Fig. 7)
- [ ] Baseline comparisons — at minimum reproduce the "No-Probing IDEAM"
      ablation (same system, motion planner without the lane-probing
      state) since that isolates the paper's own headline contribution;
      other baselines (SO-DM, DRB-FSM, MOBIL) are lower priority given
      they're reimplementations of *other* papers, not IDEAM itself
- [ ] Case study scenarios: one "evaluation" scenario (normal traffic,
      Section VI-D-1) and the two "emergency" scenarios (sudden
      leader-vehicle deceleration, Section VI-D-2, Fig. 11)

### Suggested work split (≥2 people per supervisor's instruction)

- **Person A — Motion planner:** vehicle dynamics + linearization, DCBF/DHOCBF constraint formulation, the three MPC optimization problems (31)-(33), solver integration (cvxpy/ECOS). Primarily QCar 2-side given the Ackermann-model fit.
- **Person B — Decision layer:** LSGM/C-DFS graph search, risk assessment, long/short-term efficiency selectors, the (scaled-down, disclosed) vehicle-group graph structure feeding off the V2V broadcast for peer state.
- **Shared:** metrics/logging harness reproducing Table III/IV/Fig. 7 exactly, so results are directly comparable to the paper's numbers at the next meeting.

### 6. Other, smaller housekeeping (non-V2V)

- [ ] `core/obstacle_tracker.py` is currently unused dead code (see
      `structure.md`) — decide whether to wire it in, or delete it, before
      it goes stale further.
- [ ] `pandas` isn't installed on `rosbot-server`'s Python but
      `config/smooth.py` needs it — confirm environment before next
      recording/smoothing session (see README's Environment/Versions note).
- [ ] Decide whether `config/track_map.data` / `.posegraph` (14MB/21MB,
      already tracked in git) should move to Git LFS as the repo grows.
