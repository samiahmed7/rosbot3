# TODO / Future Work

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
- [ ] **Map-frame alignment between the two robots — real, unresolved.**
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
