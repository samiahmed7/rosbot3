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

### 6. Other, smaller housekeeping (non-V2V)

- [ ] `core/obstacle_tracker.py` is currently unused dead code (see
      `structure.md`) — decide whether to wire it in, or delete it, before
      it goes stale further.
- [ ] `pandas` isn't installed on `rosbot-server`'s Python but
      `config/smooth.py` needs it — confirm environment before next
      recording/smoothing session (see README's Environment/Versions note).
- [ ] Decide whether `config/track_map.data` / `.posegraph` (14MB/21MB,
      already tracked in git) should move to Git LFS as the repo grows.
