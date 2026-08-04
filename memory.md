# ROSbot 3 Server Migration & Debugging Log (Session 2)
**Date:** July 9, 2026
**Environment:** `sami@Inspiron-15` (Local laptop), cross-checked against `rosbot-server` (GPU server) and the ROSbot itself at `192.168.0.110`
**Continues from:** `gemini-code-1783627443304.md`, last entry `[22:00 CEST] TF Tree & Frame Resolution Diagnosis`

> Timestamps are reconstructed from conversation order; `22:14 CEST` is the one directly observed value (from a `ps aux` timestamp). All others are approximate.

### Timeline of Operations

* **[~22:05 CEST] Discovery Env Vars Deleted**
    * **Issue:** Ran the following against `~/.bashrc`, then unset both in the live shell:
      ```bash
      sed -i '/ROS_AUTOMATIC_DISCOVERY_RANGE/d' ~/.bashrc
      sed -i '/ROS_STATIC_PEERS/d' ~/.bashrc
      unset ROS_AUTOMATIC_DISCOVERY_RANGE
      unset ROS_STATIC_PEERS
      ```
    * **Effect:** `ros2 topic list` stopped returning any remote topics immediately after.

* **[~22:08 CEST] Root Cause Recovery**
    * **Action:** Inspected `~/.bashrc` directly and confirmed both lines were gone. Also noted:
        * `FASTDDS_BUILTIN_TRANSPORTS` is set twice (`UDPv4` then `LARGE_DATA`) — the first assignment is dead, only `LARGE_DATA` is active.
        * `~/.bash_aliases` does not exist — the `--no-daemon` aliases mentioned in Session 1's `[20:15 CEST]` entry were never actually persisted to disk; they were one-off manual invocations.
    * **Recovery:** Pulled the deleted values back out of `~/.bash_history`:
      ```bash
      export ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
      export ROS_STATIC_PEERS="192.168.0.110"
      ```
    * **Diagnosis:** `ROS_AUTOMATIC_DISCOVERY_RANGE=OFF` fully disables Fast DDS's multicast-based SPDP discovery — it's not a "restriction," it's a full switch-off. With it set, `ROS_STATIC_PEERS` becomes the *only* path to discovering the robot. Deleting both left zero discovery path, explaining the silent hang (this was almost certainly the actual fix behind Session 1's `[20:15 CEST]` FastDDS daemon freeze entry, not the `--no-daemon` aliases).

* **[~22:10 CEST] Fix Applied**
    * **Action:** Re-added both `export` lines to `~/.bashrc` and sourced it.

* **[~22:12 CEST] `ros2 daemon stop` Hang**
    * **Issue:** Command hung indefinitely.
    * **Diagnosis:** A daemon likely auto-respawned during earlier bare `ros2 topic list` calls (any `ros2` CLI call without `--no-daemon` spawns one), inheriting the broken discovery env and wedging inside a graph-discovery call — leaving it unable to service the stop IPC request.
    * **Action:** Ctrl+C the hang, `pkill -9 -f _ros2_daemon`, and use `--no-daemon` directly instead of waiting on a graceful stop.

* **[22:14 CEST] Process Check**
    * `ps aux | grep -i ros2` confirmed no daemon process survived — nothing left to kill, clear to retest discovery directly.

* **[~22:15 CEST] Discovery Retest — Still Failing**
    * `ros2 topic list --no-daemon --spin-time 2.0` → only `/parameter_events`, `/rosout` (these are the querying CLI's own default topics, not evidence of successful remote discovery).
    * Confirmed in-shell: `ROS_AUTOMATIC_DISCOVERY_RANGE=OFF`, `ROS_STATIC_PEERS=192.168.0.110` both correctly restored and live.
    * `ping -c 2 192.168.0.110` succeeded (3.98–4.14 ms) — ruled out basic L3/ICMP reachability as the blocker.

* **[~22:17 CEST] Further Isolation Attempt**
    * `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` both unset/empty on the laptop (i.e. defaults on both — not a mismatch by itself, but unconfirmed on the robot/server side).
    * SSH attempt to the robot failed — used a literal `<robot-user>` placeholder instead of the real username.
    * `ros2 daemon status` hung again — noted as non-blocking, since `--no-daemon` bypasses the daemon entirely for the actual discovery test.

* **[~22:19 CEST] Reframing the Test**
    * Flagged that `/parameter_events` + `/rosout` alone is the *expected* output even with fully correct discovery config, if nothing is actually running on the robot at that moment. Needed to confirm the robot side was live before concluding discovery itself was broken.

* **[~22:20 CEST] Resolution — Isolated to Laptop's Network Path**
    * User checked `rosbot-server` (GPU server) directly: `ros2 topic list` returned real robot topics there without issue.
    * **Conclusion:** The robot's driver stack is healthy and publishing correctly. The discovery break is specific to the *laptop's* network path to `192.168.0.110` (most likely WiFi client isolation or multicast/unicast filtering on that access point) — not the robot, and not the DDS config logic itself.
    * **Decision:** Continue the working session on `rosbot-server`, the original intended migration target. Parked the laptop-specific discovery quirk as a known limitation rather than a blocker.

### Open Items / Next Steps

- Resume the TF tree / `/amcl_pose` diagnosis on `rosbot-server` (Session 1's last unresolved thread).
- Check whether `rosbot-server`'s own `~/.bashrc` needs its own `ROS_STATIC_PEERS` entry pointing at the robot, or whether it's working via plain multicast (would explain why the laptop needed the workaround and the server doesn't).
- Laptop-only, low priority: revisit `ROS_AUTOMATIC_DISCOVERY_RANGE=OFF` / `ROS_STATIC_PEERS` only if standalone laptop-to-robot operation (without the server) is needed again later.
- Cosmetic: dedupe the two `FASTDDS_BUILTIN_TRANSPORTS` exports in `~/.bashrc` (lines currently ~123-124) — harmless today but confusing to read.

### Key Files Touched

- `~/.bashrc` (laptop) — restored `ROS_AUTOMATIC_DISCOVERY_RANGE` and `ROS_STATIC_PEERS` export lines.

---

# ROSbot 3 Firmware 2.0 Migration & Path-Recording Debugging Log (Session 3)
**Date:** July 10, 2026
**Environment:** `rosbot-server` only (laptop-side DDS issue from Session 2 not revisited)
**Continues from:** Session 2's open item "Resume the TF tree / `/amcl_pose` diagnosis on `rosbot-server`"
**Context:** Robot was flashed to Husarion firmware 2.0 / `rosbot_ros` `jazzy` branch since Session 2. This is a different class of change from Session 2 — not a network/discovery issue, but the firmware upgrade silently breaking topic names and localization assumptions the existing config relied on.

> Timestamps below `00:43` onward are directly observed (converted from ROS log epoch stamps seen in terminal output); earlier ones are reconstructed from conversation order.

### Timeline of Operations

* **`scan_filtered` → `scan` topic migration**
    * **Issue:** Robot wouldn't move when running `trajectory_follower_node.py`. Diagnosed via `ros2 topic info /rosbot3/scan_filtered --verbose` → **0 publishers**. Firmware 2.0 only publishes raw `/rosbot3/scan`, not a filtered topic — something (likely a `laser_filters` self-filter node) that used to exist upstream of AMCL/lane-following isn't present in this firmware/software combination.
    * **Effect chain:** AMCL never receives scans → never broadcasts `map→odom` → `map` frame never exists → `trajectory_follower_node.py`'s `_update_pose()` (`map→base_link` TF lookup) always fails → node sits forever in `WAITING_FOR_LOCALIZATION`, publishing zero velocity. No error, just silent stillness.
    * **Fix — 4 files changed** to use `/rosbot3/scan` instead of `/rosbot3/scan_filtered`:
        - `config/amcl_params.yaml` (`scan_topic`)
        - `config/lane_params.yaml` (`topics.scan`)
        - `rosbot_lane/trajectory_follower_node.py` (`create_subscription(LaserScan, ...)`)
        - `rosbot_lane/core/config.py` (the `.get('scan', ...)` fallback default)
    * **Known risk accepted:** raw `/rosbot3/scan` includes the robot's own chassis self-returns that a filter would normally strip. Not yet an observed problem, but flagged in `README.md`/`guide.md` as something to watch for (jittery AMCL localization, or false-positive obstacle stops right in front of the robot).

* **`config/slam_trajectory_recorder.py` TF warning — false alarm**
    * `[WARN] ... TF lookup failed: "map" ... does not exist` seen right after launch.
    * **Diagnosis:** Ordinary startup race — the recorder starts polling `map→base_link` immediately, but `slam_toolbox` needs a moment to publish its first `map→odom` transform after receiving its first scan. Confirmed resolved within seconds (recorder was already writing points to `slam_trajectory.csv` by the time it was checked). Documented in `guide.md`'s troubleshooting section as expected/benign *unless* it never stops.

* **`00:49:39` — `map_saver_cli` + `serialize_map` — half-silent failure**
    * User ran both commands as a multi-line paste; `map_saver_cli` succeeded (`.pgm`/`.yaml` updated), but `.data`/`.posegraph` stayed 2 days stale.
    * **Root cause:** the multi-line `serialize_map` service call's closing `"` was missing in the actual terminal paste, leaving bash sitting at a `>` continuation prompt — the command never executed at all. No error was ever printed because the command never ran.
    * **Verification method going forward:** don't trust "no error" as proof of a write — check `ls -la config/track_map.data config/track_map.posegraph` timestamps against *now*.
    * `guide.md` updated to put the `serialize_map` call on one line (avoids the trap) and added an explicit "verify it actually wrote" callout.

* **`00:55:52` — Confirmed working** — re-ran both commands correctly (one-liner), `.data`/`.posegraph` timestamps matched immediately. Map/posegraph save flow is good from here on as long as the one-line form is used.

* **`01:06:46`–`01:13:41` — AMCL convergence walkthrough**
    * `[WARN] AMCL cannot publish a pose...` — expected at this stage, not a bug: `amcl_params.yaml` has `set_initial_pose: false`, so AMCL won't guess on its own.
    * Ran `ros2 service call /reinitialize_global_localization std_srvs/srv/Empty` → particles spread across the whole map.
    * Covariance had to be walked down across several re-checks before it was trustworthy: `9.7/4.1/5.4` → `4.0/4.3/1.4` → **`0.06/0.006/0.03`** (converged). Established the reference: diagonal covariance indices `[0]` (x), `[7]` (y), `[35]` (yaw) in the flattened 6×6 matrix; target roughly `<0.2` for x/y, `<0.1` for yaw. A single in-place wiggle isn't enough — needs real motion through distinguishing track geometry (turns), not just rotation in place.

* **`config/smooth.py` — missing `pandas`, no sudo available**
    * `rosbot-server`'s system Python is externally-managed (PEP 668, Ubuntu Noble) and this is a university machine — no sudo, so `apt install python3-pandas` and plain `pip install` were both off the table.
    * **Fix:** created a dedicated venv (`python3 -m venv .venv`, then `.venv/bin/pip install pandas numpy scipy`) scoped only to `smooth.py`, since it's a standalone script with no `rclpy` dependency. Added `.venv/` to `.gitignore`.
    * **Follow-on trap hit immediately after:** user ran `source .venv/bin/activate` in the same shell they then tried to run `trajectory_follower_node.py` in → `ModuleNotFoundError: No module named 'yaml'`. Cause: the venv is deliberately isolated from system `dist-packages`, which is where `rclpy`'s own dependency `PyYAML` actually lives (`rclpy` itself was still found only because `PYTHONPATH` explicitly points at `/opt/ros/jazzy/...`, a different directory than `dist-packages`). **Rule going forward:** `.venv/bin/python3 config/smooth.py` (no activation) for smoothing; plain `python3 ...` in a *non-activated* shell for every ROS node. Documented prominently in `guide.md`.

* **`01:30:55`–`01:33:20` — First successful full-lap run** on the newly recorded/smoothed path. All 5 segments completed, then looped into lap 2 automatically (`_handle_complete()` resets to `WAITING_FOR_LOCALIZATION` by design).
    * **Anomaly noticed:** ~24s (lap 1) then ~38s (lap 2, worse) spent oscillating right at the `Segment(REV: WP 91 → 93)` transition — goal distance repeatedly growing/shrinking instead of cleanly converging.

* **Root-caused the oscillation — traced to raw recording noise, not real driving:**
    * Compared `smoothed_trajectory.csv` waypoints 91–93 against the nearest raw `slam_trajectory.csv` rows (idx 165–172). Found the whole raw stretch is a smooth, continuous curve (heading drifting steadily, ~2.5cm steps) **except** raw index 168, which jumps ~205° off the trend for exactly one frame then snaps straight back — a single-frame SLAM scan-matching glitch, not the driver pausing/backing up.
    * Searched the rest of the raw recording for the same signature and found 3 more (idx 613, 651, 795), each an isolated single-frame outlier embedded in an otherwise smooth arc. **All 4 of the original "flip" segments (WP91, 93, 355, 431) turned out to be artifacts of these glitches — the actual recorded track has zero intentional reversals.**
    * **Fix, both added to `config/smooth.py`:**
        1. A despike pre-pass: drops any raw point where going through it is a "detour" (neighbors are much closer to *each other* directly than to it — `detour_ratio` default `2.0`).
        2. A short-segment merge pass (kept as a second safety net): any post-flip-detection segment under `0.15m` gets folded into its previous neighbor, in case future noise isn't a clean single-point spike.
    * **Verified independently** — didn't just trust `smooth.py`'s own segment count; re-loaded the output through `rosbot_lane/core/trajectory.py`'s `Trajectory` class directly (the same code path `trajectory_follower_node.py` actually uses at runtime) and confirmed it now reports a single 540-waypoint forward segment, no flips.

* **Re-recorded the map/path** (fresh SLAM session, `config/track_map.*` and `config/slam_trajectory.csv` all updated), re-ran the fixed `smooth.py` against it.

* **Git: committed and attempted push to two remotes — blocked on missing credentials, both times.**
    * `git commit` initially failed with `Author identity unknown` — no `user.name`/`user.email` configured on this machine; per this assistant's Git Safety Protocol, git config is never set by the assistant even with permission — user had to run `git config` themselves.
    * `git push origin main` (GitLab, `gitlab.tu-ilmenau.de/qayo2953/opta2.git`) and `git push github main` (new remote added, `github.com/samiahmed7/rosbot3.git`) both fail with `could not read Username` — no credential helper, no SSH key, no `gh` CLI installed. Assistant's Bash tool calls are non-interactive, so a password/token prompt can't be relayed through it — **user has to run the push themselves** in a real terminal (GitLab: TU credentials; GitHub: username + personal access token as the password).
    * `github` remote's `main` was already in sync with local history at the time it was added (same commit `105bfc2`), so pushes there should fast-forward cleanly once auth is sorted.

* **Aside — cup-holder-induced jerking, diagnosed as battery, not code:** `/rosbot3/battery` showed `10.22V` / `20%` / `3.41V per cell` (3S pack) — low enough to sag hard under the extra current draw from the added weight, plausibly causing the motor controller's brownout protection to briefly cut torque. Recommended charging first before chasing a software/gain-tuning explanation. Not yet confirmed fixed — no re-check after charging logged in this session.

* **`ROS_DOMAIN_ID` changed 1 → 0.** `~/.bashrc` had `export ROS_DOMAIN_ID=1` hardcoded (line 127, origin/reason not recorded — possibly leftover from earlier laptop-side work, since Session 2's laptop debugging happened around the same discovery-config area). User changed it to `0` and confirmed in their own interactive terminal that `ros2 topic list` now shows the full robot topic set correctly — domain `0` (ROS 2's actual default when nothing is set) reaches the robot's driver stack fine.
    * **Tooling note for future sessions:** this assistant's own Bash tool shell did **not** pick up the `.bashrc` change even after explicitly re-sourcing it (`bash -lc ...`) — `ros2 topic list` kept showing only `/parameter_events`/`/rosout` from here, while the user's real terminal showed everything. Root cause: Ubuntu's stock `.bashrc` has an early `case $- in *i*) ;; *) return;; esac` guard (line 8) that makes it bail out immediately for any non-interactive shell — which is what every Bash-tool call is, even with `-l`. So `ROS_DOMAIN_ID` (and anything else set later in `.bashrc`) will never be visible to this assistant's shell mid-session no matter how it's invoked; only the user's actual terminal reflects `.bashrc` edits. Don't mistake that gap for a real discovery problem next time.

* **"No sound coming from the car" — traced to an architecture mismatch, not a bug.** `trajectory_follower_node.py`'s `_play_sound()` (`obstacle.wav`/`pickup.wav`/`delivery.wav`) shells out to local `subprocess.run(['aplay', ...])`. Since the node runs on `rosbot-server`, that plays through **the server's own sound card** (`card 0: PCH [HDA Intel PCH]`, confirmed present, confirmed `aplay` exits `0` successfully) — never through the robot, because there's no ROS topic/service in this codebase that sends audio to the robot at all.
    * Investigated fixing it via SSH-triggered remote playback on the robot. SSH username is `husarion` (not `rosbot3` — that was tried first and rejected; `husarion` worked once the password was supplied interactively in the user's own terminal, standard non-interactive-Bash-tool limitation same as the earlier git push issue).
    * **Turned out moot: `aplay -l` on the robot (`husarion@rosbot3`) shows only `vc4hdmi0`/`vc4hdmi1`** — the Raspberry Pi's built-in HDMI audio outs. No analog jack, no USB audio device. **The robot has no speaker hardware at all**, so remote playback wouldn't produce any audible sound regardless of how it's wired up.
    * **Decision:** leave `_play_sound()` as-is — server-side audio is being kept intentionally as an operator-side debug/feedback cue, not something meant to come from the robot. Not filed as a TODO item (per explicit instruction) since it's not something being pursued right now; if on-robot audio cues are wanted later, the real prerequisite is physically attaching a USB speaker to the Pi first — no code fix alone can produce that.

### Open Items / Next Steps

- **Push still pending** — commit `8b2e68a` is sitting locally; user needs to run `git push origin main` and `git push github main` themselves with credentials.
- Confirm whether raw `/rosbot3/scan` (no filter) causes any real problems over longer runs — chassis self-returns were a known risk accepted, not yet observed as an actual issue.
- Never re-checked `/rosbot3/battery` after a charge — cup-holder jerking diagnosis (low battery voltage sag) unconfirmed as the actual fix.
- V2V communication with Quanser QCar 2 — full design breakdown written to `TODO.md`, nothing implemented yet; blocked on hardware access to confirm QCar 2's actual ROS 2/SDK situation.
- `pandas`/`numpy`/`scipy` in `.venv/` are pinned to whatever `pip` resolved at install time (pandas 3.0.3, numpy 2.5.1, scipy 1.18.0) — notably newer than the system apt-installed `numpy`/`scipy` versions used elsewhere in this environment; not expected to matter since the venv is scoped to `smooth.py` only, but worth remembering if that scope ever expands.

### Key Files Touched

- `config/amcl_params.yaml`, `config/lane_params.yaml`, `rosbot_lane/trajectory_follower_node.py`, `rosbot_lane/core/config.py` — `/rosbot3/scan_filtered` → `/rosbot3/scan`.
- `config/smooth.py` — fixed output path bug (was writing to repo root instead of `config/`), added despike pre-pass and short-segment merge.
- `config/slam_trajectory.csv`, `config/smoothed_trajectory.csv`, `config/track_map.{pgm,yaml,data,posegraph}` — fresh recording.
- `.venv/` (new, gitignored) — isolated environment for `smooth.py`'s `pandas`/`numpy`/`scipy`.
- `.gitignore` — added `__pycache__/`, editor swap files, `.venv/`.
- `README.md`, `guide.md`, `structure.md` (new pipeline-architecture section), `TODO.md` (new) — documentation added/updated to match all of the above.
- `~/.bashrc` (`rosbot-server`) — `ROS_DOMAIN_ID` changed `1` → `0`.

---

# Project Context: Course/Lab Structure & Supervisor Guideline (V2V Group)

**Date logged:** July 11, 2026

This project is part of a supervised course/lab (supervisor: **Qais**)
with multiple groups working on connected/autonomous-vehicle topics. The
user (this repo's owner) is a member of the **V2V group**, working
alongside at least one other named teammate on this specific sub-team.
A parallel/sibling group is doing **V2X**. Two other named individuals —
**Mr. Afreed** and **Mr. Sharjeel** — own a separate implementation
(perception, localization, and actuation) that this group is instructed to
reuse rather than rebuild. This group's teammate on the Quanser QCar 2 side
has already implemented V2V communication independently; **that code has
not yet been merged with this repo** as of this entry.

Each group's project is organized into 6 modules. Supervisor's guideline
(verbatim, received by email):

> Dear V2V and V2X groups,
>
> To make better progress, I have the below suggestions, as your projects
> are constituted of the 6 main modules below, you may do the
> implementation following the below plan.
>
> 1. Perception Layer (Sensors & Environment Understanding): Take it as it
>    is from Mr. Afreed and/or Mr. Sharjeels implementation.
> 2. Localization and State Estimation: Take it as it is from Mr. Afreed
>    and/or Mr. Sharjeels implementation.
> 3. Communication Middleware (ROS 2 + V2X Radio): Use a suitable
>    ready-to-use topic from ROS2.
> 4. V2X Message Layer (ETSI / SAE Standards): Use a suitable ready-to-use
>    topic from ROS2.
> 5. Decision-Making and Cooperative Behaviors: Reconstruct the attached
>    paper.
> 6. Actuation Layer (Motion Control): Take it as it is from Mr. Afreed
>    and/or Mr. Sharjeels implementation.
>
> Kindly find the attached papers study and understand deeply. Then
> reconstruct it with all its details (without adding new or removing any
> part from it) on your Robot or/and car. Use the same tools as they use,
> same methods same equations, same experiments. every step exactly the
> same. It is expected that in our next meeting you show me that you
> obtained the same results for all the experiments they mentioned in this
> paper.
>
> Kindly destribute the work between you in a balanced way, so that you
> work in parallel. in each team, you need to consider at least 2 members
> for the paper reconstruction.
>
> BR
> Qais

**The attached paper** (Module 5's reconstruction target): Shu, Zhou &
Zhang, "Agile Decision-Making and Safety-Critical Motion Planning for
Emergency Autonomous Vehicles" (IDEAM), IEEE Trans. Intell. Transp. Syst.,
Vol. 26, No. 9, Sep. 2025.

**Why this matters for how this repo develops going forward:** the
existing V2V design drafted in `TODO.md` (custom `v2v_msgs`/`v2v_comm`
packages, UDP/Zenoh transport options) was written *before* this
guideline arrived, and partially conflicts with it — Qais's instruction
for modules 3/4 is "use a suitable ready-to-use topic from ROS2," which
points toward reusing an existing ROS 2 message package rather than the
bespoke transport/message design already in `TODO.md`. That section needs
revisiting against this guideline, not just extended. Full reconciliation
(paper equations/algorithms mapped to feasibility on ROSbot 3 vs. QCar 2,
work split, checklist) is written into `TODO.md` under "PAPER
Implementation and Supervisor Guideline."

**Known hardware-driven conflict with "reconstruct exactly, add/remove
nothing":** the paper's vehicle dynamics model (Section III-A) assumes
Ackermann steering (fits QCar 2), but ROSbot 3 is differential-drive with
no steering angle — the model cannot be applied to ROSbot 3 unmodified.
These are flagged as things to raise with Qais rather than silently
work around — see `TODO.md` for the proposed (disclosed) adaptations.
User confirmed they'll clarify these points with Qais directly.

**Correction to the physical track description** (an earlier pass of this
analysis incorrectly assumed a single unmarked loop): the actual track is
a black mat with solid left/right lane boundary lines, a **dotted center
line** splitting two lanes, and a separate **intersection and roundabout**
elsewhere on the mat. This makes a genuine 2-lane (not artificially
collapsed) subset of the paper's LSGM graph structure achievable — 2 lanes
× 2 vehicle groups/lane = 4 nodes — and the paper's own gap-magnitude
judgment (which excludes empty nodes before the graph search) means 2
physical vehicles is enough to legitimately populate a sparse instance of
that graph, not a blocker. Neither the intersection nor the roundabout has
any counterpart anywhere in the IDEAM paper (it's continuous multi-lane
traffic, no intersection/roundabout/right-of-way logic) — open question
for Qais on whether either is in-scope for Module 5 at all.
Full detail in `TODO.md`'s "Hardware-reality check" section, including a
note that `rosbot_lane/lane_keeping_node.py`'s existing CL/RL line
detection (already built for exactly this dashed-center/solid-side lane
pattern) may be directly reusable for the paper's Frenet lane-geometry
inputs rather than building that extraction from scratch.

---

# `lane_keeping_node.py` Debugging Saga + V2V Code Pull (Session 4)
**Date:** August 4, 2026
**Environment:** `rosbot-server`, `.venv-gui` (new — see below)

This was one long, iterative debugging session on the camera-based
lane-keeping pipeline (`lane_keeping_node.py` + `core/{detection,geometry,
config,debug}.py`) against the real 2-lane track (solid outer lines,
dotted center, intersection, roundabout — see the "PAPER Implementation"
project-context entry above), followed by pulling a teammate's V2V code.
Ordered by what was actually found, since each fix mostly surfaced the
next failure mode rather than converging immediately.

### How to actually run/debug this node

- Correct invocation: `python3 -m rosbot_lane.lane_keeping_node` from the
  repo root (**not** `python3 rosbot_lane/lane_keeping_node.py` directly —
  that puts the wrong directory on `sys.path` and `from rosbot_lane.core...`
  fails). No colcon build needed; `rosbot_lane/__init__.py` makes it a
  discoverable package from cwd.
- `cv2.imshow` crashed outright on the system OpenCV (`4.13.0`, a
  from-source build at `/usr/install/opencv-4.13.0/` with **no GUI backend
  compiled in** — not a DISPLAY/X11 problem, the library itself can't
  render a window). Confirmed there's a real local X11 session on this
  machine (KDE Plasma, `tty2`, `DISPLAY=:1`) worth actually using, so
  built a **separate venv, `.venv-gui`** (`python3 -m venv --system-site-packages
  .venv-gui`, then `pip install opencv-python`) — `--system-site-packages`
  so `rclpy`/`cv_bridge` still resolve. Run GUI-needing sessions with
  `.venv-gui/bin/python3 -m rosbot_lane.lane_keeping_node`, same
  don't-`source`-`activate` rule as the original `.venv`.
  - Pip resolved `opencv-python` to `5.0.0.93` by default — a major-version
    jump from the system's `4.13.0` that changed `cv2.HoughLinesP`'s return
    array shape (`(N,4)` vs the legacy `(N,1,4)` this codebase's unpacking
    assumes), crashing `detect_cl` with `TypeError: cannot unpack
    non-iterable numpy.int32`. Fixed by pinning
    `pip install "opencv-python==4.13.0.92"` — match the pip version to the
    system version being replaced, don't just take latest.
- **This session's own Bash tool shell repeatedly, silently reverted to
  having a stale `.venv`/`.venv-gui` active mid-session** (`VIRTUAL_ENV`
  set, wrong `python3` on `PATH`), causing several confusing false-negative
  results (e.g. `ModuleNotFoundError: No module named 'cv2'` from the
  *system* python). Root cause unclear (never definitively found — no
  `.bashrc` line, no direnv), but the practical fix every time was to
  explicitly `unset VIRTUAL_ENV` and rebuild `PATH` by hand before trusting
  any check. Don't trust `which python3` implicitly in a long session here.

### Real bugs found and fixed, in order

1. **`detect_cl` had no sliding-window fallback, unlike `detect_rl`.**
   `detect_rl` only used Hough segments to seed a starting x, then did a
   robust pixel-density sliding-window sweep (tolerant of gaps).
   `detect_cl` trusted Hough segments *directly* as the final detection —
   fine for a continuous solid line, badly unreliable for the dashed
   center line, since individual dash fragments are short/gapped and
   routinely fail Hough's own length/vote thresholds. Fixed by giving CL
   the same seed-then-sliding-window treatment as RL. Removed now-dead
   `_pick_best_line`/`eval_line_at_y`/the `Line` type alias as a result.
2. **`cl_line` truthiness crash, latent until fix #1 made CL actually
   detect things.** `if cl_line:` on a NumPy array (the new polynomial
   return type) raises `ValueError: truth value of an array... is
   ambiguous` instead of behaving like a `None` check. Changed to
   `is not None` throughout (matching how RL was already written).
3. **Corridor masking (`apply_corridor_mask`) had no reset-on-failure.**
   Once a line was detected, its position was remembered forever as a
   search-corridor anchor — even after the line was later lost. As the
   robot moved, the stale anchor stopped matching the real line's
   position, permanently excluding it from all future frames (the root
   mechanism behind every "drives fine, then permanently stops" pattern
   seen this session). Fixed: clear a line's corridor reference the
   instant that line fails to detect, not only ever set it.
4. **`compute_lane_error_cl_only` didn't exist — only an RL-only
   fallback did.** CL/RL are assigned by *which side of frame-center* a
   detected line lands on, not by whether it's physically the dashed or
   solid line — on a sharp turn the robot's heading can rotate enough
   that the solid line swings into the "CL" slot and the dashed line into
   "RL". Treating RL as always-reliable was wrong. Added a mirrored
   `compute_lane_error_cl_only` (with an unverified `CL_OFFSET_PX = 165.0`
   guess, same caveat as the pre-existing `RL_OFFSET_PX`) and made
   `lane_keeping_node.py` drive off *whichever* line is actually detected,
   only treating it as truly lost when both are gone. Also fixed the
   `_no_detection_count` stop-warning: the message said "Both lane lines
   not detected" but the code's condition was `or`, not `and` — it fired
   on losing *either* line, not both; the wording was just misleading.
5. **`lane_params.yaml` had the wrong camera resolution the entire
   time.** Configured `768×432`; the OAK camera's real `camera_info`
   (queried live) is `640×400`. Every frame-center-relative calculation
   (`cx = img_width/2`, corridor bounds, the steering error's
   `img_width/2` reference) was silently using a center **64px right of
   the true center**. Fixed to `640×400`. Also measured the real FOV from
   the intrinsics while there: 66.6° horizontal / 44.6° vertical — a
   normal range, not the actual bottleneck the resolution bug was.
6. **`kp: 0.30` was mathematically too weak to ever reach a tight turn
   radius**, independent of any detection bug. `LaneController.compute()`
   clips to `max_angular` (1.5 rad/s, hardcoded, not in config), but with
   `error` clipped to `[-1,1]`, max possible output was `kp * 1.0 = 0.30`
   rad/s — nowhere near that clamp. At `drive_speed=0.20`, that floors the
   tightest possible turn radius at `v/ω ≈ 0.67m`; any real curve tighter
   than that causes structural understeer. Raised to `kp=1.00` (→ floor
   `~0.2m`). This is the same *class* of bug as a finding in the
   teammate's V2V work (see below) — a controller physically incapable of
   the turn radius a path demands, silently producing bad tracking
   instead of an obvious error.
7. **White walls pass the same HSV segmentation as the white line**, and
   are close enough to the track on one side to matter. A `_sliding_window`
   poly fit pulled toward wall pixels could extrapolate to physically
   impossible positions (observed: `cl_bot_x ≈ -252` to `-267`, hundreds
   of px off a 640px-wide frame) and still get accepted as a confident
   detection — clipped to `error = 1.0000` (max), i.e. a full-strength
   false steering command from garbage data. Fixed: reject a
   `_sliding_window` fit if its bottom-of-strip position falls outside
   `[-0.3w, 1.3w]`.
8. **Raising `kp` (fix #6) made the controller newly sensitive to
   noisy-but-not-extreme detections that fix #7's outlier rejection
   doesn't catch** (e.g. `cl_bot_x=-96` to `-159`, not extreme enough to
   reject, but still noise) — strong enough now to genuinely oversteer
   into the other lane on a single bad frame. Added exponential smoothing
   (`smoothing_alpha=0.4`) on `error` before it reaches the controller, so
   one noisy frame can't produce a full-strength commit while a real,
   sustained curve (large error across many frames) still comes through.
9. **The per-line corridor fix (item 3) was itself too aggressive once
   actually tested.** When only RL was tracked, the corridor restricted
   search to a narrow band *around RL only* — accidentally excluding the
   entire region where CL naturally lives, including on straight
   sections, making CL permanently unrecoverable once lost even once.
   ("not detecting the left striped lane in a straight lane" — reported
   directly, confirmed from saved debug frames.) Fixed: a tracked line's
   corridor now only excludes the region *beyond* it (away from the other
   line), not the entire opposite side of frame — preserves the original
   wall/noise protection while no longer blocking legitimate reacquisition
   of a lost line.
10. **Duplicate-line-on-a-curve, found by actually looking at saved
    frames (see below).** On a sharp curve with only one real boundary
    line in view, that single curve can cross from left-of-center to
    right-of-center within the strip — `detect_cl` and `detect_rl` then
    each independently seed onto *the same physical line* and both report
    a "detection". `compute_lane_error` averaging two copies of one line
    produces a fake "lane center" and a large false correction — the
    likely actual mechanism behind at least one "went off on the curve"
    failure. Fixed in `fit_lane_lines`: if both detected lines land
    implausibly close together (`< min_line_gap_px=80`), collapse them
    into a single line instead of trusting both as independent.

### Wiring up `cv2.imwrite` debug capture (a real turning point)

After enough rounds of inferring root causes purely from `lane_log.csv`
statistics, wired up actual saved-frame capture instead of continuing to
guess blind: `draw_debug_frame` now *returns* `(vis, bottom_row)` instead
of only calling `cv2.imshow` internally; `lane_keeping_node.py` saves both
via `cv2.imwrite` to `debug_frames/` (new, gitignored, auto-cleared on
every node startup) — every no-detection frame (the actual failure
moments) plus a periodic cadence otherwise. New `lane_params.yaml`
`debug:` keys: `save_frames`, `save_every_n_frames`, `save_dir`.

This directly found bugs #9 and #10 above by letting the raw binary mask
(the *unannotated* right-hand panel of each `*_mask.jpg`) be inspected
directly — confirmed in one case a single connected curve being detected
as "BOTH LINES", and in another the wall filling nearly the entire frame
with only a sliver of real track surviving. Worth reaching for early next
time rather than late — most of this session's back-and-forth guessing
from CSV stats alone could have been shortcut by this.

### Config parameter notes (asked about directly, worth remembering)

- `corridor_px` — the per-line search-corridor margin (see bugs #3, #9).
- `split_fraction` — **purely cosmetic**, only draws a debug-view divider
  line; despite living under `detection:` in the YAML, does not affect
  detection at all.
- `lane_width_px` — **passed into `compute_lane_error` but never actually
  used in its body.** Vestigial from an earlier version of the error
  calculation. Not fixed/removed, just flagged — don't waste time tuning
  it.

### V2V code pulled from a teammate's branch

Pulled `rosbot_v2v_broadcaster.py` and `rosbot_v2v_gate.py` from
`github.com/6hammad9/qcar2-lane-keeping` (branch
`fix/lidar-curve-corridor`) into `rosbot_lane/`. Full detail (architecture,
deployment, findings, open items) written into `TODO.md`'s new
"Actual V2V Implementation" section — summary here:

- Confirmed by diff that the teammate's `rosbot_opta2/` mirror of the core
  lane-keeping files (`detection.py`, `geometry.py`, `config.py`,
  `lane_keeping_node.py`, `lane_params.yaml`) is an **older snapshot that
  predates this entire session's fixes** (still has the `768×432`
  resolution bug, `kp: 0.30`) — nothing to merge back from those, don't
  overwrite local copies with them.
- The two V2V files are genuinely new and were the actual point of the
  pull: an asymmetric UDP-based V2V link (QCar 2 optimizes/decides via an
  MPC+DCBF directly implementing the IDEAM paper's ellipse-constraint
  concept; ROSbot 3 only broadcasts predicted state and obeys bounded
  HOLD/PROCEED commands, never runs its own MPC). This answers several
  open questions from `TODO.md`'s original V2V design draft directly —
  see that file's update note at the top of the V2V section.
- Their README documents a directly relevant finding: QCar 2's default
  steering limit made it geometrically unable to follow a path recorded
  by the ROSbot (differential-drive, can pivot in place) — same class of
  bug as this session's own `kp`/turn-radius issue (#6 above), just on
  the other vehicle. Worth remembering as a general checklist item:
  verify a *driving* vehicle's actual turn-radius capability against
  whatever *recorded* the path, whenever the two aren't the same vehicle
  or the same control scheme.
- Open item: unclear whether GitHub handle `6hammad9` corresponds to
  Mr. Afreed or Mr. Sharjeel from Qais's guideline, or is a third
  collaborator — matters for the "≥2 members on paper reconstruction"
  requirement. Not yet confirmed by the user.

### QCar 2 side pulled locally + bench-tested, same day

Full teammate repo (not just the two ROSbot files) copied to
`~/qcar2_side/` (own git remote, `git pull`-able directly from
`6hammad9/qcar2-lane-keeping`). Then actually **ran** their documented
bench-test workflow on `rosbot-server` — QCar-side `v2v_receiver_node.py`
in one terminal, the real (not fake-tool) `rosbot_v2v_broadcaster.py` in
another, both talking over real UDP on localhost. Confirmed genuinely
working, not just import-checked: `rx: 624, parse_errors: 0` on the
receiver from real schema-2 packets; `/v2v/alive` correctly `false`
because no SLAM/AMCL was running (broadcaster had no TF, sent honest
"not localized" heartbeats) — the documented fail-safe contract observed
actually behaving as designed, not just read about. Separately also
verified their `v2v_fake_rosbot.py` tool replaying our real
`smoothed_trajectory.csv`: `/v2v/rosbot_speed` correctly reflected the
`--speed` passed to it. `gap`/`on_path` came back `NaN`/`false` in that
run — correct, not a bug, since the fake tool's path (ours) and the
receiver's reference path (the QCar's own) are two different physical
tracks with this ad hoc pairing. Full detail, exact commands, and
remaining open items (real field test with both vehicles on the *same*
map; the HOLD/PROCEED gate path is still unexercised) in `TODO.md`'s V2V
section. All test processes were stopped after validation.

### Live cross-machine V2V test, both real robots powered on, same day

Full success, real hardware — not the earlier bench/localhost test.
ROSbot 3 (`rosbot-server`, `192.168.0.110`) and QCar 2 (`192.168.0.53`)
both brought up (localization + broadcaster on ROSbot 3; hardware +
localization + `v2v_receiver` on QCar 2) and the link watched live from
QCar 2's own `/v2v/stats`/`/v2v/alive`: 1000+ packets, 0 parse errors, 0
seq drops, `/v2v/alive` correctly tracking ROSbot 3's real localization
state (false → true once AMCL converged). Full writeup, including two
unrelated infra problems hit along the way (QCar 2 terminals needing
per-shell `ROS_DOMAIN_ID`, and a `nav2_lifecycle_manager` /
`diagnostic_updater` ABI-mismatch crash on `rosbot-server` worked around
by manually activating `map_server`/`amcl` instead of relying on the
crashing lifecycle manager binary), in `TODO.md`'s V2V section.

**Real, unresolved finding surfaced by this test:** ROSbot 3 and QCar 2
localize against two *different*, independently-recorded SLAM maps of
the same physical track (`track_map.yaml` vs `track_map_new.yaml`,
different origins) — `v2v_params.yaml`'s `frame_tx/ty/tyaw` transform
between them is left at identity (their own README's stated assumption:
"both robots localize against the same map"), which isn't actually true
here. `on_path`/`gap` won't be meaningful on a real field test until
someone measures two shared physical landmarks in both maps and sets
that transform. Not blocking — today's goal (prove the link itself
works) is fully validated regardless — but a real prerequisite before
`gap`/`on_path`-dependent behavior (the QCar's speed governor, DCBF
positioning) can be trusted on an actual joint drive.

**Diagnostic pattern worth remembering:** most of this troubleshooting
thread was "topics not showing" repeatedly turning out to be per-terminal
`ROS_DOMAIN_ID`/sourcing not carrying over to new shells — same lesson
already learned on the ROSbot 3 side with `.venv` activation earlier this
session, now confirmed to generalize to a completely different machine
and ROS distro (QCar 2, Humble) too. When topics silently don't show, the
first check should now always be "does *this specific terminal* have the
same env as whatever's supposed to be publishing," not the app code.

### `rosbot_v2v_gate.py` HOLD/PROCEED — tested directly on real hardware

Completed the one remaining untested V2V piece same day. Fed a synthetic
tiny `TwistStamped` (angular-only, no forward motion — user approved
before publishing anything to the real robot) through the gate on
`rosbot-server`, drove it with raw UDP commands matching the real wire
schema. All 5 behaviors confirmed directly against the real
`/rosbot3/cmd_vel` topic: passthrough, HOLD zeroes output, TTL
auto-expiry (never a permanent latch), out-of-range TTL correctly
rejected (`30.0` > their own `MAX_COMMAND_TTL_SEC=5.0` cap — safety
validation working, not a bug), explicit PROCEED releases early. Full
detail in `TODO.md`'s V2V section. This closes out the V2V testing arc
for today — communication link, fail-safe contract, and the command/gate
direction are all now verified on real hardware, not just read about or
bench-tested. Remaining known gap is still the map-frame alignment
between the two robots' independent SLAM maps (separate open item).
