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

# ROSbot 3 Laptop Discovery Debugging Log (Session 3 — Laptop Track)
**Date:** July 10, 2026
**Environment:** `sami@Inspiron-15` (Local laptop, WSL2 `Ubuntu-24.04`, mirrored networking), robot at `192.168.0.110`
**Continues from:** Session 2's parked "laptop-specific discovery quirk," now actively re-investigated because the user is back on the laptop (previously the workaround was to just use `rosbot-server` instead).
**Note:** this ran as a separate, parallel Claude Code session on the laptop while the `rosbot-server` work below (also independently labeled "Session 3" at the time) was ongoing — merged into one file after the fact, hence the disambiguating "Laptop Track" suffix rather than a renumber.

### Timeline of Operations

* **Env var re-verification**
    * Non-interactive `bash -lc` showed `ROS_STATIC_PEERS`/`ROS_AUTOMATIC_DISCOVERY_RANGE` as unset — false alarm, caused by Ubuntu's default `~/.bashrc` early-return guard (`case $- in *i*) ;; *) return;; esac`) skipping the rest of the file for non-interactive shells. A true interactive shell (`bash -ic`) confirmed all three vars correctly live: `ROS_STATIC_PEERS=192.168.0.110`, `ROS_AUTOMATIC_DISCOVERY_RANGE=OFF`, `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`.

* **Baseline connectivity — all healthy**
    * `ping 192.168.0.110` succeeds (~4-130ms, jittery but connected).
    * `curl http://192.168.0.110:8081/stream_viewer?topic=/rosbot3/oak/rgb/image_raw` → HTTP 200 in 11ms — robot's web/camera stack is live.
    * `ros2 topic list --no-daemon --spin-time 3.0` (and later `12.0`) → still only `/parameter_events`, `/rosout`. Discovery genuinely broken, not a "nothing running" false negative.

* **Windows Firewall — not the blocker**
    * `Get-NetFirewallRule` found a pre-existing inbound rule `"ROS2 WSL2 Discovery"`: `Allow`, `UDP 7400-7500`, `Profile=Any`. Already permissive; not from this session (predates `memory.md`, no log of who/when created it).
    * Confirmed via `Get-NetConnectionProfile`: Wi-Fi (`ROSBOTWIFI5`) is categorized `Public`, and WSL2 is in `networkingMode=mirrored` (shares the host's real Wi-Fi IP `192.168.0.74` directly, subject to Windows Firewall) — flagged as a plausible risk factor, but the existing `Profile=Any` rule already covers it.

* **Found `~/fastdds_wsl.xml` — a prior, unfinished fix attempt**
    * Pre-existing custom Fast-DDS XML profile (not referenced anywhere in `memory.md` from Session 1/2 — predates this log). Whitelists Fast-DDS to bind only `127.0.0.1` + `192.168.0.74` (excluding the laptop's `docker0` bridge at `172.18.0.1` and `loopback0`, both also UP), and defines an `initialPeersList`.
    * Two problems found: (1) peer IP inside was stale — `192.168.0.50` instead of the robot's actual `192.168.0.110`; (2) never wired up — no `FASTRTPS_DEFAULT_PROFILES_FILE` env var pointed at it anywhere.
    * **Fix applied:** corrected the IP to `192.168.0.110` in the XML, and added `export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_wsl.xml"` to `~/.bashrc` (placed after the `ROS_STATIC_PEERS` line). Confirmed Fast-DDS version (`rmw_fastrtps_cpp` 8.4.3 / `fastrtps` 2.14.6) uses this pre-3.x env var name correctly, and the XML parses cleanly.
    * **Result: no change.** `ros2 topic list --no-daemon` still only returns the 2 default topics, with or without this profile loaded (confirmed by re-running with `env -u FASTRTPS_DEFAULT_PROFILES_FILE` to bypass it entirely — identical outcome). The XML/interface-whitelist theory is **ruled out**.

* **Definitive wire-level proof via `tcpdump`** (user ran manually, capturing `udp portrange 7400-7500 and host 192.168.0.110` on `eth1` while running `ros2 topic list --no-daemon --spin-time 6.0`)
    * Laptop sends unicast SPDP announces (480 bytes) from `192.168.0.74` to `192.168.0.110` on ports `7410/7412/7414/7416` (guessing participant-ID slots 0-3, as expected for `ROS_STATIC_PEERS` on domain 0).
    * **The robot replies.** Four distinct ephemeral source ports (`58675`, `60729`, `58028`, `56465` — four separate DDS participants on the robot) send data back to `192.168.0.74:7410`, ~9ms round-trip, continuing for minutes across multiple CLI invocations (normal reliable-writer resend behavior).
    * **This conclusively rules out network/firewall/NAT as the cause** — bidirectional UDP flow is real and fast.
    * `/proc/net/snmp` UDP counters corroborated this earlier (494 new inbound datagrams during one ~8s attempt, only 3 hitting closed ports) before the full packet capture was available.

* **Hex-dump analysis (`tcpdump -X`) — traffic is genuine, valid RTPS, but SPDP-only**
    * Confirmed proper `RTPS` magic bytes, vendor ID `01 0f` (eProsima Fast-DDS), and fully decodable SPDP `Data(p)` participant announcements containing real metadata:
      - `fastdds.physical_data.host` = `rosbot3:8625654422478520320`
      - `fastdds.physical_data.user` = **`husarion`** ← robot's SSH username, previously unknown (Session 2's SSH attempt failed on a literal `<robot-user>` placeholder)
      - `fastdds.physical_data.process` = `22845` / `22846` (two live ROS2 node processes replying)
      - `enclave=/;`, `PARTICIPANT_TYPE=SIMPLE`
    * **Key narrowing finding:** every observed packet (both the 508-byte `DATA` and the 112-128-byte `HEARTBEAT` messages) belongs only to the **SPDP builtin participant writer** (entity ID `000100c2`). **No SEDP traffic** (topic/endpoint discovery, entity IDs `0002/0003 00c2`) was ever observed in either direction. So: participant-level discovery (SPDP) fully succeeds bidirectionally; topic-level discovery (SEDP) never even starts.

* **Ruled out: stale/competing daemon squatting on the discovery ports.** `ps aux | grep -iE "ros2|fastdds|_ros2_daemon"` and `sudo ss -lunp | grep 74xx` both came back empty — no leftover process contending for port 7410.

* **Ruled out: needs more time.** Wrote `~/poll_graph.py` (an `rclpy` node polling `get_topic_names_and_types()` every 0.5s for 25s instead of relying on the CLI's short-lived `--spin-time` window). Ran twice — once with the XML profile active, once without — both times still only ever saw `/parameter_events`, `/rosout` for the full 25s.

* **Working theory (unconfirmed, next step):** `ROS_AUTOMATIC_DISCOVERY_RANGE=OFF` may behave asymmetrically — if the robot's own side is *also* static-peers-only and its `ROS_STATIC_PEERS` only lists `rosbot-server`'s IP (not the laptop's `192.168.0.74`), it may dutifully answer inbound SPDP from an unlisted peer but never push SEDP to it. This is the exact question Session 2 flagged and never answered ("Check whether `rosbot-server`'s own `~/.bashrc` needs its own `ROS_STATIC_PEERS` entry pointing at the robot").

* **Blocked: SSH into the robot to check.** Attempted `ssh husarion@192.168.0.110` (username recovered from the packet capture above) with `BatchMode=yes` to test key-based auth — no key configured (`Permission denied (publickey,password)`). Needs the robot's actual SSH password, which the user has not yet provided. **This is the open blocker as of end of session.**

### Open Items / Next Steps

- **Get the robot's SSH password** for `husarion@192.168.0.110`, then check its `ROS_STATIC_PEERS`/`ROS_AUTOMATIC_DISCOVERY_RANGE` env and compare against the laptop's — test the "robot needs the laptop's IP in its own static peers list" theory directly.
- If that's confirmed, the fix is robot-side: add `192.168.0.74` (or `ROS_AUTOMATIC_DISCOVERY_RANGE` loosened) to the robot's own discovery config — a change on the robot, not the laptop.
- If robot-side peers aren't the cause, next diagnostic depth would be full FastDDS internal debug logging (not yet attempted — no known env-var toggle found for this Fast-DDS 2.14.6 build; would need an XML `<log>` block or rebuilding with debug symbols) or a Wireshark RTPS-dissector pass on `~/dds_capture.pcap` for a fuller decode than manual hex-reading.
- Cosmetic, still outstanding from Session 2: dedupe the two `FASTDDS_BUILTIN_TRANSPORTS` exports in `~/.bashrc`.

### Key Files Touched

- `~/fastdds_wsl.xml` (laptop) — corrected stale peer IP `192.168.0.50` → `192.168.0.110`.
- `~/.bashrc` (laptop) — added `export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_wsl.xml"`.
- `~/sniff_dds.py` (laptop) — created; shared-port UDP sniffer, result inconclusive (superseded by `tcpdump`).
- `~/poll_graph.py` (laptop) — created; long-lived `rclpy` graph-polling script, used to rule out the "just needs more time" and "XML profile" theories.
- `~/dds_capture.pcap` (laptop) — `tcpdump` capture of the working discovery attempt; primary evidence for this session's conclusions.

---

# ROSbot 3 Firmware 2.0 Migration & Path-Recording Debugging Log (Session 3 — rosbot-server Track)
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

---

# Shared-map migration: QCar 2 map copied to ROSbot 3 (2026-08-06)

**Goal.** Put both vehicles on one map so `frame_tx/ty/tyaw` in QCar 2's
`v2v_params.yaml` can go back to identity, instead of the measured
landmark transform. `V2V_README.md` calls the single-map case the intended
deployment.

**This entry is written so the change can be undone. Nothing was
overwritten — the step below is purely additive.**

## What was done

Copied from QCar 2 `nvidia@192.168.0.53:~/qcar_v2v_ws/mapping_output/`
into `rosbot3/config/`:

| file | bytes | md5 |
|---|---|---|
| `qcar_real_20260802-014755.pgm` | 38039 | `c7e772c5c700d4c43ea38539d987a7ff` |
| `qcar_real_20260802-014755.yaml` | 143 | `733265e28a5ba54a74905d9449944985` |

md5 verified identical on the car and after landing in `config/`.

Map parameters:

```
image: qcar_real_20260802-014755.pgm
resolution: 0.05
origin: [-5.47, -5.79, 0]
occupied_thresh: 0.65
free_thresh: 0.25
```

196 x 194 cells @ 0.05 m = **9.80 x 9.70 m**, covering
x -5.47..4.33, y -5.79..3.91.

> Note: rviz earlier reported 247 x 227 at origin (-5.4653, -7.4435).
> That is the **live** `/map` topic, which
> `cartographer_occupancy_grid_node` regenerates from the pose graph; it
> does not match the saved file. Use the saved `.yaml` values above.

## TO REVERT

Delete the two new files. Nothing else changed:

```bash
rm rosbot3/config/qcar_real_20260802-014755.pgm \
   rosbot3/config/qcar_real_20260802-014755.yaml
```

`config/track_map.{yaml,pgm,data,posegraph}` were **not touched** —
verified byte-identical before and after (track_map.pgm 35565 B,
track_map.data 32089695 B, track_map.posegraph 44019871 B). ROSbot 3
still localizes against its own map until someone changes the `map:=`
argument, so this copy is inert on its own.

## NOT yet done — required before this map is actually used

1. **Point AMCL at it.** `README.md` §2 Terminal 2 currently launches with
   `map:=config/track_map.yaml`. Switching to
   `map:=config/qcar_real_20260802-014755.yaml` is what activates the
   shared map. Untested.
2. **Verify AMCL converges in it.** Real risk, not a formality: QCar 2's
   scan plane is at **0.161 m** (URDF `body_lidar_joint`) and ROSbot 3's
   LiDAR sits at a different height, so the two see different wall
   features. The QCar's map may not contain what ROSbot 3's LiDAR
   returns. Check `/amcl_pose` covariance converges below ~0.02 the way it
   did during the landmark collection, and do not trust a tight covariance
   alone — AMCL can converge confidently to a wrong place.
3. **`config/smoothed_trajectory.csv` becomes invalid.** It holds 540
   points, 27.02 m, mean spacing 0.0501 m, in **ROSbot-map** coordinates.
   In the QCar frame those waypoints point somewhere else entirely.
   **Do not run `trajectory_follower_node.py` after switching maps** until
   the route is redone.
4. **Route plan (decided, not yet executed).** Re-record the trajectory on
   **QCar 2** — it is Ackermann with a minimum turning radius, ROSbot 3 is
   differential drive and can follow anything the QCar can; the reverse is
   not true. Then convert format rather than re-smoothing: QCar
   `my_route_loop.npy` is `(N,3) = x, y, yaw`, already spline-smoothed and
   resampled at exactly 0.05 m by `smooth_path.py`; ROSbot wants CSV with
   header `x,y,theta`. Both already use 0.05 m spacing, so it is a direct
   column mapping. Passing it through `config/smooth.py` again would be
   double-smoothing and only rounds corners further.
5. **Start-point conflict on a shared route.** Not a shared-pose problem —
   each robot localizes independently in the shared frame. The clash is
   that `path_mpc` accepts the **nearest** waypoint (logs showed
   `Start alignment accepted at idx=768`, then `idx=655`), while ROSbot's
   `_handle_rotate_to_start` drives to `self._trajectory.start`, i.e.
   **waypoint 0**, every time. On one shared route both would converge on
   waypoint 0. Cheapest fix: stage the QCar ~2 m behind waypoint 0 and
   change nothing. Durable fix: make `Trajectory.start` honour a
   configurable `start_waypoint_index` in `core/trajectory.py`.
6. **Check segment detection on the converted route.** ROSbot's follower
   splits a trajectory into segments at direction flips
   (`Trajectory._detect_segments`). The QCar loop was recorded by a vehicle
   that never reverses, so it should yield one forward segment — confirm
   that, or ROSbot will try to reverse mid-loop.
7. **The QCar loop is 38.80 m (777 pts) vs ROSbot's current 27.02 m**, and
   it contains the loop seam — a synthesised 1.07 m bridge with a wall at
   0.55 m, curvature previewing at 3.728. ROSbot would inherit both.

## Why this is worth doing

Removes the landmark transform entirely
(`frame_tx=-4.679956, frame_ty=-3.745196, frame_tyaw=-0.705757`, 3.7 cm
residual) and with it a whole class of error. Supporting evidence that the
two are the same physical track: `qcar2_side/sim/assets/track_map.pgm` is
**byte-identical** to `rosbot3/config/track_map.pgm` (md5
`c35079fc264688e29057ce3c62e70e34`) — the QCar simulation already runs on
the ROSbot's map.

## Shared-map switch executed on ROSbot 3 (2026-08-06, same session)

**The `map:=` launch argument does NOT switch the map on this robot.**
`config/amcl_params.yaml` hard-codes `map_server.ros__parameters.yaml_filename`
and silently wins over the command line. The first attempt launched with
`map:=config/qcar_real_20260802-014755.yaml` and *appeared* to succeed, but
`/map` came back **237 x 150 at origin (-7.712, -0.931)** — ROSbot's own old
map. Anyone following README §2 alone would believe they had switched maps
while still running the old one.

**Real switch point:** `config/amcl_params.yaml`, the `yaml_filename` line.
Now set to `config/qcar_real_20260802-014755.yaml`, with the previous value
commented directly above it. Original backed up to
`/tmp/rosbot_logs/amcl_params.yaml.bak` (session-temp -- copy somewhere
durable if it matters). README §2 updated with the same comment and a note
that the params file, not `map:=`, is what takes effect.

**Confirmed loaded after the fix:** `/map` reports 196 x 194 @ 0.05,
origin (-5.47, -5.79) -- matches the QCar file exactly.

**Stack brought up** (trajectory follower deliberately NOT started, so the
robot cannot drive itself): `tf_relay`, `map_server`, `amcl`, all activated
manually via `ros2 lifecycle set` because `nav2_lifecycle_manager` still
dies on the `diagnostic_updater` undefined-symbol ABI mismatch (shared
machine, no sudo) -- unchanged from earlier sessions.
`/rosbot3/scan` confirmed flowing, AMCL correctly configured to it
(`scan_topic: "/rosbot3/scan"`).

**Convergence: NOT yet achieved.** After
`reinitialize_global_localization`, `/amcl_pose` covariance read
**x=8.4728, y=7.1074, yaw=6.2727** against a <0.02 target -- a fully
dispersed particle cloud. This is expected while stationary: global
localization scatters particles across the whole map and needs **motion**
to collapse them. It is NOT yet evidence for or against the shared map.

**Blocked on motion, and the robot is boxed in.** Its own LiDAR reports
**0.132 m to the nearest return forward** (+/-20 deg); backward is clear at
1.224 m; 171 returns inside 1.0 m. Forward motion would collide almost
immediately, so no drive command was issued.

**The open question the next session must answer:** drive ROSbot a few
metres and re-read the covariance.
  - converges < 0.02  -> shared map works; set QCar's
    `frame_tx/ty/tyaw` to identity and retire the landmark transform
    (-4.679956, -3.745196, -0.705757).
  - stays dispersed    -> the QCar's map does not contain what ROSbot's
    LiDAR sees. QCar scan plane is 0.161 m (URDF `body_lidar_joint`);
    ROSbot's LiDAR sits elsewhere, so they may not share wall features.
    Shared-map approach then needs rethinking, not forcing.

**Do not trust a tight covariance on its own** -- check the pose is
physically where the robot actually is. QCar 2 spent this morning
confidently 3.9 m wrong with convincing covariance.

**To revert everything:** restore the `yaml_filename` line in
`config/amcl_params.yaml` (previous value is in the comment above it), and
optionally delete the two copied map files. Nothing else was changed.

## RESULT: shared map WORKS on ROSbot 3 -- and the card collar was blinding it

**Verdict: ROSbot 3 localizes successfully in QCar 2's Cartographer map.**
The height-mismatch worry (QCar scan plane 0.161 m vs ROSbot's LiDAR) does
**not** block it. The shared-map plan is viable.

### The real blocker was the card collar, not the map

The collar taped to ROSbot 3 so QCar 2's LiDAR can see it was sitting **in
ROSbot 3's own LiDAR plane**. Measured across 60 consecutive scan frames: a
persistent continuous arc of returns from **-55 deg to +40 deg at
0.08-0.17 m** -- about 95 degrees of the forward view. Not a wall (a flat
wall at 0.085 m would read 0.111 m at 40 deg; it read 0.174 m), and not a
clean circle: an irregular surface wrapped close around the front.

Effect: AMCL was fed a phantom obstacle at ~0.1 m across its whole forward
arc, matching nothing in any map. `amcl_params.yaml` sets
`laser_min_range: 0.1` and these returns straddle it (0.079-0.17 m), so
many passed the filter and poisoned the likelihood field. **AMCL could not
have converged in ANY map in that state** -- so the earlier dispersed
covariance was never evidence against the shared map.

With the collar removed the arc vanished; only a 13-beam band at -20 deg /
0.131 m remains, which is fixed structure and rotates with the robot.

### Convergence measurement (rotation in place, 0.40 rad/s, ~4.8 revs)

| t | cov x | cov y | cov yaw | pose |
|---|---|---|---|---|
| 5 s | 6.1812 | 4.6340 | 2.8255 | (+1.21, +0.58) |
| 20 s | 0.4817 | 5.0014 | 5.4038 | (-1.34, -4.20) |
| 35 s | 0.0723 | 0.3497 | 0.1182 | (-0.12, -0.08) |
| 50 s | 0.0469 | 0.0272 | 0.0285 | (-0.15, -0.12) |
| 65 s | 0.0322 | 0.0194 | 0.0242 | (-0.21, -0.12) |
| final | 0.0336 | 0.0217 | 0.0243 | (-0.261, -0.084) |

~200x collapse in x and y; pose stops wandering after t=35 s and settles
near (-0.2, -0.1). Rotation alone gets y to 0.019; x stays ~0.03 because
pure rotation carries little position information. A short translation
should close that.

### CONFLICT this creates for the V2V work -- unresolved

`PROMPT.md` records that **without** the collar, QCar 2's LiDAR returns
*literally zero* points off the ROSbot -- the collar is what makes ROSbot
detectable for obstacle avoidance and overtaking. So:

* collar ON  -> QCar sees ROSbot; ROSbot cannot localize
* collar OFF -> ROSbot localizes; QCar cannot see ROSbot

Both are needed simultaneously. Likely fix is **height**: mount the collar
so it sits above ROSbot 3's own scan plane while still intersecting QCar
2's at 0.161 m. Requires measuring ROSbot 3's LiDAR height -- not yet done.
Until resolved, any overtake test needs the collar and therefore cannot
rely on ROSbot 3's AMCL at the same time.

### Still to verify

Covariance is not proof of correctness. **Confirm the converged pose is
physically where the robot actually is** before trusting it -- QCar 2 spent
this morning confidently 3.9 m wrong with tight covariance.

### Translation check -- covariance DEGRADED (unresolved)

Rotation converged; a short forward drive did not. Drove 0.12 m/s, moved
**0.76 m** in the map:

| | cov x | cov y | cov yaw | pose |
|---|---|---|---|---|
| start | 0.0343 | 0.0252 | 0.0229 | (-0.22, +0.05) |
| +5 s | 0.0346 | 0.0290 | 0.0245 | (-0.17, +0.18) |
| final | **0.1635** | **0.1248** | **0.2189** | (+0.213, +0.679) |

Covariance grew ~5x and did not recover. AMCL uncertainty normally grows
with motion then shrinks as scan matches confirm the pose; it growing and
staying grown suggests scan matching is not confirming well in the QCar map
once ROSbot leaves the spot it converged on. **So the shared map is proven
for rotation-in-place but NOT yet for driving.** Do not treat the earlier
convergence as sufficient.

Run aborted safely by the 0.45 m forward guard. Note forward clearance fell
**4.82 m -> 0.448 m while the robot travelled only ~0.84 m** -- something
entered its path (person, or the QCar). Not identified.

**Next:** confirm the reported pose (+0.213, +0.679) is physically where
the robot actually is. If the pose is wrong, the degradation is a bad match,
not sensor noise. A ROSbot-side version of `qcar2_side/utils/map_web_view.py`
(retargeted at `/rosbot3/scan`, `/map`, and the ROSbot TF tree) would settle
it visually -- the check that matters is whether scan returns land on the
mapped walls.

### CONFIRMED: ROSbot 3 localizes correctly in QCar 2's map (drive test)

Drove ROSbot ~7 m under a 0.45 m obstacle guard (0.12 m/s, turning away when
blocked), scoring **scan-vs-map match** = fraction of scan endpoints landing
within one cell of an occupied map cell. Covariance says how *confident*
AMCL is; this says whether it is *right*.

| t | pose | cov x | match |
|---|---|---|---|
| 6 s | (-0.09, +0.65) | 0.1672 | **12.4%** |
| 18 s | (-1.80, -0.16) | 0.0223 | **77.4%** |
| 30 s | (-3.32, -0.18) | 0.0178 | **91.1%** |
| 42 s | (-4.32, -0.19) | 0.0204 | 72.7% |
| 54 s | (-4.26, -0.25) | 0.1101 | 19.0% |
| 66 s | (-2.84, -1.51) | 0.0692 | 4.0% |
| 78 s | (-1.51, -2.90) | 0.0148 | 59.7% |
| final | (+0.064, -3.422) | 0.0322 | **66.6%** |

**Verdict: the shared map is viable.** 91% match is a genuine lock, so the
0.131 m (ROSbot) vs 0.161 m (QCar) LiDAR height difference does not prevent
localization. `frame_tx/ty/tyaw` can go to identity once the route is
redone.

**Confirms the earlier failure was AMCL locked on a wrong pose, not a bad
map.** At t=6 s the pose was still (-0.09,+0.65) -- essentially where it had
been sitting -- with only 12.4% match. It needed *translation* to escape;
rotation in place had produced tight covariance (0.03) at a wrong pose.
**Covariance alone would have lied.** Same failure mode as QCar 2 being
confidently 3.9 m wrong.

**Map coverage is uneven -- plan around it.** Match swung 91% -> 4% -> 63%
depending on location. Cartographer built this map along a driving route, so
coverage is good near the path and thin away from it. Expect reliable
localization on-route and degradation off-route. Not a blocker; it does
constrain where things can be staged.

**Method note for future sessions:** the match score is the check worth
trusting, not covariance. Implementation in
`/tmp/rosbot_logs/drive_converge.py` (session-temp -- copy into the repo if
it is wanted long term). Viewer equivalent on :8092, cv2-based because
matplotlib on rosbot-server is broken by a numpy 1.x/2.x ABI split
(`numpy.core.multiarray failed to import`); system python has working
numpy 2.3.0 + cv2 4.13.0 + rclpy.

**Also measured:** ROSbot 3 `base_link -> laser` is xyz (+0.020, 0, +0.131)
with **yaw = 180 deg** -- the same mounting flip as the QCar. Any tool
plotting scan angles against the body pose must add pi.

## Shared route converted and verified on the shared map (2026-08-06)

**Decision:** both vehicles drive the SAME 38.8 m loop. ROSbot keeps its
existing behaviour of starting at waypoint 0; the QCar is staged elsewhere
on the loop, which sidesteps the start-point clash entirely with no code
change (`path_mpc` accepts the nearest waypoint, ROSbot always drives to
waypoint 0).

### Files now in `rosbot3/config/`

| file | provenance |
|---|---|
| `qcar_real_20260802-014755.pgm` / `.yaml` | QCar Cartographer grid, md5 `c7e772c5...` verified |
| `shared_route.csv` | 777 pts, converted from the QCar's `my_route_loop.npy` |
| `qcar_route_to_csv.py` | the converter, with ROSbot-segment validation |

`my_route_loop.npy` fetched from
`nvidia@192.168.0.53:~/qcar_v2v_ws/mapping_output/`, md5
`f26406d27f38be47b13479de2a9736c2`, verified after transfer.

### Conversion result -- clean

```
points        : 777
length        : 38.80 m
spacing       : mean 0.0500 m (min 0.0500, max 0.0500)
loop closure  : 0.018 m between last and first
bbox          : x -3.44..+2.09   y -4.00..+0.10
direction flips (dot < -0.5) : 0   (min dot +0.995)
initial segment reversed     : False  (travel vs heading 1.0 deg)
```

Zero flips with min dot +0.995 -- nowhere near the -0.5 threshold -- so
`Trajectory._detect_segments` sees ONE continuous forward segment and the
ROSbot will not try to reverse mid-loop. Spacing min == max, so no gaps.

**Deliberately NOT re-smoothed.** The QCar's `smooth_path.py` already spline
-fitted and resampled at 0.05 m; running it through `config/smooth.py` again
would round the corners a second time. The converter only changes container
(`.npy` (N,3) x,y,yaw -> CSV x,y,theta) and wraps yaw to +/-pi.

### Visual confirmation -- resolves the "track outside the walls" question

Rendered the converted route on the shared map: **the loop sits neatly
inside the walled area**, ~0.9 m clear of the left wall and ~0.5 m of the
bottom, never crossing a wall or leaving mapped free space. Waypoint 0 is
at the top of the loop just below the north wall.

The earlier "track is outside the walls" was the frame-mismatched overlay --
ROSbot's own `smoothed_trajectory.csv` (ROSbot map frame) painted on the
QCar map. With the correct route in the correct frame everything agrees, and
**the map is the one the QCar's test runs validated**. My earlier
"the .pgm is a stale export" theory was wrong; the 247x227 vs 196x194
difference is just Cartographer's live grid regenerating from the pose graph
versus the saved export.

### MUST DO before this route drives anything

1. **Point the V2V broadcaster at it too** --
   `-p trajectory_csv:=config/shared_route.csv`. The follower reads it, but
   so does `rosbot_v2v_broadcaster.py`, which rolls the robot forward along
   that route to predict its next 2 s and transmits those 26 poses. That
   prediction feeds the QCar's DCBF keep-out and its overtake/yield calls.
   Left on the old CSV, the QCar plans around a path in the wrong frame.
2. **Set `frame_tx/ty/tyaw` to 0** in the QCar's `v2v_params.yaml` once the
   shared map is confirmed in use -- currently
   (-4.679956, -3.745196, -0.705757). NOT yet done.
3. **The loop seam is in this route** -- the synthesised 1.07 m bridge with
   a wall at 0.55 m, previewing at kappa 3.728. ROSbot now inherits it. Do
   not stage a pass there.

### To revert

Delete `config/shared_route.csv`; restore `yaml_filename` in
`config/amcl_params.yaml` (previous value is in the comment above it).
`config/smoothed_trajectory.csv` was never modified.

---

## Session 2026-08-06 (evening): first ROSbot run on the shared route — 3 bugs found

`trajectory_follower_node.py` was pointed at `config/shared_route.csv` and
run. **It drove off the route and had to be stopped by hand.** Three
separate defects, all of which only appear on the QCar's route. All three
are now fixed; the run has NOT been repeated (battery, see below).

### What the log showed

```
Aligned! Starting Segment(FWD: WP 0 → 776)
[FWD] Seg 1/1 | WP 33 → LA:40  | Goal: 1.62m
[FWD] Seg 1/1 | WP 33 → LA:503 | Goal: 1.50m     <- lookahead jumped 460 wp
[FWD] Seg 1/1 | WP 33 → LA:618 | Goal: 4.16m     <- and again, now 4 m off route
Overtake: detour generated, 21 pts, rejoin at WP 275
Obstacle at 0.39m -> 0.10m -> 0.05m -> 0.02m     <- driving into something
```

`WP 33` never advanced; `LA` jumped between 40, 284, 503, 618, 706.

### Bug 1 — the "38.8 m loop" is 3 laps of a 13.15 m circuit

Not a long loop. `my_route_loop.npy` returns to within 0.04 m of waypoint 0
at **wp 263**, and 0.018 m at wp 776. 777 / 263 = 2.95 laps.

Consequence: **97% of the 777 waypoints sit within 0.60 m of a
non-adjacent waypoint**, minimum self-approach **0.002 m**. wp 33 is 0.16 m
from wp 493. `Trajectory.find_closest_waypoint` did a global argmin over the
whole segment, so it was choosing between laps essentially at random.

Offline replay of the recorded route with 6 cm of simulated AMCL jitter:

| route | backward index jumps > 5 wp | max index error |
|---|---|---|
| 777 pts (3 laps), old global search | **147** | **517 wp (25 m)** |
| 263 pts (1 lap), old global search | 0 | 5 wp |
| 263 pts (1 lap), new windowed search | 0 | 5 wp |

**Fix:** `config/qcar_route_to_csv.py` gained `--single-lap` (truncates at
the first return to the start) plus a self-approach measurement that makes
it **refuse to write** when > 5% of waypoints are ambiguous.
`config/shared_route.csv` is now **263 pts / 13.10 m**, 0% ambiguous,
minimum self-approach 1.468 m. The 3-lap version is at
`/tmp/rosbot_logs/shared_route_3lap.csv.bak`.

### Bug 2 — `find_closest_waypoint` searched globally

Even one lap closes on itself: wp 0 and wp 262 are 0.081 m apart. Measured
with the old global search, a robot sitting *behind* waypoint 0 — exactly
where it starts — resolved to the **end** of the route:

| robot position | old search | new search |
|---|---|---|
| 0.30 m behind wp0 | wp 257 | wp 0 |
| 0.15 m behind wp0 | wp 260 | wp 0 |
| 0.05 m behind wp0 | wp 262 | wp 0 |

**Fix:** `rosbot_lane/core/trajectory.py` — the search is now windowed to
`[_search_anchor - 20, _search_anchor + 100]` (−1.0 m / +5.0 m), and the
anchor advances on its own. Deliberately *not* keyed off `current_wp_idx`:
`advance_waypoint` only advances while the next waypoint is within 0.12 m,
so it **freezes permanently** the moment the robot drifts off route — that
is why the log showed `WP 33` forever. The anchor cannot freeze that way.

`_search_anchor` is reset in `reset()` and `advance_segment()`.

### Bug 3 — segment goal fires before the robot moves

`_segment_goal_tolerance = 0.04` (`trajectory_follower_node.py:82`) against
an 0.081 m start-finish gap, with ~6 cm of AMCL jitter. The follower would
have declared the lap complete while standing still.

**Fix:** `reached_segment_goal` now also requires
`_search_anchor >= seg.end_idx - SEARCH_BACK`, i.e. the robot must actually
have travelled to the far end. Verified: `False` at every distance behind
wp 0 before driving, `True` after walking the lap.

### Also fixed: two stacks were running at once

`ps` showed **two** `tf_relay` and **two** `localization_launch`
(2 × `map_server` + 2 × `amcl`). Same duplicate-process class that caused
the earlier TF flapping. The *older* AMCL (pid 27955, started 20:13:47) was
the live one — identified by CPU time (11076 ticks and climbing vs 119
frozen), not by age. It loaded the shared map correctly: `amcl_params.yaml`
was edited at 20:12:48, **59 s before** it started, and live `/map` is
196×194 @ origin (−5.470, −5.790). The idle duplicate set was killed by
explicit PID.

> Lesson repeated for the third time: identify the live process by **CPU
> time**, and kill by **explicit PID**, never `pkill -f`.

### Why the run was not repeated

- **Battery flat.** `/rosbot3/battery` (not `battery_state`) read 9.84 V and
  falling at −1.7 V/hour, `percentage` 0, `current` NaN — not charging. The
  3S pack cuts out around 9.0 V and sags further under motor load.
- **Localization degraded.** Scan-map match had dropped to **51.9%**
  (1342/2588 endpoints) from ~71% earlier in the session. Below the ~70%
  bar we set for trusting a pose. Pose was `(−0.404, −0.038) yaw +123.9°`,
  0.014 m from wp 2.

Both need clearing before the next attempt. Charge first, then re-check the
match score before commanding motion.

### Unrelated issue seen, not fixed

`_closest_in_region` rejects returns below `msg.range_min`, but the ROSbot's
LiDAR reports `range_min = 0.050` while its real blind zone is much larger.
Returns in 0.05–0.15 m are noise and are currently trusted. During the run
the obstacle distance flapped inf ↔ 0.05 m ↔ 0.02 m, and because the
hysteresis is stop-at-0.40 / resume-at-0.50, each "clear" let the robot
resume at full 0.40 m/s. Worth raising the floor to ~0.15 m.

## Collar conflict RESOLVED (2026-08-06, evening)

A second collar was fitted that **clears the ROSbot's 0.131 m scan plane
while still intersecting the QCar's 0.161 m** -- roughly a 3 cm window.
Both properties now hold at once:

- ROSbot's AMCL is no longer blinded (the old collar put a persistent 95 deg
  arc at 0.08-0.17 m across its scan and stopped it converging in *any* map)
- QCar's LiDAR gets returns off the ROSbot again, so `obstacle_ahead` is set
  by the physical sensor

**This closes the open item "collar conflict" and changes the overtake
picture.** The QCar's `DRIVE -> WAIT_FOR_CLEAR -> OVERTAKE_LEFT` path runs
with no V2V trigger at all; `should_inject_slow_v2v_lead` gates only the V2V
*early-warning injection*, not the primary path. The earlier conclusion
"QCar will follow at 0.70 m and never pass" was conditioned on there being
no collar and no longer applies.

**Detection was never the cause of the recorded deadlock.** `PROMPT.md`'s
"detects the ROSbot, stops in WAIT_FOR_CLEAR, never commands a lateral
offset" was observed *with* the original collar. That is `can_avoid`
failing:

- `allow_overtake` is published by the MPC curvature preview
  (`path_mpc_node.py:1896`), not a static parameter. The bend measured 1.396
  against a hard cap of 1.0 -- no legal value authorises a pass there.
  Location-dependent; stage passes on a straight.
- `left_clear` read the outside road edge in a bend -- the curve-corridor
  bug. That fix **is now present** in the repo (`front_narrow_min` in
  `lidar_overtake_node.py`, `lidar_sector_analyzer.py`, `overtake_types.py`,
  `overtake_safety.py`).

**Cheapest next experiment:** park the ROSbot *unpowered* on a straight as a
passive obstacle and drive the QCar at it. Needs no ROSbot battery (flat at
9.8 V) and no V2V, and isolates the curve-corridor fix from everything else.

Plan for the paper-grounded improvement (lane-probing state, IDEAM Sec. V) is
in `qcar2_side/IDEAM_LP_PLAN.md`, untracked so it does not disturb the
teammate until pushed.

## Map decision: keep the SHARED map for V2V (2026-08-06)

Asked whether the old ROSbot map + landmark transform would be better than
the shared QCar map. **Shared map wins.** Computed from the round-2 landmark
readings in `TODO.md`:

- The two maps disagree by **7.3 cm** about the distance between the same two
  physical landmarks (1.0941 m in the QCar map vs 1.0209 m in ROSbot's) -- a
  **6.9% scale error**. A rigid transform preserves distance, so this is
  irreducible; the recorded "3.7 cm residual" is just that disagreement split
  evenly between the two points, not a quality measure.
- Baseline is only **1.06 m** for a track spanning 5.5 x 4.1 m, giving a
  **1.98 deg** heading uncertainty that extrapolates to **9.1 cm at the
  nearest route point and 22.0 cm at the farthest** (1.58 m / 5.30 m from the
  landmark midpoint). That is 55-63% of `V2V_ELLIPSE_B` (0.40) and
  `lane_half_width` (0.35) before any localization error.

**Decisive argument is not calibration, it is `on_path`.** Already measured
2026-08-05: `lat_offset = -0.451 m` vs a 0.35 m threshold, logged then as
"genuine physical divergence between ROSbot 3's recorded route and QCar 2's
specific reference lane". The transform aligns *frames*, not *routes* -- with
separate maps the two vehicles drive different physical lines and `on_path`
drops wherever they diverge. `on_path` is a hard gate on both
`should_inject_slow_v2v_lead` and the speed governor, so every V2V behaviour
goes inert there. Shared map + shared route makes `lat_offset ~ 0` by
construction.

Counter-argument considered and rejected: transform error is a *bounded
systematic bias* while shared-map localization failure is an *unbounded
jump*. True, but the measured shared-map match is 91% on-route vs 4%
off-route -- the error is bounded by staying on the route, which is what the
follower fixes deliver. The 51.9% measured today was taken after the failed
run had already driven the robot off-route.

**If shared-map localization ever does prove unreliable, the fallback is NOT
the old map** -- it is to re-record the shared map with the ROSbot's LiDAR
and give that to the QCar. Sharing in the other direction keeps the
single-frame property while removing the 0.131 / 0.161 m sensor-height
mismatch.
