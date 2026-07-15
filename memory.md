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

# ROSbot 3 Laptop Discovery Debugging Log (Session 3)
**Date:** July 10, 2026
**Environment:** `sami@Inspiron-15` (Local laptop, WSL2 `Ubuntu-24.04`, mirrored networking), robot at `192.168.0.110`
**Continues from:** Session 2's parked "laptop-specific discovery quirk," now actively re-investigated because the user is back on the laptop (previously the workaround was to just use `rosbot-server` instead).

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
