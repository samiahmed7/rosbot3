#!/usr/bin/env python3
"""V2V status dashboard -- one browser page with both robots' camera feeds
and live-decoded V2V parameters, so you don't have to read console logs.

Run the SAME file on both robots, each with its own --role. Each instance
serves its own local camera as MJPEG (like camera_web_view.py) and reads
whatever V2V debug topics exist locally for that role; the page also
embeds the peer's camera stream directly by URL (plain HTTP, so no
cross-machine ROS/DDS domain crossing is needed -- consistent with the
UDP-only V2V link this project deliberately uses).

    # On QCar 2:
    python3 v2v_dashboard.py --role qcar2 --peer-host 192.168.0.110

    # On ROSbot 3:
    python3 v2v_dashboard.py --role rosbot3 --peer-host 192.168.0.53

Then browse to http://<either-ip>:8090/ from a laptop -- either instance's
page shows both cameras and both sides' V2V state.

Self-contained on purpose, like rosbot_v2v_broadcaster.py: no imports from
rosbot_lane or qcar_science_night_pkg, so this single file can be copied
next to any checkout on either robot and run directly.

The page is served over plain HTTP with no authentication -- trusted lab
network only, same caveat as camera_web_view.py.
"""

import argparse
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import PoseStamped

try:
    import cv2
except ImportError:  # pragma: no cover - depends on the robot's image
    cv2 = None


ROLES = {
    "qcar2": {
        "label": "QCar 2",
        "camera_topic": "/camera/color_image",
    },
    "rosbot3": {
        "label": "ROSbot 3",
        "camera_topic": "/rosbot3/oak/rgb/image_raw",
    },
}

# The ROSbot runs web_video_server on its own board (192.168.0.110), which is
# a different machine from rosbot-server (192.168.0.100) where this script
# runs. Taking the feed straight from there costs nothing on rosbot-server and
# works even when this dashboard's own ROS graph is down.
#
# Must be /stream, not /stream_viewer: stream_viewer returns text/html (a page
# wrapping the feed) and will not render inside an <img>, while /stream is the
# multipart/x-mixed-replace MJPEG the tag needs.
#
# The quality/width/height parameters are not cosmetic. Unthrottled this feed
# is 640x400 at 20 Hz and measured **21 Mbps** on the wire; both dashboards
# embed it, so with a browser tab open that is three concurrent copies over
# WiFi. web_video_server wedged under exactly that load -- port 8081 still
# accepted TCP but never answered, while the camera topic itself stayed
# healthy at 20 Hz. Downscaling costs nothing worth having on a monitoring
# view and keeps the link inside its budget.
ROSBOT3_CAMERA_URL = (
    "http://192.168.0.110:8081/stream"
    "?topic=/rosbot3/oak/rgb/image_raw"
    "&type=mjpeg&quality=35&width=424&height=240"
)


def to_bgr(msg):
    """Decode common sensor_msgs/Image encodings into a BGR array.

    Same approach as camera_web_view.py: avoids cv_bridge on purpose, it's
    an extra dependency that's easy to have mismatched against the local
    OpenCV, and only a few encodings ever appear on these robots.
    """
    height, width = msg.height, msg.width
    enc = msg.encoding.lower()
    if height == 0 or width == 0 or len(msg.data) == 0:
        raise ValueError(
            f"empty frame ({width}x{height}, {len(msg.data)} bytes) -- the "
            "camera is publishing but producing nothing"
        )

    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(height, width, 3)
        bgr = img[:, :, ::-1] if enc == "rgb8" else img
    elif enc in ("rgba8", "bgra8"):
        img = buf.reshape(height, width, 4)[:, :, :3]
        bgr = img[:, :, ::-1] if enc == "rgba8" else img
    elif enc in ("mono8", "8uc1"):
        bgr = np.dstack([buf.reshape(height, width)] * 3)
    elif enc in ("16uc1", "mono16"):
        img = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)
        top = float(img.max()) or 1.0
        img8 = (img.astype(np.float32) * (255.0 / top)).astype(np.uint8)
        bgr = np.dstack([img8] * 3)
    else:
        raise ValueError(f"unhandled encoding {msg.encoding!r}")

    return np.ascontiguousarray(bgr)


class Dashboard(Node):

    def __init__(self, role, camera_topic, quality):
        super().__init__("v2v_dashboard")
        self.role = role
        self.quality = int(quality)
        self.frames = 0
        self.frame_errors = 0
        self._jpeg = None
        self._jpeg_lock = threading.Lock()

        self.data = {"role": role}
        self._data_lock = threading.Lock()

        self.create_subscription(
            Image, camera_topic, self._on_image, qos_profile_sensor_data
        )

        if role == "qcar2":
            self._wire_qcar2()
        else:
            self._wire_rosbot3()

        self.create_timer(5.0, self._report)
        self.create_timer(2.0, self._refresh_nodes)
        self._refresh_nodes()
        self.get_logger().info(
            f"v2v_dashboard up: role={role}, camera={camera_topic}"
        )

    def _refresh_nodes(self):
        # Same source ros2 node list uses -- shows what's actually up in
        # THIS robot's own ROS graph (the two robots' graphs are
        # deliberately separate, see rosbot_v2v_broadcaster.py's docstring,
        # so this can only ever report local nodes, same as the camera).
        try:
            names = sorted(
                f"{ns.rstrip('/')}/{name}" if ns not in ("", "/") else f"/{name}"
                for name, ns in self.get_node_names_and_namespaces()
            )
        except Exception as e:  # pragma: no cover - defensive, graph query
            self.get_logger().warn(f"node list refresh failed: {e}",
                                    throttle_duration_sec=5.0)
            return
        self._set("nodes", names)

    # -- camera -------------------------------------------------------
    def _on_image(self, msg):
        try:
            bgr = to_bgr(msg)
        except ValueError as e:
            self.frame_errors += 1
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        ok, buf = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        )
        if not ok:
            self.frame_errors += 1
            return
        with self._jpeg_lock:
            self._jpeg = buf.tobytes()
        self.frames += 1

    def latest_jpeg(self):
        with self._jpeg_lock:
            return self._jpeg

    def _report(self):
        self.get_logger().info(
            f"frames={self.frames} errors={self.frame_errors}"
        )

    # -- V2V topics -----------------------------------------------------
    def _set(self, key, value):
        with self._data_lock:
            self.data[key] = value
            self.data["_updated"] = time.time()

    def snapshot(self):
        with self._data_lock:
            return dict(self.data)

    def _wire_qcar2(self):
        # v2v_receiver_node's own bundled stats: rx, parse_errors,
        # seq_drops, age_s, gap, on_path, blocked, detour_intent,
        # hold_active, command_tx, command_errors.
        self.create_subscription(
            String, "/v2v/stats", self._on_stats_json, 10
        )
        self.create_subscription(
            Bool, "/v2v/alive",
            lambda m: self._set("alive", bool(m.data)), 10
        )
        self.create_subscription(
            Float32, "/v2v/rosbot_speed",
            lambda m: self._set("rosbot_speed", round(m.data, 3)), 10
        )
        self.create_subscription(
            String, "/v2v/encounter",
            lambda m: self._set("encounter", m.data), 10
        )
        self.create_subscription(
            PoseStamped, "/v2v/rosbot_pose", self._on_rosbot_pose, 10
        )
        # From path_mpc / lidar_overtake, if those nodes happen to be up --
        # optional extras, harmless if nothing is publishing them.
        self.create_subscription(
            Bool, "/allow_overtake",
            lambda m: self._set("allow_overtake", bool(m.data)), 10
        )
        self.create_subscription(
            Bool, "/motion_enable",
            lambda m: self._set("motion_enable", bool(m.data)), 10
        )

    def _on_stats_json(self, msg):
        try:
            stats = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        with self._data_lock:
            self.data.update(stats)
            self.data["_updated"] = time.time()

    def _on_rosbot_pose(self, msg):
        self._set("rosbot_x", round(msg.pose.position.x, 3))
        self._set("rosbot_y", round(msg.pose.position.y, 3))

    def _wire_rosbot3(self):
        # rosbot_v2v_broadcaster's own local debug stats: tx, tx_errors,
        # seq, speed, blocked, blocked_distance, detour_intent.
        self.create_subscription(
            String, "/v2v/tx_stats", self._on_stats_json, 10
        )
        # rosbot_v2v_gate's own local debug stats: rx, parse_errors,
        # hold_events, gated_messages, holding. Prefixed on the way in so
        # they can't collide with the broadcaster's own field names.
        self.create_subscription(
            String, "/v2v/rx_stats", self._on_gate_stats_json, 10
        )
        self.create_subscription(
            PoseStamped, "/v2v/tx_pose", self._on_tx_pose, 10
        )

    def _on_gate_stats_json(self, msg):
        try:
            stats = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        with self._data_lock:
            for k, v in stats.items():
                self.data[f"gate_{k}"] = v
            self.data["_updated"] = time.time()

    def _on_tx_pose(self, msg):
        self._set("my_x", round(msg.pose.position.x, 3))
        self._set("my_y", round(msg.pose.position.y, 3))


PAGE_TEMPLATE = """<!doctype html>
<meta name=viewport content="width=device-width, initial-scale=1">
<title>V2V Dashboard -- __MY_LABEL__</title>
<style>
  :root { color-scheme: dark; }
  body {
    margin: 0; background: #0b0d10; color: #d7dbe0;
    font: 14px/1.4 -apple-system, Segoe UI, Roboto, sans-serif;
  }
  header {
    padding: 10px 16px; background: #14171c; border-bottom: 1px solid #262b33;
    display: flex; align-items: baseline; gap: 12px;
  }
  header h1 { font-size: 16px; margin: 0; font-weight: 600; }
  header .stale { color: #ff5c5c; font-weight: 600; }
  main {
    display: grid; gap: 14px; padding: 14px;
    grid-template-columns: 1fr 1fr; align-items: start;
  }
  @media (max-width: 900px) { main { grid-template-columns: 1fr; } }
  .cams { display: grid; gap: 14px; grid-template-columns: 1fr 1fr; grid-column: 1 / -1; }
  @media (max-width: 600px) { .cams { grid-template-columns: 1fr; } }
  .panel {
    background: #14171c; border: 1px solid #262b33; border-radius: 10px;
    overflow: hidden;
  }
  .panel h2 {
    font-size: 12px; text-transform: uppercase; letter-spacing: .04em;
    color: #8a93a1; margin: 0; padding: 8px 12px; border-bottom: 1px solid #262b33;
  }
  .cams img { width: 100%; display: block; background: #000; aspect-ratio: 4/3; object-fit: contain; }
  .nodes { grid-column: 1 / -1; }
  .nodes ul { list-style: none; margin: 0; padding: 10px 12px; display: flex; flex-wrap: wrap; gap: 8px; }
  .nodes li {
    background: #1c2027; border: 1px solid #262b33; border-radius: 6px;
    padding: 4px 9px; font: 12px/1.4 ui-monospace, SFMono-Regular, Menlo, monospace;
    color: #a7f3d0;
  }
  .nodes li.empty {
    background: none; border: none; color: #8a93a1;
    font: 13px/1.4 -apple-system, sans-serif; padding: 0;
  }
  table { width: 100%; border-collapse: collapse; }
  td { padding: 7px 12px; border-bottom: 1px solid #1c2027; }
  td.k { color: #8a93a1; width: 55%; }
  td.v { font-variant-numeric: tabular-nums; text-align: right; font-weight: 600; }
  .ok { color: #4ade80; }
  .warn { color: #fbbf24; }
  .bad { color: #ff5c5c; }
  .muted { color: #8a93a1; font-weight: 400; }
</style>
<header>
  <h1>V2V Dashboard</h1>
  <span class=muted>viewing from __MY_LABEL__ (__ROLE__)</span>
  <span id=stale class=stale style="display:none">-- STALE DATA --</span>
</header>
<main>
  <div class=cams>
    <div class=panel>
      <h2>QCar 2 camera</h2>
      <img id=cam-qcar2 src="__QCAR2_CAM__">
    </div>
    <div class=panel>
      <h2>ROSbot 3 camera</h2>
      <img id=cam-rosbot3 src="__ROSBOT3_CAM__">
    </div>
  </div>
  <div class=panel>
    <h2>Link &amp; motion</h2>
    <table id=t-link></table>
  </div>
  <div class=panel>
    <h2>Safety state</h2>
    <table id=t-safety></table>
  </div>
  <div class="panel nodes">
    <h2 id=nodes-title>Active ROS nodes (__MY_LABEL__)</h2>
    <ul id=nodes-list></ul>
  </div>
</main>
<script>
const FIELDS_LINK = [
  ["rx", "Packets received"],
  ["tx", "Packets sent"],
  ["gate_rx", "Gate packets received"],
  ["tx_errors", "Send errors"],
  ["seq", "Sequence #"],
  ["seq_drops", "Sequence drops"],
  ["parse_errors", "Parse errors"],
  ["gate_parse_errors", "Gate parse errors"],
  ["age_s", "Data age (s)"],
  ["gap", "Gap (m)"],
  ["speed", "Speed (m/s)"],
  ["rosbot_speed", "ROSbot speed, as seen by QCar (m/s)"],
  ["rosbot_x", "ROSbot x, in QCar frame"],
  ["rosbot_y", "ROSbot y, in QCar frame"],
  ["my_x", "My x, own frame"],
  ["my_y", "My y, own frame"],
  ["command_tx", "Commands sent"],
  ["command_errors", "Command errors"],
];
const FIELDS_SAFETY = [
  ["alive", "Link alive"],
  ["on_path", "On path"],
  ["blocked", "Blocked"],
  ["blocked_distance", "Blocked distance (m)"],
  ["detour_intent", "Detour intent"],
  ["hold_active", "Hold active (governor)"],
  ["gate_holding", "Being held (gate)"],
  ["gate_hold_events", "Hold events (gate)"],
  ["gate_gated_messages", "Gated messages (gate)"],
  ["encounter", "Encounter state"],
  ["allow_overtake", "Allow overtake (MPC)"],
  ["motion_enable", "Motion enabled"],
];

function fmt(key, val) {
  if (val === undefined || val === null) return ["--", "muted"];
  if (typeof val === "boolean") {
    const goodTrue = ["alive", "on_path", "motion_enable", "allow_overtake"];
    const badTrue = ["blocked", "hold_active", "gate_holding", "detour_intent"];
    let cls = "muted";
    if (goodTrue.includes(key)) cls = val ? "ok" : "bad";
    else if (badTrue.includes(key)) cls = val ? "warn" : "ok";
    return [val ? "true" : "false", cls];
  }
  return [String(val), ""];
}

function render(tbl, fields, data) {
  tbl.innerHTML = fields.map(([key, label]) => {
    if (!(key in data)) return "";
    const [text, cls] = fmt(key, data[key]);
    return `<tr><td class=k>${label}</td><td class="v ${cls}">${text}</td></tr>`;
  }).join("");
}

function renderNodes(data) {
  const el = document.getElementById("nodes-list");
  const nodes = data.nodes || [];
  el.innerHTML = nodes.length === 0
    ? '<li class=empty>-- none discovered --</li>'
    : nodes.map(n => `<li>${n}</li>`).join("");
}

async function poll() {
  try {
    const res = await fetch("/data.json", {cache: "no-store"});
    const data = await res.json();
    render(document.getElementById("t-link"), FIELDS_LINK, data);
    render(document.getElementById("t-safety"), FIELDS_SAFETY, data);
    renderNodes(data);
    const age = data._updated ? (Date.now() / 1000 - data._updated) : 999;
    document.getElementById("stale").style.display = age > 3 ? "inline" : "none";
  } catch (e) {
    document.getElementById("stale").style.display = "inline";
  }
}
setInterval(poll, 400);
poll();

// MJPEG <img> streams don't auto-reconnect once a connection breaks --
// periodically force a fresh request so a peer that restarts (or a
// dropped connection) recovers without a manual page reload.
function reconnect(id, base) {
  const img = document.getElementById(id);
  img.src = base + (base.includes("?") ? "&" : "?") + "t=" + Date.now();
}
setInterval(() => {
  reconnect("cam-qcar2", "__QCAR2_CAM__");
  reconnect("cam-rosbot3", "__ROSBOT3_CAM__");
}, 8000);
</script>
"""


class Handler(BaseHTTPRequestHandler):

    protocol_version = "HTTP/1.0"
    dashboard = None       # set by main() before serving
    page_html = b""        # rendered once at startup

    def log_message(self, *args):
        pass                      # keep the console free for ROS output

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(self.page_html)))
            self.end_headers()
            self.wfile.write(self.page_html)
            return

        if self.path == "/data.json":
            body = json.dumps(self.dashboard.snapshot()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()
            try:
                while True:
                    jpeg = self.dashboard.latest_jpeg()
                    if jpeg is None:
                        threading.Event().wait(0.1)
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                    )
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    threading.Event().wait(0.05)      # ~20 fps ceiling
            except (BrokenPipeError, ConnectionResetError):
                pass                                   # browser tab closed
            return

        self.send_error(404)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--role", required=True, choices=sorted(ROLES))
    ap.add_argument("--camera-topic", default=None,
                     help="overrides the role's default camera topic")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--quality", type=int, default=80)
    ap.add_argument("--peer-host", required=True,
                     help="IP of the OTHER robot running this same script")
    ap.add_argument("--peer-port", type=int, default=None,
                     help="defaults to --port (peer runs on the same port)")
    ap.add_argument("--rosbot3-camera-url", default=ROSBOT3_CAMERA_URL,
                     help="ROSbot 3 camera MJPEG URL (web_video_server on the "
                          "robot itself); pass '' to use this dashboard's own "
                          "/stream instead")
    args = ap.parse_args()

    if cv2 is None:
        raise SystemExit(
            "python3-opencv is required for JPEG encoding "
            "(try: python3 -c 'import cv2')"
        )

    role_cfg = ROLES[args.role]
    camera_topic = args.camera_topic or role_cfg["camera_topic"]
    peer_port = args.peer_port or args.port

    my_cam_src = "/stream"
    peer_cam_src = f"http://{args.peer_host}:{peer_port}/stream"
    if args.role == "qcar2":
        qcar2_cam_src, rosbot3_cam_src = my_cam_src, peer_cam_src
    else:
        qcar2_cam_src, rosbot3_cam_src = peer_cam_src, my_cam_src

    # web_video_server on the robot serves the ROSbot camera directly, so both
    # instances point at the same place and neither has to relay it.
    if args.rosbot3_camera_url:
        rosbot3_cam_src = args.rosbot3_camera_url

    page_html = (
        PAGE_TEMPLATE
        .replace("__MY_LABEL__", role_cfg["label"])
        .replace("__ROLE__", args.role)
        .replace("__QCAR2_CAM__", qcar2_cam_src)
        .replace("__ROSBOT3_CAM__", rosbot3_cam_src)
    ).encode()

    rclpy.init()
    node = Dashboard(args.role, camera_topic, args.quality)

    Handler.dashboard = node
    Handler.page_html = page_html
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    host = socket.gethostbyname(socket.gethostname())
    print(f"\n  open  http://{host}:{args.port}/   "
          f"(or http://<this-robot-ip>:{args.port}/)\n"
          f"  showing both cameras -- local + peer at "
          f"http://{args.peer_host}:{peer_port}/stream\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
