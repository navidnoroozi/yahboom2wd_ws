#!/usr/bin/env python3
"""Create team-level GIF and interactive HTML animations for Yahboom DMPC bags.

The animation uses common world-frame poses from /dmpc/<robot>/pose_world when
available, and falls back to /<robot>/odom if pose_world is not recorded.  It
visualizes team trajectory, robot headings, inter-robot distance, hold-zone
state, and one circular obstacle with its inflated safety margin.

A GIF is still written for easy sharing.  Because GIF files are static and do
not support interactive widgets, this script also writes an HTML animation with
a speed slider and a time scrubber.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from typing import Dict, Iterable, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


PoseSeries = Dict[str, List[float]]
Series = Dict[str, List[float]]


def expand_path(path_text: str) -> pathlib.Path:
    return pathlib.Path(path_text).expanduser().resolve()


def infer_storage_id(bag_path: pathlib.Path) -> str:
    if any(bag_path.glob("*.mcap")):
        return "mcap"
    if any(bag_path.glob("*.db3")):
        return "sqlite3"
    metadata = bag_path / "metadata.yaml"
    if metadata.exists():
        text = metadata.read_text(encoding="utf-8", errors="ignore")
        if "storage_identifier: mcap" in text or "storage_id: mcap" in text:
            return "mcap"
        if "storage_identifier: sqlite3" in text or "storage_id: sqlite3" in text:
            return "sqlite3"
    raise RuntimeError(f"Could not infer bag storage backend for {bag_path}. Use --storage-id mcap or sqlite3.")


def normalize_namespace(namespace: str) -> str:
    namespace = namespace.strip()
    if namespace.startswith("/"):
        namespace = namespace[1:]
    if namespace.endswith("/"):
        namespace = namespace[:-1]
    return namespace


def ns_topic(namespace: str, relative_name: str) -> str:
    return f"/{normalize_namespace(namespace)}/{relative_name.lstrip('/')}"


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap(values: Iterable[float]) -> np.ndarray:
    vals = np.asarray(list(values), dtype=float)
    if vals.size == 0:
        return vals
    return np.unwrap(vals)


def open_reader(bag_path: pathlib.Path, storage_id: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)
    return reader


def choose_existing_topic(type_map: Dict[str, str], candidates: Iterable[str], expected_type: Optional[str] = None) -> Optional[str]:
    for name in candidates:
        if name in type_map and (expected_type is None or type_map[name] == expected_type):
            return name
    for candidate in candidates:
        suffix = candidate if candidate.startswith("/") else f"/{candidate}"
        for name, typ in type_map.items():
            if name.endswith(suffix) and (expected_type is None or typ == expected_type):
                return name
    return None


def empty_pose_series() -> PoseSeries:
    return {"t": [], "x": [], "y": [], "yaw": [], "src": []}


def _append_pose(series: PoseSeries, t: float, x: float, y: float, yaw: float, src: str) -> None:
    series["t"].append(t)
    series["x"].append(float(x))
    series["y"].append(float(y))
    series["yaw"].append(float(yaw))
    series["src"].append(src)


def _interp(values_t: np.ndarray, values_y: np.ndarray, t_common: np.ndarray, fill: float = np.nan) -> np.ndarray:
    if values_t.size == 0:
        return np.full_like(t_common, fill, dtype=float)
    if values_t.size == 1:
        return np.full_like(t_common, values_y[0], dtype=float)
    order = np.argsort(values_t)
    return np.interp(t_common, values_t[order], values_y[order])


def make_common_pose_grid(pose_by_ns: Dict[str, PoseSeries], namespaces: List[str], sample_dt: float) -> Tuple[np.ndarray, Dict[str, Dict[str, np.ndarray]]]:
    available = [ns for ns in namespaces if len(pose_by_ns[ns]["t"]) > 0]
    if not available:
        raise RuntimeError("No pose_world or odom pose topics were found for the requested namespaces.")
    start = max(float(np.min(pose_by_ns[ns]["t"])) for ns in available)
    end = min(float(np.max(pose_by_ns[ns]["t"])) for ns in available)
    if end <= start:
        base_ns = available[0]
        t_common = np.asarray(pose_by_ns[base_ns]["t"], dtype=float)
    else:
        n = max(2, int(math.ceil((end - start) / max(sample_dt, 1e-3))) + 1)
        t_common = np.linspace(start, end, n)
    common: Dict[str, Dict[str, np.ndarray]] = {}
    for ns in available:
        t = np.asarray(pose_by_ns[ns]["t"], dtype=float)
        x = np.asarray(pose_by_ns[ns]["x"], dtype=float)
        y = np.asarray(pose_by_ns[ns]["y"], dtype=float)
        yaw = unwrap(pose_by_ns[ns]["yaw"])
        common[ns] = {"x": _interp(t, x, t_common), "y": _interp(t, y, t_common), "yaw": _interp(t, yaw, t_common)}
    return t_common, common


def _topic_has_samples(series_by_ns: Dict[str, Series], key: str = "t") -> bool:
    return any(len(series.get(key, [])) > 0 for series in series_by_ns.values())


def should_show_obstacle(mode: str, obstacle_data_available: bool) -> bool:
    if mode == "always":
        return True
    if mode == "never":
        return False
    return obstacle_data_available


def load_bag_data(bag_path: pathlib.Path, storage_id: str, namespaces: List[str], sample_dt: float):
    reader = open_reader(bag_path, storage_id)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    pose_topics: Dict[str, Optional[str]] = {}
    odom_topics: Dict[str, Optional[str]] = {}
    obstacle_metrics_topics: Dict[str, Optional[str]] = {}
    for ns in namespaces:
        pose_topics[ns] = choose_existing_topic(type_map, [f"/dmpc/{ns}/pose_world"], "geometry_msgs/msg/PoseStamped")
        odom_topics[ns] = choose_existing_topic(type_map, [ns_topic(ns, "odom")], "nav_msgs/msg/Odometry")
        obstacle_metrics_topics[ns] = choose_existing_topic(type_map, [f"/dmpc/{ns}/obstacle_metrics"], "geometry_msgs/msg/Vector3Stamped")

    hold_topic = choose_existing_topic(type_map, ["/dmpc/two_robot/hold_state"], "geometry_msgs/msg/Vector3Stamped")
    obstacle_thresholds_topic = choose_existing_topic(type_map, ["/dmpc/two_robot/obstacle_thresholds"], "geometry_msgs/msg/Vector3Stamped")

    selected_topics = set()
    selected_topics.update(t for t in pose_topics.values() if t)
    selected_topics.update(t for ns, t in odom_topics.items() if t and pose_topics[ns] is None)
    selected_topics.update(t for t in obstacle_metrics_topics.values() if t)
    if hold_topic:
        selected_topics.add(hold_topic)
    if obstacle_thresholds_topic:
        selected_topics.add(obstacle_thresholds_topic)
    if not selected_topics:
        raise RuntimeError("No pose_world or odom topics found. Check --bag and --namespaces.")

    msg_type_map = {name: get_message(type_map[name]) for name in selected_topics}
    pose_by_ns = {ns: empty_pose_series() for ns in namespaces}
    hold_raw: Series = {"t": [], "active": [], "selected_error": [], "pair_error": []}
    obs_thresh_raw: Series = {"t": [], "d_obs_enter": [], "d_obs_exit": [], "warning": []}
    obs_raw: Dict[str, Series] = {ns: {"t": [], "distance": [], "clearance": [], "active": []} for ns in namespaces}

    print("Selected animation topics:")
    for ns in namespaces:
        print(f"  {ns}: pose_world={pose_topics[ns] or 'not found'}, odom={odom_topics[ns] or 'not found'}, obstacle_metrics={obstacle_metrics_topics[ns] or 'not found'}")
    print(f"  hold_state: {hold_topic or 'not found'}")
    print(f"  obstacle_thresholds: {obstacle_thresholds_topic or 'not found'}")
    print(f"Using storage_id={storage_id}")

    t0: Optional[int] = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in msg_type_map:
            continue
        if t0 is None:
            t0 = timestamp
        t = (timestamp - t0) * 1e-9
        msg = deserialize_message(data, msg_type_map[topic])
        for ns in namespaces:
            if topic == pose_topics[ns]:
                p = msg.pose.position
                q = msg.pose.orientation
                _append_pose(pose_by_ns[ns], t, p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w), "pose_world")
            elif topic == odom_topics[ns] and pose_topics[ns] is None:
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                _append_pose(pose_by_ns[ns], t, p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w), "odom_fallback")
            elif topic == obstacle_metrics_topics[ns]:
                obs_raw[ns]["t"].append(t)
                obs_raw[ns]["distance"].append(float(msg.vector.x))
                obs_raw[ns]["clearance"].append(float(msg.vector.y))
                obs_raw[ns]["active"].append(float(msg.vector.z))
        if topic == hold_topic:
            hold_raw["t"].append(t)
            hold_raw["active"].append(float(msg.vector.x))
            hold_raw["selected_error"].append(float(msg.vector.y))
            hold_raw["pair_error"].append(float(msg.vector.z))
        elif topic == obstacle_thresholds_topic:
            obs_thresh_raw["t"].append(t)
            obs_thresh_raw["d_obs_enter"].append(float(msg.vector.x))
            obs_thresh_raw["d_obs_exit"].append(float(msg.vector.y))
            obs_thresh_raw["warning"].append(float(msg.vector.z))

    t_common, common = make_common_pose_grid(pose_by_ns, namespaces, sample_dt=sample_dt)

    hold = {
        "active": _interp(np.asarray(hold_raw["t"], dtype=float), np.asarray(hold_raw["active"], dtype=float), t_common, fill=0.0),
        "selected_error": _interp(np.asarray(hold_raw["t"], dtype=float), np.asarray(hold_raw["selected_error"], dtype=float), t_common, fill=np.nan),
        "pair_error": _interp(np.asarray(hold_raw["t"], dtype=float), np.asarray(hold_raw["pair_error"], dtype=float), t_common, fill=np.nan),
        "recorded": bool(hold_raw["t"]),
    }
    obstacle = {}
    for ns in namespaces:
        obstacle[ns] = {
            "distance": _interp(np.asarray(obs_raw[ns]["t"], dtype=float), np.asarray(obs_raw[ns]["distance"], dtype=float), t_common, fill=np.nan),
            "clearance": _interp(np.asarray(obs_raw[ns]["t"], dtype=float), np.asarray(obs_raw[ns]["clearance"], dtype=float), t_common, fill=np.nan),
            "active": _interp(np.asarray(obs_raw[ns]["t"], dtype=float), np.asarray(obs_raw[ns]["active"], dtype=float), t_common, fill=0.0),
            "recorded": bool(obs_raw[ns]["t"]),
        }
    thresholds = {
        "recorded": bool(obs_thresh_raw["t"]),
        "d_obs_enter": obs_thresh_raw["d_obs_enter"][-1] if obs_thresh_raw["d_obs_enter"] else np.nan,
        "d_obs_exit": obs_thresh_raw["d_obs_exit"][-1] if obs_thresh_raw["d_obs_exit"] else np.nan,
        "warning": obs_thresh_raw["warning"][-1] if obs_thresh_raw["warning"] else np.nan,
    }
    return t_common, common, hold, obstacle, thresholds, _topic_has_samples(obs_raw)


def write_interactive_html(html_path: pathlib.Path, data: dict) -> None:
    template = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Yahboom DMPC team animation</title>
<style>
  body { font-family: system-ui, -apple-system, Segoe UI, sans-serif; margin: 20px; }
  #wrap { max-width: 980px; }
  svg { width: 100%; height: auto; border: 1px solid #ccc; background: #fff; }
  .controls { margin-top: 12px; display: grid; grid-template-columns: auto 1fr auto; gap: 10px; align-items: center; }
  .readout { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; white-space: pre; background: #f7f7f7; padding: 10px; border-radius: 6px; }
  button { padding: 6px 12px; }
</style>
</head>
<body>
<div id="wrap">
<h2>Yahboom two-robot DMPC animation</h2>
<svg id="svg" viewBox="0 0 900 700" role="img" aria-label="Team animation"></svg>
<div class="controls"><label>Time</label><input id="frame" type="range" min="0" max="1" value="0" step="1"><span id="timeLabel"></span></div>
<div class="controls"><label>Playback speed</label><input id="speed" type="range" min="0.1" max="5" step="0.1" value="1"><span id="speedLabel">1.0x</span></div>
<div class="controls"><button id="play">Pause</button><span></span><span></span></div>
<div id="info" class="readout"></div>
</div>
<script>
const data = __DATA__;
const svg = document.getElementById('svg');
const frameSlider = document.getElementById('frame');
const speedSlider = document.getElementById('speed');
const speedLabel = document.getElementById('speedLabel');
const timeLabel = document.getElementById('timeLabel');
const info = document.getElementById('info');
const playBtn = document.getElementById('play');
const W = 900, H = 700, pad = 60;
let playing = true;
let i = 0;
let last = performance.now();
let acc = 0;
frameSlider.max = data.t.length - 1;
function sx(x){ return pad + (x - data.bounds.xmin) / (data.bounds.xmax - data.bounds.xmin) * (W - 2*pad); }
function sy(y){ return H - pad - (y - data.bounds.ymin) / (data.bounds.ymax - data.bounds.ymin) * (H - 2*pad); }
function sr(r){ return r / (data.bounds.xmax - data.bounds.xmin) * (W - 2*pad); }
function el(name, attrs){ const e = document.createElementNS('http://www.w3.org/2000/svg', name); for (const [k,v] of Object.entries(attrs)) e.setAttribute(k,v); svg.appendChild(e); return e; }
svg.innerHTML = '';
el('rect', {x:0, y:0, width:W, height:H, fill:'white'});
el('line', {x1:sx(data.bounds.xmin), y1:sy(0), x2:sx(data.bounds.xmax), y2:sy(0), stroke:'#eee'});
el('line', {x1:sx(0), y1:sy(data.bounds.ymin), x2:sx(0), y2:sy(data.bounds.ymax), stroke:'#eee'});
if (data.obstacle.show) {
  el('circle', {cx:sx(data.obstacle.cx), cy:sy(data.obstacle.cy), r:sr(data.obstacle.radius + data.obstacle.margin), fill:'none', stroke:'#555', 'stroke-width':2, 'stroke-dasharray':'7,5'});
  el('circle', {cx:sx(data.obstacle.cx), cy:sy(data.obstacle.cy), r:sr(data.obstacle.radius), fill:'none', stroke:'#222', 'stroke-width':2});
  el('text', {x:sx(data.obstacle.cx)+8, y:sy(data.obstacle.cy)-8, 'font-size':14}).textContent = 'obstacle + inflated margin';
}
const colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'];
const paths = {}, robots = {}, headings = {}, labels = {};
const pairLine = el('line', {x1:0,y1:0,x2:0,y2:0, stroke:'#111', 'stroke-width':2, 'stroke-dasharray':'6,4'});
data.namespaces.forEach((ns, idx) => {
  paths[ns] = el('polyline', {points:'', fill:'none', stroke:colors[idx % colors.length], 'stroke-width':3});
  robots[ns] = el('circle', {cx:0, cy:0, r:8, fill:colors[idx % colors.length]});
  headings[ns] = el('line', {x1:0,y1:0,x2:0,y2:0, stroke:colors[idx % colors.length], 'stroke-width':4});
  labels[ns] = el('text', {x:0, y:0, 'font-size':14, fill:colors[idx % colors.length]});
});
function render(k){
  i = Math.max(0, Math.min(data.t.length - 1, k));
  frameSlider.value = i;
  const pts = {};
  data.namespaces.forEach((ns, idx) => {
    const xs = data.robots[ns].x, ys = data.robots[ns].y, yaw = data.robots[ns].yaw[i];
    const x = xs[i], y = ys[i]; pts[ns] = {x,y};
    let poly = '';
    for (let j=0; j<=i; ++j) poly += sx(xs[j]).toFixed(1)+','+sy(ys[j]).toFixed(1)+' ';
    paths[ns].setAttribute('points', poly.trim());
    robots[ns].setAttribute('cx', sx(x)); robots[ns].setAttribute('cy', sy(y));
    headings[ns].setAttribute('x1', sx(x)); headings[ns].setAttribute('y1', sy(y));
    headings[ns].setAttribute('x2', sx(x + 0.18*Math.cos(yaw))); headings[ns].setAttribute('y2', sy(y + 0.18*Math.sin(yaw)));
    labels[ns].setAttribute('x', sx(x)+10); labels[ns].setAttribute('y', sy(y)-10); labels[ns].textContent = ns;
  });
  const a = data.namespaces[0], b = data.namespaces[1];
  pairLine.setAttribute('x1', sx(pts[a].x)); pairLine.setAttribute('y1', sy(pts[a].y));
  pairLine.setAttribute('x2', sx(pts[b].x)); pairLine.setAttribute('y2', sy(pts[b].y));
  const dx = pts[a].x - pts[b].x, dy = pts[a].y - pts[b].y;
  const dist = Math.hypot(dx, dy);
  const target = data.d_safe + data.formation_margin;
  let minClear = 'n/a';
  if (data.obstacle.recorded) {
    let vals = data.namespaces.map(ns => data.obstacle.metrics[ns].clearance[i]).filter(v => Number.isFinite(v));
    if (vals.length) minClear = Math.min(...vals).toFixed(3) + ' m';
  }
  timeLabel.textContent = data.t[i].toFixed(1) + ' s';
  speedLabel.textContent = parseFloat(speedSlider.value).toFixed(1) + 'x';
  info.textContent = `t: ${data.t[i].toFixed(1)} s\ninter-robot distance: ${dist.toFixed(3)} m\nsafety margin: ${(dist-data.d_safe).toFixed(3)} m\nformation error: ${(dist-target).toFixed(3)} m\nhold active: ${data.hold.active[i] > 0.5 ? 'yes' : 'no'}\nminimum obstacle clearance: ${minClear}`;
}
frameSlider.addEventListener('input', () => { playing = false; playBtn.textContent = 'Play'; render(parseInt(frameSlider.value)); });
speedSlider.addEventListener('input', () => speedLabel.textContent = parseFloat(speedSlider.value).toFixed(1) + 'x');
playBtn.addEventListener('click', () => { playing = !playing; playBtn.textContent = playing ? 'Pause' : 'Play'; last = performance.now(); });
function tick(now){
  const dt = (now - last) / 1000; last = now;
  if (playing) {
    acc += dt * parseFloat(speedSlider.value);
    const nominalDt = Math.max(0.02, data.nominal_dt);
    while (acc >= nominalDt) { acc -= nominalDt; i = (i + 1) % data.t.length; }
    render(i);
  }
  requestAnimationFrame(tick);
}
render(0); requestAnimationFrame(tick);
</script>
</body>
</html>
'''
    html = template.replace("__DATA__", json.dumps(data))
    html_path.write_text(html, encoding="utf-8")
    print(f"Saved: {html_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Animate team-level Yahboom two-robot DMPC bag data.")
    parser.add_argument("--bag", required=True, help="Path to ROS 2 bag folder.")
    parser.add_argument("--namespaces", nargs="+", default=["robot1", "robot2"], help="Robot namespaces, e.g. robot1 robot2.")
    parser.add_argument("--storage-id", default="auto", choices=["auto", "mcap", "sqlite3"], help="rosbag2 storage backend.")
    parser.add_argument("--output-dir", default="", help="Output directory. Default: <bag>/team_animation")
    parser.add_argument("--gif-path", default="", help="GIF output path. Default: <output-dir>/team_motion.gif")
    parser.add_argument("--html-path", default="", help="Interactive HTML output path. Default: <output-dir>/team_motion_interactive.html")
    parser.add_argument("--no-html", action="store_true", help="Disable interactive HTML output.")
    parser.add_argument("--fps", type=float, default=8.0, help="GIF frames per second. GIF speed is fixed at export time.")
    parser.add_argument("--step", type=int, default=1, help="Use every Nth interpolated frame.")
    parser.add_argument("--sample-dt", type=float, default=0.1, help="Interpolation step for animation data [s].")
    parser.add_argument("--max-frames", type=int, default=600, help="Cap rendered frames to keep GIF size reasonable. Use 0 for no cap.")
    parser.add_argument("--d-safe", type=float, default=0.65)
    parser.add_argument("--formation-margin", type=float, default=0.15)
    parser.add_argument("--show-obstacle", choices=["auto", "always", "never"], default="auto")
    parser.add_argument("--obstacle-center-x", type=float, default=1.0)
    parser.add_argument("--obstacle-center-y", type=float, default=-0.33)
    parser.add_argument("--obstacle-radius", type=float, default=0.15)
    parser.add_argument("--obstacle-margin", type=float, default=0.10)
    args = parser.parse_args()

    bag_path = expand_path(args.bag)
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag folder does not exist: {bag_path}")
    namespaces = [normalize_namespace(ns) for ns in args.namespaces]
    if len(namespaces) < 2:
        raise RuntimeError("At least two namespaces are required for team animation.")
    storage_id = infer_storage_id(bag_path) if args.storage_id == "auto" else args.storage_id
    outdir = expand_path(args.output_dir) if args.output_dir else bag_path / "team_animation"
    outdir.mkdir(parents=True, exist_ok=True)
    gif_path = expand_path(args.gif_path) if args.gif_path else outdir / "team_motion.gif"
    html_path = expand_path(args.html_path) if args.html_path else outdir / "team_motion_interactive.html"

    t_common, common, hold, obstacle, thresholds, obstacle_data_available = load_bag_data(bag_path, storage_id, namespaces, sample_dt=args.sample_dt)

    frame_indices = np.arange(0, len(t_common), max(1, int(args.step)), dtype=int)
    if args.max_frames > 0 and frame_indices.size > args.max_frames:
        frame_indices = np.linspace(0, len(t_common) - 1, args.max_frames).astype(int)

    xs = np.concatenate([common[ns]["x"] for ns in namespaces])
    ys = np.concatenate([common[ns]["y"] for ns in namespaces])
    draw_obstacle = should_show_obstacle(args.show_obstacle, bool(obstacle_data_available or thresholds["recorded"]))
    if draw_obstacle:
        inflated = args.obstacle_radius + args.obstacle_margin
        xs = np.concatenate([xs, np.asarray([args.obstacle_center_x - inflated, args.obstacle_center_x + inflated])])
        ys = np.concatenate([ys, np.asarray([args.obstacle_center_y - inflated, args.obstacle_center_y + inflated])])
    pad = max(0.5, args.d_safe)
    xmin, xmax = float(np.nanmin(xs) - pad), float(np.nanmax(xs) + pad)
    ymin, ymax = float(np.nanmin(ys) - pad), float(np.nanmax(ys) + pad)

    target = args.d_safe + args.formation_margin
    safe_radius_visual = args.d_safe / 2.0

    fig, ax = plt.subplots(figsize=(7.5, 7.0))
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.grid(True)

    if draw_obstacle:
        ax.add_patch(Circle((args.obstacle_center_x, args.obstacle_center_y), args.obstacle_radius + args.obstacle_margin, fill=False, linestyle="--", linewidth=1.8, alpha=0.85, label="inflated obstacle"))
        ax.add_patch(Circle((args.obstacle_center_x, args.obstacle_center_y), args.obstacle_radius, fill=False, linestyle="-", linewidth=1.8, alpha=0.85, label="physical obstacle"))
        ax.plot([args.obstacle_center_x], [args.obstacle_center_y], marker="x", markersize=8, label="obstacle center")

    cmap = plt.get_cmap("tab10")
    colors = {ns: cmap(i % 10) for i, ns in enumerate(namespaces)}

    trails = {}
    points = {}
    labels = {}
    safety_circles = {}
    heading_quivers = {}
    for ns in namespaces:
        (trail,) = ax.plot([], [], color=colors[ns], linewidth=1.8, label=f"{ns} path")
        trails[ns] = trail
        points[ns] = ax.scatter([], [], color=[colors[ns]], s=55)
        labels[ns] = ax.text(0.0, 0.0, ns)
        safety_circles[ns] = Circle((0.0, 0.0), safe_radius_visual, fill=False, linestyle="--", linewidth=1.0, alpha=0.45, color=colors[ns])
        ax.add_patch(safety_circles[ns])
        heading_quivers[ns] = ax.quiver([], [], [], [], angles="xy", scale_units="xy", scale=1.0, color=colors[ns], width=0.006)

    (pair_line,) = ax.plot([], [], "k--", alpha=0.65, label="robot pair")
    text = ax.text(0.02, 0.98, "", transform=ax.transAxes, ha="left", va="top", fontsize=10, bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.75"))
    ax.legend(loc="lower right")

    def _frame(fi: int):
        ti = int(frame_indices[fi])
        xy = {}
        for ns in namespaces:
            x = float(common[ns]["x"][ti])
            y = float(common[ns]["y"][ti])
            yaw = float(common[ns]["yaw"][ti])
            xy[ns] = (x, y)
            trails[ns].set_data(common[ns]["x"][: ti + 1], common[ns]["y"][: ti + 1])
            points[ns].set_offsets(np.array([[x, y]]))
            labels[ns].set_position((x + 0.03, y + 0.03))
            safety_circles[ns].center = (x, y)
            heading_quivers[ns].set_offsets(np.array([[x, y]]))
            heading_quivers[ns].set_UVC(np.array([0.18 * math.cos(yaw)]), np.array([0.18 * math.sin(yaw)]))

        ns1, ns2 = namespaces[0], namespaces[1]
        x1, y1 = xy[ns1]
        x2, y2 = xy[ns2]
        dist = math.hypot(x1 - x2, y1 - y2)
        pair_line.set_data([x1, x2], [y1, y2])
        safety_margin = dist - args.d_safe
        formation_error = dist - target
        hold_active = bool(hold["active"][ti] > 0.5)
        clearance_lines = []
        if any(obstacle[ns]["recorded"] for ns in namespaces):
            for ns in namespaces:
                c = obstacle[ns]["clearance"][ti]
                if np.isfinite(c):
                    clearance_lines.append(f"{ns} obs clr: {c:.3f} m")
        ax.set_title(f"Two-robot DMPC team motion | t={t_common[ti]:.1f} s")
        text.set_text(
            f"distance: {dist:.3f} m\n"
            f"d_safe: {args.d_safe:.3f} m\n"
            f"safety margin: {safety_margin:.3f} m\n"
            f"target distance: {target:.3f} m\n"
            f"formation error: {formation_error:+.3f} m\n"
            f"hold active: {'yes' if hold_active else 'no'}"
            + ("\n" + "\n".join(clearance_lines) if clearance_lines else "")
        )
        return [*trails.values(), *points.values(), *labels.values(), *safety_circles.values(), *heading_quivers.values(), pair_line, text]

    ani = FuncAnimation(fig, _frame, frames=len(frame_indices), interval=1000.0 / max(args.fps, 0.1), blit=False)
    print(f"Rendering {len(frame_indices)} frames to {gif_path} ...")
    ani.save(str(gif_path), writer=PillowWriter(fps=max(args.fps, 0.1)))
    plt.close(fig)
    print(f"Saved: {gif_path}")

    if not args.no_html:
        # Use the same downsampled frames for the HTML so it stays lightweight.
        t_html = t_common[frame_indices]
        robots_html = {ns: {"x": common[ns]["x"][frame_indices].round(5).tolist(), "y": common[ns]["y"][frame_indices].round(5).tolist(), "yaw": common[ns]["yaw"][frame_indices].round(5).tolist()} for ns in namespaces}
        obstacle_html = {
            "show": draw_obstacle,
            "recorded": any(obstacle[ns]["recorded"] for ns in namespaces),
            "cx": args.obstacle_center_x,
            "cy": args.obstacle_center_y,
            "radius": args.obstacle_radius,
            "margin": args.obstacle_margin,
            "metrics": {ns: {"clearance": [None] * len(t_html)} for ns in namespaces},
        }
        # json cannot serialize nan through nan_to_num with nan=None in older numpy, so normalize manually.
        for ns in namespaces:
            vals = []
            arr = obstacle[ns]["clearance"][frame_indices]
            for v in arr:
                vals.append(None if not np.isfinite(v) else round(float(v), 5))
            obstacle_html["metrics"][ns]["clearance"] = vals
        html_data = {
            "t": [round(float(v), 5) for v in t_html],
            "nominal_dt": float(np.median(np.diff(t_html))) if len(t_html) > 1 else 0.1,
            "namespaces": namespaces,
            "robots": robots_html,
            "d_safe": args.d_safe,
            "formation_margin": args.formation_margin,
            "hold": {"active": [round(float(v), 3) for v in hold["active"][frame_indices]]},
            "obstacle": obstacle_html,
            "bounds": {"xmin": xmin, "xmax": xmax, "ymin": ymin, "ymax": ymax},
        }
        write_interactive_html(html_path, html_data)


if __name__ == "__main__":
    main()
