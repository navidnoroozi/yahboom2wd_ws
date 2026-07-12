#!/usr/bin/env python3
"""Team-level plots for two-robot Yahboom DMPC ROS 2 bags.

This script complements plot_yahboom_bag.py.  The single-robot plotter is still
best for inspecting one robot's odometry, cmd_vel, IMU, encoders, and diagnostics.
This script reads the common world-frame DMPC topics from a real or simulated
bag and plots the overall team behavior:

  /dmpc/robot1/pose_world, /dmpc/robot2/pose_world
  /dmpc/robot1/u_world,    /dmpc/robot2/u_world
  /dmpc/two_robot/metrics
  /dmpc/two_robot/safety_thresholds
  /dmpc/two_robot/hold_state
  /dmpc/two_robot/obstacle_thresholds
  /dmpc/robot1/obstacle_metrics, /dmpc/robot2/obstacle_metrics
  /robot1/cmd_vel,         /robot2/cmd_vel

It supports sqlite3 and mcap bags and auto-detects the storage backend.
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
from collections import Counter
from typing import Dict, Iterable, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


PoseSeries = Dict[str, List[float]]


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
    raise RuntimeError(
        f"Could not infer bag storage backend for {bag_path}. "
        "Use --storage-id mcap or --storage-id sqlite3."
    )


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
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
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


def empty_series() -> PoseSeries:
    return {"t": [], "x": [], "y": [], "yaw": [], "vx": [], "vy": [], "src": []}


def _append_pose(series: PoseSeries, t: float, x: float, y: float, yaw: float, src: str) -> None:
    series["t"].append(t)
    series["x"].append(float(x))
    series["y"].append(float(y))
    series["yaw"].append(float(yaw))
    series["src"].append(src)


def _append_u(series: Dict[str, List[float]], t: float, ux: float, uy: float) -> None:
    series["t"].append(t)
    series["x"].append(float(ux))
    series["y"].append(float(uy))
    series["norm"].append(float(math.hypot(ux, uy)))


def _append_cmd(series: Dict[str, List[float]], t: float, vx: float, wz: float) -> None:
    series["t"].append(t)
    series["linear_x"].append(float(vx))
    series["angular_z"].append(float(wz))


def _interp(values_t: np.ndarray, values_y: np.ndarray, t_common: np.ndarray) -> np.ndarray:
    if values_t.size == 0:
        return np.full_like(t_common, np.nan, dtype=float)
    if values_t.size == 1:
        return np.full_like(t_common, values_y[0], dtype=float)
    order = np.argsort(values_t)
    return np.interp(t_common, values_t[order], values_y[order])


def make_common_pose_grid(pose_by_ns: Dict[str, PoseSeries], namespaces: List[str]) -> Tuple[np.ndarray, Dict[str, Dict[str, np.ndarray]]]:
    available = [ns for ns in namespaces if len(pose_by_ns[ns]["t"]) > 0]
    if not available:
        raise RuntimeError("No pose_world or odom pose topics were found for the requested namespaces.")

    start = max(float(np.min(pose_by_ns[ns]["t"])) for ns in available)
    end = min(float(np.max(pose_by_ns[ns]["t"])) for ns in available)
    if end <= start:
        # Fall back to the first available robot's times if the overlap is degenerate.
        base_ns = available[0]
        t_common = np.asarray(pose_by_ns[base_ns]["t"], dtype=float)
    else:
        base_ns = max(available, key=lambda ns: len(pose_by_ns[ns]["t"]))
        t0 = np.asarray(pose_by_ns[base_ns]["t"], dtype=float)
        t_common = t0[(t0 >= start) & (t0 <= end)]
        if t_common.size < 2:
            t_common = np.linspace(start, end, 200)

    common: Dict[str, Dict[str, np.ndarray]] = {}
    for ns in available:
        t = np.asarray(pose_by_ns[ns]["t"], dtype=float)
        x = np.asarray(pose_by_ns[ns]["x"], dtype=float)
        y = np.asarray(pose_by_ns[ns]["y"], dtype=float)
        yaw = unwrap(pose_by_ns[ns]["yaw"])
        common[ns] = {
            "x": _interp(t, x, t_common),
            "y": _interp(t, y, t_common),
            "yaw": _interp(t, yaw, t_common),
        }
    return t_common, common


def compute_pair_metrics(t_common: np.ndarray, common: Dict[str, Dict[str, np.ndarray]], namespaces: List[str], d_safe: float, target_distance: float) -> Dict[str, np.ndarray]:
    if len(namespaces) < 2:
        raise RuntimeError("At least two namespaces are required for team metrics.")
    ns1, ns2 = namespaces[0], namespaces[1]
    dx = common[ns1]["x"] - common[ns2]["x"]
    dy = common[ns1]["y"] - common[ns2]["y"]
    dist = np.sqrt(dx * dx + dy * dy)
    return {
        "t": t_common,
        "distance": dist,
        "target": np.full_like(dist, target_distance, dtype=float),
        "margin": dist - d_safe,
        "formation_error": dist - target_distance,
        "abs_formation_error": np.abs(dist - target_distance),
    }


def load_analysis_csv(path: pathlib.Path) -> Dict[str, np.ndarray]:
    if not path.exists():
        return {}
    with path.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}

    def _col(candidates: List[str]) -> Optional[str]:
        headers = rows[0].keys()
        lower_map = {h.lower(): h for h in headers}
        for cand in candidates:
            if cand.lower() in lower_map:
                return lower_map[cand.lower()]
        for h in headers:
            hl = h.lower()
            if any(cand.lower() in hl for cand in candidates):
                return h
        return None

    cols = {
        "t": _col(["t", "time", "time_s", "elapsed", "elapsed_s"]),
        "distance": _col(["distance", "inter_robot_distance", "pair_distance", "d_pair", "d12"]),
        "target": _col(["target_pair_distance", "target_distance", "d_form"]),
        "margin": _col(["safety_margin", "distance_minus_d_safe", "margin"]),
        "formation_error": _col(["formation_error", "distance_error", "distance_minus_target"]),
    }
    out: Dict[str, np.ndarray] = {}
    for key, col in cols.items():
        if not col:
            continue
        vals = []
        for row in rows:
            try:
                vals.append(float(row[col]))
            except Exception:
                pass
        if vals:
            out[key] = np.asarray(vals, dtype=float)
    return out


def savefig(outdir: pathlib.Path, name: str) -> None:
    out = outdir / f"{name}.png"
    plt.tight_layout()
    plt.savefig(out, dpi=160, bbox_inches="tight")
    plt.close()
    print(f"Saved: {out}")


def _as_np(data: List[float]) -> np.ndarray:
    return np.asarray(data, dtype=float)


def _topic_has_samples(series_by_ns: Dict[str, Dict[str, List[float]]], key: str = "t") -> bool:
    return any(len(series.get(key, [])) > 0 for series in series_by_ns.values())


def should_show_obstacle(mode: str, obstacle_data_available: bool) -> bool:
    if mode == "always":
        return True
    if mode == "never":
        return False
    return obstacle_data_available


def draw_obstacle_on_axes(ax, center_x: float, center_y: float, radius: float, margin: float) -> None:
    inflated = max(0.0, float(radius) + float(margin))
    physical = Circle((center_x, center_y), radius=max(0.0, float(radius)), fill=False, linestyle="-", linewidth=1.8, alpha=0.9, label="physical obstacle")
    inflated_patch = Circle((center_x, center_y), radius=inflated, fill=False, linestyle="--", linewidth=1.8, alpha=0.9, label="inflated obstacle")
    ax.add_patch(inflated_patch)
    ax.add_patch(physical)
    ax.plot([center_x], [center_y], marker="x", markersize=8, label="obstacle center")
    ax.text(center_x, center_y, f" obstacle\nr={radius:.2f}, margin={margin:.2f}", ha="left", va="bottom")


def append_extra_summary(outdir: pathlib.Path, hold_data: Dict[str, List[float]], obstacle_metrics_by_ns: Dict[str, Dict[str, List[float]]], obstacle_threshold_data: Dict[str, List[float]], obstacle_config: Dict[str, float]) -> None:
    out = outdir / "team_summary.txt"
    with out.open("a", encoding="utf-8") as f:
        f.write("\nHold-zone metrics:\n")
        if hold_data["t"]:
            active = _as_np(hold_data["active"])
            f.write(f"  hold_state samples: {active.size}\n")
            f.write(f"  hold active samples: {int(np.count_nonzero(active > 0.5))} / {active.size}\n")
            f.write(f"  final hold active: {bool(active[-1] > 0.5)}\n")
            f.write(f"  final selected hold error: {hold_data['selected_error'][-1]:.4f} m\n")
            f.write(f"  final pairwise hold error: {hold_data['pair_error'][-1]:.4f} m\n")
        else:
            f.write("  /dmpc/two_robot/hold_state was not recorded.\n")

        f.write("\nObstacle configuration used for plots:\n")
        for key, val in obstacle_config.items():
            f.write(f"  {key}: {val:.4f}\n")
        f.write(f"  inflated_radius: {obstacle_config['obstacle_radius'] + obstacle_config['obstacle_margin']:.4f}\n")

        if obstacle_threshold_data["t"]:
            f.write("\nObstacle thresholds from last bag sample:\n")
            f.write(f"  d_obs_enter: {obstacle_threshold_data['d_obs_enter'][-1]:.4f} m\n")
            f.write(f"  d_obs_exit: {obstacle_threshold_data['d_obs_exit'][-1]:.4f} m\n")
            f.write(f"  obstacle_warning_radius: {obstacle_threshold_data['warning'][-1]:.4f} m\n")

        f.write("\nObstacle metrics:\n")
        any_obs = False
        for ns, data in obstacle_metrics_by_ns.items():
            if not data["t"]:
                continue
            any_obs = True
            clearance = _as_np(data["clearance"])
            active = _as_np(data["active"])
            f.write(f"  {ns}: samples={clearance.size}, min_clearance={np.nanmin(clearance):.4f} m, final_clearance={clearance[-1]:.4f} m, active_samples={int(np.count_nonzero(active > 0.5))} / {active.size}\n")
        if not any_obs:
            f.write("  obstacle metrics topics were not recorded.\n")
    print(f"Updated: {out}")


def write_summary(outdir: pathlib.Path, bag_path: pathlib.Path, storage_id: str, namespaces: List[str], metrics: Dict[str, np.ndarray], counts: Counter, d_safe: float, d_agent_enter: float, d_agent_exit: float, target_distance: float, used_analysis_csv: Optional[pathlib.Path]) -> None:
    distance = metrics["distance"]
    margin = metrics["margin"]
    abs_err = metrics["abs_formation_error"]
    out = outdir / "team_summary.txt"
    with out.open("w", encoding="utf-8") as f:
        f.write("Yahboom two-robot DMPC team summary\n")
        f.write("====================================\n\n")
        f.write(f"bag_path: {bag_path}\n")
        f.write(f"storage_id: {storage_id}\n")
        f.write(f"namespaces: {', '.join(namespaces)}\n")
        if used_analysis_csv:
            f.write(f"analysis_csv: {used_analysis_csv}\n")
        f.write("\nConfiguration:\n")
        f.write(f"  d_safe: {d_safe:.4f} m\n")
        f.write(f"  d_agent_enter: {d_agent_enter:.4f} m\n")
        f.write(f"  d_agent_exit: {d_agent_exit:.4f} m\n")
        f.write(f"  target pair distance: {target_distance:.4f} m\n")
        f.write("\nTeam metrics:\n")
        f.write(f"  samples: {distance.size}\n")
        f.write(f"  initial distance: {distance[0]:.4f} m\n")
        f.write(f"  final distance: {distance[-1]:.4f} m\n")
        f.write(f"  min distance: {np.nanmin(distance):.4f} m\n")
        f.write(f"  max distance: {np.nanmax(distance):.4f} m\n")
        f.write(f"  minimum safety margin distance-d_safe: {np.nanmin(margin):.4f} m\n")
        f.write(f"  final absolute formation-distance error: {abs_err[-1]:.4f} m\n")
        f.write("\nMessage counts used:\n")
        for topic, count in sorted(counts.items()):
            f.write(f"  {topic}: {count}\n")
    print(f"Saved: {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot team-level Yahboom two-robot DMPC bag data.")
    parser.add_argument("--bag", required=True, help="Path to ROS 2 bag folder.")
    parser.add_argument("--namespaces", nargs="+", default=["robot1", "robot2"], help="Robot namespaces, e.g. robot1 robot2.")
    parser.add_argument("--output-dir", default="", help="Output plot directory. Default: <bag>/team_plots")
    parser.add_argument("--storage-id", default="auto", choices=["auto", "mcap", "sqlite3"], help="rosbag2 storage backend.")
    parser.add_argument("--analysis-csv", default="auto", help="Path to two_robot_dmpc_analysis.csv, 'auto', or 'none'.")
    parser.add_argument("--d-safe", type=float, default=0.65)
    parser.add_argument("--formation-margin", type=float, default=0.15)
    parser.add_argument("--d-agent-enter", type=float, default=0.70)
    parser.add_argument("--d-agent-exit", type=float, default=0.75)
    parser.add_argument("--formation-hold-enter-error", type=float, default=0.04, help="Hold-zone entry error threshold used for plotting.")
    parser.add_argument("--formation-hold-exit-error", type=float, default=0.08, help="Hold-zone exit error threshold used for plotting.")
    parser.add_argument("--show-obstacle", choices=["auto", "always", "never"], default="auto", help="Draw obstacle geometry in XY plots. 'auto' draws it when obstacle metrics or thresholds are present.")
    parser.add_argument("--obstacle-center-x", type=float, default=1.0)
    parser.add_argument("--obstacle-center-y", type=float, default=-0.33)
    parser.add_argument("--obstacle-radius", type=float, default=0.15)
    parser.add_argument("--obstacle-margin", type=float, default=0.10)
    args = parser.parse_args()

    bag_path = expand_path(args.bag)
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag folder does not exist: {bag_path}")
    namespaces = [normalize_namespace(ns) for ns in args.namespaces]
    outdir = expand_path(args.output_dir) if args.output_dir else bag_path / "team_plots"
    outdir.mkdir(parents=True, exist_ok=True)
    storage_id = infer_storage_id(bag_path) if args.storage_id == "auto" else args.storage_id
    target_distance = float(args.d_safe + args.formation_margin)

    reader = open_reader(bag_path, storage_id)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    pose_topics: Dict[str, Optional[str]] = {}
    odom_topics: Dict[str, Optional[str]] = {}
    u_topics: Dict[str, Optional[str]] = {}
    cmd_topics: Dict[str, Optional[str]] = {}
    for ns in namespaces:
        pose_topics[ns] = choose_existing_topic(type_map, [f"/dmpc/{ns}/pose_world"], "geometry_msgs/msg/PoseStamped")
        odom_topics[ns] = choose_existing_topic(type_map, [ns_topic(ns, "odom")], "nav_msgs/msg/Odometry")
        u_topics[ns] = choose_existing_topic(type_map, [f"/dmpc/{ns}/u_world"], "geometry_msgs/msg/Vector3Stamped")
        cmd_topics[ns] = choose_existing_topic(type_map, [ns_topic(ns, "cmd_vel")], "geometry_msgs/msg/Twist")

    metrics_topic = choose_existing_topic(type_map, ["/dmpc/two_robot/metrics"], "geometry_msgs/msg/Vector3Stamped")
    thresholds_topic = choose_existing_topic(type_map, ["/dmpc/two_robot/safety_thresholds"], "geometry_msgs/msg/Vector3Stamped")
    hold_topic = choose_existing_topic(type_map, ["/dmpc/two_robot/hold_state"], "geometry_msgs/msg/Vector3Stamped")
    obstacle_thresholds_topic = choose_existing_topic(type_map, ["/dmpc/two_robot/obstacle_thresholds"], "geometry_msgs/msg/Vector3Stamped")
    obstacle_metrics_topics: Dict[str, Optional[str]] = {}
    for ns in namespaces:
        obstacle_metrics_topics[ns] = choose_existing_topic(type_map, [f"/dmpc/{ns}/obstacle_metrics"], "geometry_msgs/msg/Vector3Stamped")

    selected_topics = set()
    for d in [pose_topics, odom_topics, u_topics, cmd_topics]:
        selected_topics.update(t for t in d.values() if t)
    if metrics_topic:
        selected_topics.add(metrics_topic)
    if thresholds_topic:
        selected_topics.add(thresholds_topic)
    if hold_topic:
        selected_topics.add(hold_topic)
    if obstacle_thresholds_topic:
        selected_topics.add(obstacle_thresholds_topic)
    selected_topics.update(t for t in obstacle_metrics_topics.values() if t)

    if not selected_topics:
        raise RuntimeError("No relevant topics found. Check the bag path and namespaces.")

    msg_type_map = {name: get_message(type_map[name]) for name in selected_topics}

    print("Selected team topics:")
    for ns in namespaces:
        print(f"  {ns}: pose_world={pose_topics[ns] or 'not found'}, odom={odom_topics[ns] or 'not found'}, u_world={u_topics[ns] or 'not found'}, cmd_vel={cmd_topics[ns] or 'not found'}")
    print(f"  metrics: {metrics_topic or 'not found'}")
    print(f"  thresholds: {thresholds_topic or 'not found'}")
    print(f"  hold_state: {hold_topic or 'not found'}")
    print(f"  obstacle_thresholds: {obstacle_thresholds_topic or 'not found'}")
    for ns in namespaces:
        print(f"  {ns}: obstacle_metrics={obstacle_metrics_topics[ns] or 'not found'}")
    print(f"Using storage_id={storage_id}")

    pose_by_ns = {ns: empty_series() for ns in namespaces}
    u_by_ns = {ns: {"t": [], "x": [], "y": [], "norm": []} for ns in namespaces}
    cmd_by_ns = {ns: {"t": [], "linear_x": [], "angular_z": []} for ns in namespaces}
    metrics_topic_data = {"t": [], "distance": [], "target": [], "margin": []}
    threshold_topic_data = {"t": [], "d_safe": [], "d_agent_enter": [], "d_agent_exit": []}
    hold_topic_data = {"t": [], "active": [], "selected_error": [], "pair_error": []}
    obstacle_threshold_data = {"t": [], "d_obs_enter": [], "d_obs_exit": [], "warning": []}
    obstacle_metrics_by_ns = {ns: {"t": [], "distance": [], "clearance": [], "active": []} for ns in namespaces}

    t0: Optional[int] = None
    counts: Counter[str] = Counter()

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in msg_type_map:
            continue
        if t0 is None:
            t0 = timestamp
        t = (timestamp - t0) * 1e-9
        counts[topic] += 1
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
            elif topic == u_topics[ns]:
                _append_u(u_by_ns[ns], t, msg.vector.x, msg.vector.y)
            elif topic == cmd_topics[ns]:
                _append_cmd(cmd_by_ns[ns], t, msg.linear.x, msg.angular.z)
            elif topic == obstacle_metrics_topics[ns]:
                obstacle_metrics_by_ns[ns]["t"].append(t)
                obstacle_metrics_by_ns[ns]["distance"].append(float(msg.vector.x))
                obstacle_metrics_by_ns[ns]["clearance"].append(float(msg.vector.y))
                obstacle_metrics_by_ns[ns]["active"].append(float(msg.vector.z))

        if topic == metrics_topic:
            metrics_topic_data["t"].append(t)
            metrics_topic_data["distance"].append(float(msg.vector.x))
            metrics_topic_data["target"].append(float(msg.vector.y))
            metrics_topic_data["margin"].append(float(msg.vector.z))
        elif topic == thresholds_topic:
            threshold_topic_data["t"].append(t)
            threshold_topic_data["d_safe"].append(float(msg.vector.x))
            threshold_topic_data["d_agent_enter"].append(float(msg.vector.y))
            threshold_topic_data["d_agent_exit"].append(float(msg.vector.z))
        elif topic == hold_topic:
            hold_topic_data["t"].append(t)
            hold_topic_data["active"].append(float(msg.vector.x))
            hold_topic_data["selected_error"].append(float(msg.vector.y))
            hold_topic_data["pair_error"].append(float(msg.vector.z))
        elif topic == obstacle_thresholds_topic:
            obstacle_threshold_data["t"].append(t)
            obstacle_threshold_data["d_obs_enter"].append(float(msg.vector.x))
            obstacle_threshold_data["d_obs_exit"].append(float(msg.vector.y))
            obstacle_threshold_data["warning"].append(float(msg.vector.z))

    if threshold_topic_data["d_safe"]:
        args.d_safe = float(threshold_topic_data["d_safe"][-1])
        args.d_agent_enter = float(threshold_topic_data["d_agent_enter"][-1])
        args.d_agent_exit = float(threshold_topic_data["d_agent_exit"][-1])
        target_distance = float(args.d_safe + args.formation_margin)

    t_common, common = make_common_pose_grid(pose_by_ns, namespaces)
    computed_metrics = compute_pair_metrics(t_common, common, namespaces, args.d_safe, target_distance)

    analysis_csv_path: Optional[pathlib.Path] = None
    analysis_data: Dict[str, np.ndarray] = {}
    if args.analysis_csv != "none":
        analysis_csv_path = bag_path / "two_robot_dmpc_analysis.csv" if args.analysis_csv == "auto" else expand_path(args.analysis_csv)
        analysis_data = load_analysis_csv(analysis_csv_path)
        if not analysis_data:
            analysis_csv_path = None

    # Prefer the bag metrics topic for distance if available; otherwise use pose_world-derived metrics.
    if metrics_topic_data["distance"]:
        metrics = {
            "t": np.asarray(metrics_topic_data["t"], dtype=float),
            "distance": np.asarray(metrics_topic_data["distance"], dtype=float),
            "target": np.asarray(metrics_topic_data["target"], dtype=float),
            "margin": np.asarray(metrics_topic_data["margin"], dtype=float),
        }
        metrics["formation_error"] = metrics["distance"] - metrics["target"]
        metrics["abs_formation_error"] = np.abs(metrics["formation_error"])
    else:
        metrics = computed_metrics

    # 1) Team trajectories.
    plt.figure(figsize=(7.5, 7.0))
    for ns in namespaces:
        plt.plot(common[ns]["x"], common[ns]["y"], label=f"{ns} world path")
        plt.plot(common[ns]["x"][0], common[ns]["y"][0], marker="s", markersize=7)
        plt.plot(common[ns]["x"][-1], common[ns]["y"][-1], marker="o", markersize=7)
        plt.text(common[ns]["x"][0], common[ns]["y"][0], f" {ns} start")
        plt.text(common[ns]["x"][-1], common[ns]["y"][-1], f" {ns} end")
    if len(namespaces) >= 2:
        ns1, ns2 = namespaces[0], namespaces[1]
        plt.plot([common[ns1]["x"][-1], common[ns2]["x"][-1]], [common[ns1]["y"][-1], common[ns2]["y"][-1]], "k--", alpha=0.6, label="final pair distance")
    obstacle_data_available = bool(obstacle_threshold_data["t"] or _topic_has_samples(obstacle_metrics_by_ns))
    if should_show_obstacle(args.show_obstacle, obstacle_data_available):
        draw_obstacle_on_axes(plt.gca(), args.obstacle_center_x, args.obstacle_center_y, args.obstacle_radius, args.obstacle_margin)
    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("world x [m]")
    plt.ylabel("world y [m]")
    plt.title("Two-robot world-frame trajectories")
    plt.legend()
    savefig(outdir, "team_world_trajectories")

    # 2) World pose time series.
    plt.figure(figsize=(10, 6))
    for ns in namespaces:
        plt.plot(t_common, common[ns]["x"], label=f"{ns} x")
        plt.plot(t_common, common[ns]["y"], label=f"{ns} y")
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("position [m]")
    plt.title("World-frame positions over time")
    plt.legend(ncol=2)
    savefig(outdir, "team_world_position_timeseries")

    # 3) Inter-robot distance.
    plt.figure(figsize=(10, 5))
    plt.plot(metrics["t"], metrics["distance"], label="inter-robot distance")
    plt.axhline(args.d_safe, linestyle="--", label=f"d_safe={args.d_safe:.2f} m")
    plt.axhline(args.d_agent_enter, linestyle=":", label=f"d_agent_enter={args.d_agent_enter:.2f} m")
    plt.axhline(args.d_agent_exit, linestyle=":", label=f"d_agent_exit={args.d_agent_exit:.2f} m")
    plt.axhline(target_distance, linestyle="-.", label=f"target={target_distance:.2f} m")
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("distance [m]")
    plt.title("Inter-robot distance and safety/formation thresholds")
    plt.legend()
    savefig(outdir, "team_inter_robot_distance")

    # 4) Safety and formation error.
    plt.figure(figsize=(10, 5))
    plt.plot(metrics["t"], metrics["margin"], label="safety margin = distance - d_safe")
    plt.axhline(0.0, linestyle="--", label="safety boundary")
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("margin [m]")
    plt.title("Safety margin over time")
    plt.legend()
    savefig(outdir, "team_safety_margin")

    plt.figure(figsize=(10, 5))
    plt.plot(metrics["t"], metrics["formation_error"], label="distance - target")
    plt.plot(metrics["t"], metrics["abs_formation_error"], label="absolute formation-distance error")
    plt.axhline(0.0, linestyle="--", label="target reached")
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("error [m]")
    plt.title("Two-robot formation-distance error")
    plt.legend()
    savefig(outdir, "team_formation_distance_error")

    # 5) World commands.
    plt.figure(figsize=(10, 6))
    found_u = False
    for ns in namespaces:
        if u_by_ns[ns]["t"]:
            found_u = True
            plt.plot(u_by_ns[ns]["t"], u_by_ns[ns]["x"], label=f"{ns} u_world.x")
            plt.plot(u_by_ns[ns]["t"], u_by_ns[ns]["y"], label=f"{ns} u_world.y")
            plt.plot(u_by_ns[ns]["t"], u_by_ns[ns]["norm"], linestyle="--", label=f"{ns} |u_world|")
    if found_u:
        plt.grid(True)
        plt.xlabel("time [s]")
        plt.ylabel("world command [m/s-like]")
        plt.title("DMPC world-frame commands")
        plt.legend(ncol=2)
        savefig(outdir, "team_u_world_timeseries")
    else:
        plt.close()

    # 6) cmd_vel.
    plt.figure(figsize=(10, 6))
    found_cmd = False
    for ns in namespaces:
        if cmd_by_ns[ns]["t"]:
            found_cmd = True
            plt.plot(cmd_by_ns[ns]["t"], cmd_by_ns[ns]["linear_x"], label=f"{ns} cmd linear.x")
            plt.plot(cmd_by_ns[ns]["t"], cmd_by_ns[ns]["angular_z"], label=f"{ns} cmd angular.z")
    if found_cmd:
        plt.grid(True)
        plt.xlabel("time [s]")
        plt.ylabel("cmd_vel")
        plt.title("Published robot velocity commands")
        plt.legend(ncol=2)
        savefig(outdir, "team_cmd_vel_timeseries")
    else:
        plt.close()

    # 7) Optional comparison to analysis CSV.
    if analysis_data and "distance" in analysis_data:
        plt.figure(figsize=(10, 5))
        if "t" in analysis_data and analysis_data["t"].size == analysis_data["distance"].size:
            plt.plot(analysis_data["t"], analysis_data["distance"], label="analysis CSV distance")
        else:
            plt.plot(analysis_data["distance"], label="analysis CSV distance")
        plt.plot(metrics["t"], metrics["distance"], linestyle="--", label="bag topic/computed distance")
        plt.axhline(args.d_safe, linestyle="--", label="d_safe")
        plt.axhline(target_distance, linestyle="-.", label="target")
        plt.grid(True)
        plt.xlabel("time [s] or sample")
        plt.ylabel("distance [m]")
        plt.title("Distance from analysis CSV versus bag-derived distance")
        plt.legend()
        savefig(outdir, "team_analysis_csv_distance_comparison")

    # 8) Hold-zone state.
    if hold_topic_data["t"]:
        plt.figure(figsize=(10, 5))
        t_hold = _as_np(hold_topic_data["t"])
        plt.step(t_hold, _as_np(hold_topic_data["active"]), where="post", label="hold active (1=yes)")
        plt.plot(t_hold, _as_np(hold_topic_data["selected_error"]), label="selected hold error [m]")
        plt.plot(t_hold, _as_np(hold_topic_data["pair_error"]), label="pairwise formation error [m]")
        plt.axhline(args.formation_hold_enter_error, linestyle=":", label=f"hold enter={args.formation_hold_enter_error:.2f} m")
        plt.axhline(args.formation_hold_exit_error, linestyle="--", label=f"hold exit={args.formation_hold_exit_error:.2f} m")
        plt.grid(True)
        plt.xlabel("time [s]")
        plt.ylabel("hold state / error")
        plt.title("Formation hold-zone state and errors")
        plt.legend(ncol=2)
        savefig(outdir, "team_hold_state")

    # 9) Obstacle clearance and obstacle-active flags.
    if _topic_has_samples(obstacle_metrics_by_ns):
        d_obs_enter = obstacle_threshold_data["d_obs_enter"][-1] if obstacle_threshold_data["d_obs_enter"] else np.nan
        d_obs_exit = obstacle_threshold_data["d_obs_exit"][-1] if obstacle_threshold_data["d_obs_exit"] else np.nan
        warning = obstacle_threshold_data["warning"][-1] if obstacle_threshold_data["warning"] else np.nan

        plt.figure(figsize=(10, 5))
        for ns in namespaces:
            if obstacle_metrics_by_ns[ns]["t"]:
                plt.plot(obstacle_metrics_by_ns[ns]["t"], obstacle_metrics_by_ns[ns]["clearance"], label=f"{ns} inflated-obstacle clearance")
        plt.axhline(0.0, linestyle="--", label="inflated obstacle boundary")
        if not np.isnan(d_obs_enter):
            plt.axhline(d_obs_enter, linestyle=":", label=f"d_obs_enter={d_obs_enter:.2f} m")
        if not np.isnan(d_obs_exit):
            plt.axhline(d_obs_exit, linestyle="-.", label=f"d_obs_exit={d_obs_exit:.2f} m")
        if not np.isnan(warning):
            plt.axhline(warning, linestyle=":", label=f"warning={warning:.2f} m")
        plt.grid(True)
        plt.xlabel("time [s]")
        plt.ylabel("clearance [m]")
        plt.title("Obstacle clearance from inflated obstacle boundary")
        plt.legend(ncol=2)
        savefig(outdir, "team_obstacle_clearance")

        plt.figure(figsize=(10, 4))
        for ns in namespaces:
            if obstacle_metrics_by_ns[ns]["t"]:
                plt.step(obstacle_metrics_by_ns[ns]["t"], obstacle_metrics_by_ns[ns]["active"], where="post", label=f"{ns} obstacle active")
        plt.ylim(-0.1, 1.1)
        plt.grid(True)
        plt.xlabel("time [s]")
        plt.ylabel("active flag")
        plt.title("Obstacle avoidance active flags")
        plt.legend(ncol=2)
        savefig(outdir, "team_obstacle_active")

    write_summary(outdir, bag_path, storage_id, namespaces, metrics, counts, args.d_safe, args.d_agent_enter, args.d_agent_exit, target_distance, analysis_csv_path)
    append_extra_summary(
        outdir,
        hold_topic_data,
        obstacle_metrics_by_ns,
        obstacle_threshold_data,
        {
            "obstacle_center_x": args.obstacle_center_x,
            "obstacle_center_y": args.obstacle_center_y,
            "obstacle_radius": args.obstacle_radius,
            "obstacle_margin": args.obstacle_margin,
        },
    )
    print(f"\nDone. Team plots and summary were written to:\n{outdir}")


if __name__ == "__main__":
    main()
