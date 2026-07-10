#!/usr/bin/env python3
"""Create a team-level animation for two-robot Yahboom DMPC ROS 2 bags.

The animation uses common world-frame poses from /dmpc/<robot>/pose_world when
available, and falls back to /<robot>/odom if pose_world is not recorded.
It visualizes the team trajectory, headings, safety buffers, inter-robot line,
and distance/formation metrics over time.
"""

from __future__ import annotations

import argparse
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
    return {"t": [], "x": [], "y": [], "yaw": [], "src": []}


def _append_pose(series: PoseSeries, t: float, x: float, y: float, yaw: float, src: str) -> None:
    series["t"].append(t)
    series["x"].append(float(x))
    series["y"].append(float(y))
    series["yaw"].append(float(yaw))
    series["src"].append(src)


def _interp(values_t: np.ndarray, values_y: np.ndarray, t_common: np.ndarray) -> np.ndarray:
    if values_t.size == 0:
        return np.full_like(t_common, np.nan, dtype=float)
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
        common[ns] = {
            "x": _interp(t, x, t_common),
            "y": _interp(t, y, t_common),
            "yaw": _interp(t, yaw, t_common),
        }
    return t_common, common


def load_poses_from_bag(bag_path: pathlib.Path, storage_id: str, namespaces: List[str]) -> Tuple[np.ndarray, Dict[str, Dict[str, np.ndarray]]]:
    reader = open_reader(bag_path, storage_id)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    pose_topics: Dict[str, Optional[str]] = {}
    odom_topics: Dict[str, Optional[str]] = {}
    for ns in namespaces:
        pose_topics[ns] = choose_existing_topic(type_map, [f"/dmpc/{ns}/pose_world"], "geometry_msgs/msg/PoseStamped")
        odom_topics[ns] = choose_existing_topic(type_map, [ns_topic(ns, "odom")], "nav_msgs/msg/Odometry")

    selected_topics = set()
    selected_topics.update(t for t in pose_topics.values() if t)
    selected_topics.update(t for ns, t in odom_topics.items() if t and pose_topics[ns] is None)
    if not selected_topics:
        raise RuntimeError("No pose_world or odom topics found. Check --bag and --namespaces.")

    msg_type_map = {name: get_message(type_map[name]) for name in selected_topics}
    pose_by_ns = {ns: empty_series() for ns in namespaces}

    print("Selected animation topics:")
    for ns in namespaces:
        print(f"  {ns}: pose_world={pose_topics[ns] or 'not found'}, odom={odom_topics[ns] or 'not found'}")
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

    return make_common_pose_grid(pose_by_ns, namespaces, sample_dt=0.1)


def main() -> None:
    parser = argparse.ArgumentParser(description="Animate team-level Yahboom two-robot DMPC bag data.")
    parser.add_argument("--bag", required=True, help="Path to ROS 2 bag folder.")
    parser.add_argument("--namespaces", nargs="+", default=["robot1", "robot2"], help="Robot namespaces, e.g. robot1 robot2.")
    parser.add_argument("--storage-id", default="auto", choices=["auto", "mcap", "sqlite3"], help="rosbag2 storage backend.")
    parser.add_argument("--output-dir", default="", help="Output directory. Default: <bag>/team_animation")
    parser.add_argument("--gif-path", default="", help="GIF output path. Default: <output-dir>/team_motion.gif")
    parser.add_argument("--fps", type=float, default=8.0, help="GIF frames per second.")
    parser.add_argument("--step", type=int, default=1, help="Use every Nth interpolated frame.")
    parser.add_argument("--max-frames", type=int, default=600, help="Cap rendered frames to keep GIF size reasonable. Use 0 for no cap.")
    parser.add_argument("--d-safe", type=float, default=0.65)
    parser.add_argument("--formation-margin", type=float, default=0.15)
    args = parser.parse_args()

    bag_path = expand_path(args.bag)
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag folder does not exist: {bag_path}")
    namespaces = [normalize_namespace(ns) for ns in args.namespaces]
    storage_id = infer_storage_id(bag_path) if args.storage_id == "auto" else args.storage_id
    outdir = expand_path(args.output_dir) if args.output_dir else bag_path / "team_animation"
    outdir.mkdir(parents=True, exist_ok=True)
    gif_path = expand_path(args.gif_path) if args.gif_path else outdir / "team_motion.gif"

    t_common, common = load_poses_from_bag(bag_path, storage_id, namespaces)
    if len(namespaces) < 2:
        raise RuntimeError("At least two namespaces are required for team animation.")

    # Downsample for GIF size.
    frame_indices = np.arange(0, len(t_common), max(1, int(args.step)), dtype=int)
    if args.max_frames > 0 and frame_indices.size > args.max_frames:
        frame_indices = np.linspace(0, len(t_common) - 1, args.max_frames).astype(int)

    # Plot bounds.
    xs = np.concatenate([common[ns]["x"] for ns in namespaces])
    ys = np.concatenate([common[ns]["y"] for ns in namespaces])
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
        safety_circles[ns] = Circle((0.0, 0.0), safe_radius_visual, fill=False, linestyle="--", linewidth=1.0, alpha=0.6, color=colors[ns])
        ax.add_patch(safety_circles[ns])
        heading_quivers[ns] = ax.quiver([], [], [], [], angles="xy", scale_units="xy", scale=1.0, color=colors[ns], width=0.006)

    (pair_line,) = ax.plot([], [], "k--", alpha=0.65, label="robot pair")
    text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=10,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.75"),
    )
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
        ax.set_title(f"Two-robot DMPC team motion | t={t_common[ti]:.1f} s")
        text.set_text(
            f"distance: {dist:.3f} m\n"
            f"d_safe: {args.d_safe:.3f} m\n"
            f"safety margin: {safety_margin:.3f} m\n"
            f"target distance: {target:.3f} m\n"
            f"formation error: {formation_error:+.3f} m"
        )
        return [*trails.values(), *points.values(), *labels.values(), *safety_circles.values(), *heading_quivers.values(), pair_line, text]

    ani = FuncAnimation(fig, _frame, frames=len(frame_indices), interval=1000.0 / max(args.fps, 0.1), blit=False)
    print(f"Rendering {len(frame_indices)} frames to {gif_path} ...")
    ani.save(str(gif_path), writer=PillowWriter(fps=max(args.fps, 0.1)))
    plt.close(fig)
    print(f"Saved: {gif_path}")


if __name__ == "__main__":
    main()
