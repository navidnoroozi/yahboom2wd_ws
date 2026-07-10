from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any, Dict, Optional


def _load_rosbag_modules():
    import rosbag2_py  # type: ignore
    from rclpy.serialization import deserialize_message  # type: ignore
    from rosidl_runtime_py.utilities import get_message  # type: ignore
    return rosbag2_py, deserialize_message, get_message


def _pose_xy(msg: Any) -> tuple[float, float]:
    return float(msg.pose.position.x), float(msg.pose.position.y)


def _vec_xyz(msg: Any) -> tuple[float, float, float]:
    return float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze a two-robot DMPC ROS 2 bag.")
    parser.add_argument("--bag", required=True, help="Path to the rosbag folder.")
    parser.add_argument("--storage", default="sqlite3", help="rosbag storage id, e.g. sqlite3 or mcap.")
    parser.add_argument("--d-safe", type=float, default=0.65)
    parser.add_argument("--formation-margin", type=float, default=0.15)
    parser.add_argument("--output-prefix", default="", help="Output prefix. Defaults to <bag>/two_robot_dmpc_analysis")
    args = parser.parse_args()

    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        raise SystemExit(f"Bag path does not exist: {bag_path}")

    rosbag2_py, deserialize_message, get_message = _load_rosbag_modules()

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=args.storage)
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {topic: get_message(type_name) for topic, type_name in topic_types.items()}

    topics_of_interest = {
        "/dmpc/robot1/pose_world",
        "/dmpc/robot2/pose_world",
        "/dmpc/robot1/u_world",
        "/dmpc/robot2/u_world",
        "/dmpc/two_robot/metrics",
        "/robot1/cmd_vel",
        "/robot2/cmd_vel",
    }

    latest_pose: Dict[str, tuple[float, float]] = {}
    samples = []
    u_samples = []
    cmd_nonzero_count = {"robot1": 0, "robot2": 0}
    cmd_count = {"robot1": 0, "robot2": 0}
    metrics_samples = []

    while reader.has_next():
        topic, data, t_nsec = reader.read_next()
        if topic not in topics_of_interest:
            continue
        msg = deserialize_message(data, msg_types[topic])
        t = float(t_nsec) * 1e-9

        if topic == "/dmpc/robot1/pose_world":
            latest_pose["robot1"] = _pose_xy(msg)
        elif topic == "/dmpc/robot2/pose_world":
            latest_pose["robot2"] = _pose_xy(msg)
        elif topic == "/dmpc/robot1/u_world":
            ux, uy, _ = _vec_xyz(msg)
            u_samples.append((t, "robot1", ux, uy, math.hypot(ux, uy)))
        elif topic == "/dmpc/robot2/u_world":
            ux, uy, _ = _vec_xyz(msg)
            u_samples.append((t, "robot2", ux, uy, math.hypot(ux, uy)))
        elif topic == "/dmpc/two_robot/metrics":
            dist, target, margin = _vec_xyz(msg)
            metrics_samples.append((t, dist, target, margin))
        elif topic == "/robot1/cmd_vel":
            cmd_count["robot1"] += 1
            if abs(float(msg.linear.x)) > 1e-6 or abs(float(msg.angular.z)) > 1e-6:
                cmd_nonzero_count["robot1"] += 1
        elif topic == "/robot2/cmd_vel":
            cmd_count["robot2"] += 1
            if abs(float(msg.linear.x)) > 1e-6 or abs(float(msg.angular.z)) > 1e-6:
                cmd_nonzero_count["robot2"] += 1

        if "robot1" in latest_pose and "robot2" in latest_pose:
            x1, y1 = latest_pose["robot1"]
            x2, y2 = latest_pose["robot2"]
            d = math.hypot(x1 - x2, y1 - y2)
            samples.append((t, x1, y1, x2, y2, d, d - float(args.d_safe)))

    if not samples:
        raise SystemExit("No paired /dmpc/robot*/pose_world samples found in the bag.")

    prefix = Path(args.output_prefix) if args.output_prefix else bag_path / "two_robot_dmpc_analysis"
    csv_path = prefix.with_suffix(".csv")
    txt_path = prefix.with_suffix(".txt")

    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "robot1_x", "robot1_y", "robot2_x", "robot2_y", "distance", "distance_minus_d_safe"])
        writer.writerows(samples)

    distances = [row[5] for row in samples]
    final_distance = distances[-1]
    min_distance = min(distances)
    max_distance = max(distances)
    initial_distance = distances[0]
    target_distance = float(args.d_safe) + float(args.formation_margin)
    final_error = abs(final_distance - target_distance)

    with txt_path.open("w") as f:
        f.write("Two-robot DMPC bag analysis\n")
        f.write(f"bag: {bag_path}\n")
        f.write(f"samples: {len(samples)}\n")
        f.write(f"d_safe: {args.d_safe:.4f} m\n")
        f.write(f"formation_margin: {args.formation_margin:.4f} m\n")
        f.write(f"target pair distance for n=2: {target_distance:.4f} m\n")
        f.write(f"initial distance: {initial_distance:.4f} m\n")
        f.write(f"final distance: {final_distance:.4f} m\n")
        f.write(f"final absolute formation-distance error: {final_error:.4f} m\n")
        f.write(f"min distance: {min_distance:.4f} m\n")
        f.write(f"max distance: {max_distance:.4f} m\n")
        f.write(f"minimum safety margin distance-d_safe: {min_distance - float(args.d_safe):.4f} m\n")
        for robot in ["robot1", "robot2"]:
            f.write(f"{robot} nonzero cmd_vel samples: {cmd_nonzero_count[robot]} / {cmd_count[robot]}\n")
        if metrics_samples:
            f.write(f"/dmpc/two_robot/metrics samples: {len(metrics_samples)}\n")

    print(f"Wrote {csv_path}")
    print(f"Wrote {txt_path}")
    print(txt_path.read_text())


if __name__ == "__main__":
    main()
