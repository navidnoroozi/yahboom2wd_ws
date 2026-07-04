#!/usr/bin/env python3
"""Plot ROS 2 Humble bag data from the Yahboom 2WD interface package.

Example:
  python3 plot_yahboom_bag.py \
    --bag ~/yahboom2wd_ws/bags/robot1_forward_0p1_10s \
    --namespace robot1

The script supports MCAP and SQLite3 bags. It auto-detects the storage backend
from the bag folder contents unless --storage-id is provided.
"""

from __future__ import annotations

import argparse
import math
import pathlib
from collections import Counter
from typing import Dict, Iterable, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


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
    relative_name = relative_name.lstrip("/")
    if namespace:
        return f"/{namespace}/{relative_name}"
    return f"/{relative_name}"


def byte_or_int_to_int(value) -> int:
    if isinstance(value, (bytes, bytearray)):
        return int(value[0])
    return int(value)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap_angle_series(values: List[float]) -> List[float]:
    if not values:
        return []
    out = [values[0]]
    offset = 0.0
    prev = values[0]
    for value in values[1:]:
        delta = value - prev
        if delta > math.pi:
            offset -= 2.0 * math.pi
        elif delta < -math.pi:
            offset += 2.0 * math.pi
        out.append(value + offset)
        prev = value
    return out


def path_length(xs: List[float], ys: List[float]) -> float:
    if len(xs) < 2:
        return 0.0
    return sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs)))


def save_plot(output_dir: pathlib.Path, filename: str, title: str, xlabel: str, ylabel: str, legend: bool = True) -> None:
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    if legend:
        plt.legend()
    plt.tight_layout()
    out = output_dir / filename
    plt.savefig(out, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {out}")


def choose_existing_topic(type_map: Dict[str, str], candidates: Iterable[str], expected_type: Optional[str] = None) -> Optional[str]:
    for name in candidates:
        if name in type_map and (expected_type is None or type_map[name] == expected_type):
            return name
    # Fallback: if namespaced topic changed, use suffix matching with correct type.
    for candidate in candidates:
        suffix = candidate if candidate.startswith("/") else f"/{candidate}"
        for name, typ in type_map.items():
            if name.endswith(suffix) and (expected_type is None or typ == expected_type):
                return name
    return None


def open_reader(bag_path: pathlib.Path, storage_id: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot Yahboom 2WD ROS 2 bag data.")
    parser.add_argument("--bag", required=True, help="Path to ROS 2 bag folder.")
    parser.add_argument("--namespace", default="robot1", help="Robot namespace, e.g. robot1. Use empty string for no namespace.")
    parser.add_argument("--output-dir", default="", help="Output plot directory. Default: <bag>/plots")
    parser.add_argument("--storage-id", default="auto", choices=["auto", "mcap", "sqlite3"], help="rosbag2 storage backend.")
    args = parser.parse_args()

    bag_path = expand_path(args.bag)
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag folder does not exist: {bag_path}")

    output_dir = expand_path(args.output_dir) if args.output_dir else bag_path / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    namespace = normalize_namespace(args.namespace)
    storage_id = infer_storage_id(bag_path) if args.storage_id == "auto" else args.storage_id

    reader = open_reader(bag_path, storage_id)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    print("Available topics in bag:")
    for name, typ in sorted(type_map.items()):
        print(f"  {name}: {typ}")
    print(f"\nUsing storage_id={storage_id}")

    topics = {
        "cmd_vel": choose_existing_topic(type_map, [ns_topic(namespace, "cmd_vel"), "/cmd_vel"], "geometry_msgs/msg/Twist"),
        "odom": choose_existing_topic(type_map, [ns_topic(namespace, "odom"), "/odom"], "nav_msgs/msg/Odometry"),
        "imu": choose_existing_topic(type_map, [ns_topic(namespace, "imu/data"), "/imu/data"], "sensor_msgs/msg/Imu"),
        "battery": choose_existing_topic(type_map, [ns_topic(namespace, "battery_state"), "/battery_state"], "sensor_msgs/msg/BatteryState"),
        "encoders": choose_existing_topic(type_map, [ns_topic(namespace, "encoder_ticks"), "/encoder_ticks"], "std_msgs/msg/Int32MultiArray"),
        "diagnostics": choose_existing_topic(type_map, [ns_topic(namespace, "diagnostics"), "/diagnostics"], "diagnostic_msgs/msg/DiagnosticArray"),
        "rosout": choose_existing_topic(type_map, ["/rosout"], "rcl_interfaces/msg/Log"),
    }

    print("\nSelected topics:")
    for key, value in topics.items():
        print(f"  {key:12s}: {value if value else 'not found'}")

    selected_topics = {name for name in topics.values() if name}
    msg_type_map = {name: get_message(type_map[name]) for name in selected_topics}

    # Containers.
    t0: Optional[int] = None
    counts: Counter[str] = Counter()

    cmd_t: List[float] = []
    cmd_lin_x: List[float] = []
    cmd_ang_z: List[float] = []

    odom_t: List[float] = []
    odom_x: List[float] = []
    odom_y: List[float] = []
    odom_yaw: List[float] = []
    odom_vx: List[float] = []
    odom_vy: List[float] = []
    odom_wz: List[float] = []
    odom_speed: List[float] = []

    imu_t: List[float] = []
    imu_ax: List[float] = []
    imu_ay: List[float] = []
    imu_az: List[float] = []
    imu_gx: List[float] = []
    imu_gy: List[float] = []
    imu_gz: List[float] = []

    batt_t: List[float] = []
    batt_v: List[float] = []

    enc_t: List[float] = []
    enc_m1: List[int] = []
    enc_m2: List[int] = []
    enc_m3: List[int] = []
    enc_m4: List[int] = []

    diag_t: List[float] = []
    diag_level: List[int] = []
    diag_name: List[str] = []
    diag_message: List[str] = []

    rosout_t: List[float] = []
    rosout_level: List[int] = []
    rosout_name: List[str] = []
    rosout_msg: List[str] = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in msg_type_map:
            continue
        if t0 is None:
            t0 = timestamp
        t = (timestamp - t0) * 1e-9
        counts[topic] += 1
        msg = deserialize_message(data, msg_type_map[topic])

        if topic == topics["cmd_vel"]:
            cmd_t.append(t)
            cmd_lin_x.append(float(msg.linear.x))
            cmd_ang_z.append(float(msg.angular.z))

        elif topic == topics["odom"]:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            vx = float(msg.twist.twist.linear.x)
            vy = float(msg.twist.twist.linear.y)
            wz = float(msg.twist.twist.angular.z)
            odom_t.append(t)
            odom_x.append(x)
            odom_y.append(y)
            odom_yaw.append(yaw)
            odom_vx.append(vx)
            odom_vy.append(vy)
            odom_wz.append(wz)
            odom_speed.append(math.hypot(vx, vy))

        elif topic == topics["imu"]:
            imu_t.append(t)
            imu_ax.append(float(msg.linear_acceleration.x))
            imu_ay.append(float(msg.linear_acceleration.y))
            imu_az.append(float(msg.linear_acceleration.z))
            imu_gx.append(float(msg.angular_velocity.x))
            imu_gy.append(float(msg.angular_velocity.y))
            imu_gz.append(float(msg.angular_velocity.z))

        elif topic == topics["battery"]:
            batt_t.append(t)
            batt_v.append(float(msg.voltage))

        elif topic == topics["encoders"]:
            values = list(msg.data)
            if len(values) >= 4:
                enc_t.append(t)
                enc_m1.append(int(values[0]))
                enc_m2.append(int(values[1]))
                enc_m3.append(int(values[2]))
                enc_m4.append(int(values[3]))

        elif topic == topics["diagnostics"]:
            for status in msg.status:
                diag_t.append(t)
                diag_level.append(byte_or_int_to_int(status.level))
                diag_name.append(str(status.name))
                diag_message.append(str(status.message))

        elif topic == topics["rosout"]:
            rosout_t.append(t)
            rosout_level.append(byte_or_int_to_int(msg.level))
            rosout_name.append(str(msg.name))
            rosout_msg.append(str(msg.msg))

    # Plots.
    if cmd_t:
        plt.figure()
        plt.plot(cmd_t, cmd_lin_x, label="cmd linear.x [m/s]")
        plt.plot(cmd_t, cmd_ang_z, label="cmd angular.z [rad/s]")
        save_plot(output_dir, "cmd_vel_timeseries.png", "cmd_vel over time", "time [s]", "command")

    if odom_t:
        yaw_unwrapped = unwrap_angle_series(odom_yaw)

        plt.figure()
        plt.plot(odom_x, odom_y, label="odom trajectory")
        plt.axis("equal")
        save_plot(output_dir, "odom_xy.png", "odom trajectory", "x [m]", "y [m]")

        plt.figure()
        plt.plot(odom_t, odom_x, label="x [m]")
        plt.plot(odom_t, odom_y, label="y [m]")
        plt.plot(odom_t, yaw_unwrapped, label="yaw unwrapped [rad]")
        save_plot(output_dir, "odom_pose_timeseries.png", "odom pose over time", "time [s]", "pose")

        plt.figure()
        plt.plot(odom_t, odom_vx, label="vx [m/s]")
        plt.plot(odom_t, odom_vy, label="vy [m/s]")
        plt.plot(odom_t, odom_wz, label="wz [rad/s]")
        plt.plot(odom_t, odom_speed, label="speed [m/s]")
        save_plot(output_dir, "odom_twist_timeseries.png", "odom twist over time", "time [s]", "twist")

    if imu_t:
        plt.figure()
        plt.plot(imu_t, imu_ax, label="accel.x [m/s²]")
        plt.plot(imu_t, imu_ay, label="accel.y [m/s²]")
        plt.plot(imu_t, imu_az, label="accel.z [m/s²]")
        save_plot(output_dir, "imu_linear_acceleration.png", "IMU linear acceleration", "time [s]", "acceleration [m/s²]")

        plt.figure()
        plt.plot(imu_t, imu_gx, label="gyro.x [rad/s]")
        plt.plot(imu_t, imu_gy, label="gyro.y [rad/s]")
        plt.plot(imu_t, imu_gz, label="gyro.z [rad/s]")
        save_plot(output_dir, "imu_angular_velocity.png", "IMU angular velocity", "time [s]", "angular velocity [rad/s]")

    if batt_t:
        plt.figure()
        plt.plot(batt_t, batt_v, label="battery voltage [V]")
        save_plot(output_dir, "battery_voltage.png", "battery voltage", "time [s]", "voltage [V]")

    if enc_t:
        plt.figure()
        plt.plot(enc_t, enc_m1, label="M1 ticks")
        plt.plot(enc_t, enc_m2, label="M2 ticks")
        plt.plot(enc_t, enc_m3, label="M3 ticks")
        plt.plot(enc_t, enc_m4, label="M4 ticks")
        save_plot(output_dir, "encoder_ticks.png", "encoder ticks", "time [s]", "ticks")

        plt.figure()
        plt.plot(enc_t, [v - enc_m1[0] for v in enc_m1], label="M1 delta")
        plt.plot(enc_t, [v - enc_m2[0] for v in enc_m2], label="M2 delta")
        plt.plot(enc_t, [v - enc_m3[0] for v in enc_m3], label="M3 delta")
        plt.plot(enc_t, [v - enc_m4[0] for v in enc_m4], label="M4 delta")
        save_plot(output_dir, "encoder_delta_ticks.png", "encoder delta ticks", "time [s]", "delta ticks")

    if diag_t:
        plt.figure()
        plt.plot(diag_t, diag_level, "o", label="diagnostic level")
        plt.yticks([0, 1, 2, 3], ["OK", "WARN", "ERROR", "STALE"])
        save_plot(output_dir, "diagnostics_levels.png", "diagnostics levels", "time [s]", "level")

        diag_counts = Counter(diag_name)
        plt.figure()
        plt.bar(list(diag_counts.keys()), list(diag_counts.values()), label="count")
        plt.xticks(rotation=45, ha="right")
        save_plot(output_dir, "diagnostics_status_counts.png", "diagnostic status counts", "status", "count")

    if rosout_t:
        plt.figure()
        plt.plot(rosout_t, rosout_level, "o", label="rosout level")
        plt.yticks([10, 20, 30, 40, 50], ["DEBUG", "INFO", "WARN", "ERROR", "FATAL"])
        save_plot(output_dir, "rosout_levels.png", "rosout levels", "time [s]", "level")

    summary_path = output_dir / "summary.txt"
    with summary_path.open("w", encoding="utf-8") as f:
        f.write("Yahboom 2WD bag summary\n")
        f.write("=======================\n\n")
        f.write(f"bag_path: {bag_path}\n")
        f.write(f"storage_id: {storage_id}\n")
        f.write(f"namespace: {namespace or '<none>'}\n\n")
        f.write("Selected topics:\n")
        for key, value in topics.items():
            f.write(f"  {key:12s}: {value if value else 'not found'}\n")
        f.write("\nMessage counts:\n")
        for topic, count in sorted(counts.items()):
            f.write(f"  {topic}: {count}\n")

        if cmd_t:
            cmd_duration = cmd_t[-1] - cmd_t[0] if len(cmd_t) > 1 else 0.0
            avg_cmd_vx = sum(cmd_lin_x) / len(cmd_lin_x)
            expected_distance_by_samples = 0.0
            for i in range(1, len(cmd_t)):
                expected_distance_by_samples += 0.5 * (cmd_lin_x[i] + cmd_lin_x[i - 1]) * (cmd_t[i] - cmd_t[i - 1])
            f.write("\nCommand summary:\n")
            f.write(f"  cmd duration from bag samples: {cmd_duration:.3f} s\n")
            f.write(f"  average cmd linear.x: {avg_cmd_vx:.4f} m/s\n")
            f.write(f"  integrated commanded distance: {expected_distance_by_samples:.4f} m\n")

        if odom_t:
            dx = odom_x[-1] - odom_x[0]
            dy = odom_y[-1] - odom_y[0]
            displacement = math.hypot(dx, dy)
            length = path_length(odom_x, odom_y)
            avg_odom_speed = sum(odom_speed) / len(odom_speed)
            f.write("\nOdometry summary:\n")
            f.write(f"  final dx: {dx:.4f} m\n")
            f.write(f"  final dy: {dy:.4f} m\n")
            f.write(f"  final displacement: {displacement:.4f} m\n")
            f.write(f"  path length: {length:.4f} m\n")
            f.write(f"  average odom speed: {avg_odom_speed:.4f} m/s\n")

        if enc_t:
            f.write("\nEncoder delta summary:\n")
            f.write(f"  M1 delta: {enc_m1[-1] - enc_m1[0]} ticks\n")
            f.write(f"  M2 delta: {enc_m2[-1] - enc_m2[0]} ticks\n")
            f.write(f"  M3 delta: {enc_m3[-1] - enc_m3[0]} ticks\n")
            f.write(f"  M4 delta: {enc_m4[-1] - enc_m4[0]} ticks\n")

        if batt_t:
            f.write("\nBattery summary:\n")
            f.write(f"  min voltage: {min(batt_v):.2f} V\n")
            f.write(f"  max voltage: {max(batt_v):.2f} V\n")

        if rosout_t:
            f.write("\nRosout messages:\n")
            for t, level, name, msg in zip(rosout_t, rosout_level, rosout_name, rosout_msg):
                f.write(f"  {t:10.3f} s [{level}] {name}: {msg}\n")

        if diag_t:
            f.write("\nDiagnostic messages:\n")
            for t, level, name, message in zip(diag_t, diag_level, diag_name, diag_message):
                f.write(f"  {t:10.3f} s [{level}] {name}: {message}\n")

    print(f"Saved: {summary_path}")
    print(f"\nDone. All plots and summary were written to:\n{output_dir}")


if __name__ == "__main__":
    main()
