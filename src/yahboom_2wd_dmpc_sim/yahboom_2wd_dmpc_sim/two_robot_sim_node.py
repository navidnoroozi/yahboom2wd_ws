from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import math
import random

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_from_yaw(yaw: float) -> tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    w: float = 0.0
    last_cmd_time_s: float = 0.0
    left_ticks: int = 0
    right_ticks: int = 0


class TwoRobotSimNode(Node):
    """Lightweight local-odom simulator for the Yahboom two-robot DMPC stack.

    The simulator intentionally mimics the real robots' local odometry behavior:
    both /robot1/odom and /robot2/odom start near local (0, 0, 0). The VM-side
    DMPC coordinator must still be given robot1_initial_* and robot2_initial_*
    parameters to transform these local odometries into the common world/map
    frame. This makes the simulation interface compatible with the hardware test.
    """

    def __init__(self) -> None:
        super().__init__("two_robot_sim_node")

        self.declare_parameter("robot_namespaces", ["robot1", "robot2"])
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("wheel_radius", 0.0325)
        self.declare_parameter("wheel_separation", 0.120)
        self.declare_parameter("encoder_ticks_per_meter", 9000.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("battery_voltage", 12.0)
        self.declare_parameter("linear_velocity_scale", 1.0)
        self.declare_parameter("angular_velocity_scale", 1.0)
        self.declare_parameter("odom_noise_std_xy", 0.0)
        self.declare_parameter("odom_noise_std_yaw", 0.0)
        self.declare_parameter("publish_tf", True)

        self.robot_namespaces: List[str] = [str(x).strip("/") for x in self.get_parameter("robot_namespaces").value]
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.encoder_ticks_per_meter = float(self.get_parameter("encoder_ticks_per_meter").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.battery_voltage = float(self.get_parameter("battery_voltage").value)
        self.linear_velocity_scale = float(self.get_parameter("linear_velocity_scale").value)
        self.angular_velocity_scale = float(self.get_parameter("angular_velocity_scale").value)
        self.odom_noise_std_xy = float(self.get_parameter("odom_noise_std_xy").value)
        self.odom_noise_std_yaw = float(self.get_parameter("odom_noise_std_yaw").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.states: Dict[str, RobotState] = {ns: RobotState() for ns in self.robot_namespaces}
        now_s = self.get_clock().now().nanoseconds * 1e-9
        for st in self.states.values():
            st.last_cmd_time_s = now_s

        self.odom_publishers = {}
        self.imu_publishers = {}
        self.encoder_publishers = {}
        self.battery_publishers = {}
        self.diag_publishers = {}
        self.cmd_subscriptions = []
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        for ns in self.robot_namespaces:
            self.odom_publishers[ns] = self.create_publisher(Odometry, f"/{ns}/odom", 10)
            self.imu_publishers[ns] = self.create_publisher(Imu, f"/{ns}/imu/data", 10)
            self.encoder_publishers[ns] = self.create_publisher(Int32MultiArray, f"/{ns}/encoder_ticks", 10)
            self.battery_publishers[ns] = self.create_publisher(BatteryState, f"/{ns}/battery_state", 10)
            self.diag_publishers[ns] = self.create_publisher(DiagnosticArray, f"/{ns}/diagnostics", 10)
            self.cmd_subscriptions.append(
                self.create_subscription(Twist, f"/{ns}/cmd_vel", self._make_cmd_callback(ns), 10)
            )

        self.last_update_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1e-6), self._step)
        self.get_logger().info(
            f"Two-robot simulator started for namespaces={self.robot_namespaces}; "
            "odom is local per robot, as on the physical Yahboom robots."
        )

    def _make_cmd_callback(self, ns: str):
        def _cb(msg: Twist) -> None:
            st = self.states[ns]
            st.v = float(msg.linear.x) * self.linear_velocity_scale
            st.w = float(msg.angular.z) * self.angular_velocity_scale
            st.last_cmd_time_s = self.get_clock().now().nanoseconds * 1e-9
        return _cb

    def _step(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_update_time).nanoseconds * 1e-9, 0.0)
        self.last_update_time = now
        now_s = now.nanoseconds * 1e-9

        for ns, st in self.states.items():
            if now_s - st.last_cmd_time_s > self.cmd_timeout_s:
                st.v = 0.0
                st.w = 0.0

            # Midpoint integration for planar unicycle kinematics in the robot's local odom frame.
            yaw_mid = st.yaw + 0.5 * st.w * dt
            st.x += st.v * math.cos(yaw_mid) * dt
            st.y += st.v * math.sin(yaw_mid) * dt
            st.yaw = normalize_angle(st.yaw + st.w * dt)

            v_left = st.v - 0.5 * self.wheel_separation * st.w
            v_right = st.v + 0.5 * self.wheel_separation * st.w
            st.left_ticks += int(round(v_left * dt * self.encoder_ticks_per_meter))
            st.right_ticks += int(round(v_right * dt * self.encoder_ticks_per_meter))

            self._publish_robot(ns, st, now)

    def _publish_robot(self, ns: str, st: RobotState, stamp) -> None:  # noqa: ANN001
        x_noise = random.gauss(0.0, self.odom_noise_std_xy) if self.odom_noise_std_xy > 0.0 else 0.0
        y_noise = random.gauss(0.0, self.odom_noise_std_xy) if self.odom_noise_std_xy > 0.0 else 0.0
        yaw_noise = random.gauss(0.0, self.odom_noise_std_yaw) if self.odom_noise_std_yaw > 0.0 else 0.0
        yaw = normalize_angle(st.yaw + yaw_noise)
        qz, qw = quat_from_yaw(yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = f"{ns}/odom"
        odom.child_frame_id = f"{ns}/base_footprint"
        odom.pose.pose.position.x = st.x + x_noise
        odom.pose.pose.position.y = st.y + y_noise
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = st.v
        odom.twist.twist.angular.z = st.w
        self.odom_publishers[ns].publish(odom)

        imu = Imu()
        imu.header.stamp = odom.header.stamp
        imu.header.frame_id = f"{ns}/imu_link"
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.angular_velocity.z = st.w
        self.imu_publishers[ns].publish(imu)

        ticks = Int32MultiArray()
        # Match the hardware convention [M1, M2, M3, M4], where M2/M4 are used.
        ticks.data = [0, int(st.left_ticks), 0, int(st.right_ticks)]
        self.encoder_publishers[ns].publish(ticks)

        batt = BatteryState()
        batt.header.stamp = odom.header.stamp
        batt.voltage = self.battery_voltage
        batt.percentage = 1.0
        self.battery_publishers[ns].publish(batt)

        diag = DiagnosticArray()
        diag.header.stamp = odom.header.stamp
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = f"{ns}/two_robot_sim_node"
        status.message = "simulated"
        status.values = [
            KeyValue(key="v", value=f"{st.v:.4f}"),
            KeyValue(key="w", value=f"{st.w:.4f}"),
            KeyValue(key="x_local", value=f"{st.x:.4f}"),
            KeyValue(key="y_local", value=f"{st.y:.4f}"),
            KeyValue(key="yaw_local", value=f"{st.yaw:.4f}"),
        ]
        diag.status.append(status)
        self.diag_publishers[ns].publish(diag)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = odom.header.stamp
            tf.header.frame_id = f"{ns}/odom"
            tf.child_frame_id = f"{ns}/base_footprint"
            tf.transform.translation.x = odom.pose.pose.position.x
            tf.transform.translation.y = odom.pose.pose.position.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwoRobotSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
