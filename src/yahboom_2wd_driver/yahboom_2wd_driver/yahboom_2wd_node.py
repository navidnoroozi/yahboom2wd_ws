#!/usr/bin/env python3
"""ROS 2 interface node for a custom Yahboom 2WD differential robot.

The node intentionally does not modify STM32 firmware. It wraps the Yahboom
Rosmaster Python serial protocol and exposes a small ROS 2 API:

Subscribed:
  cmd_vel (geometry_msgs/Twist)
Published:
  odom (nav_msgs/Odometry)
  imu/data (sensor_msgs/Imu)
  battery_state (sensor_msgs/BatteryState)
  encoder_ticks (std_msgs/Int32MultiArray)
  diagnostics (diagnostic_msgs/DiagnosticArray)

Two command modes are provided:
  motion   : calls Rosmaster.set_car_motion(vx, 0, wz). This keeps Yahboom's
             firmware-level motion PID path and is the recommended first test.
  pwm_diff : computes differential left/right wheel speeds and maps them to
             Rosmaster.set_motor(...). This is useful for commissioning the M2/M4
             wiring, but the uploaded Rosmaster_Lib documents set_motor as PWM,
             not encoder-speed control.
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Int32MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import TransformBroadcaster

try:  # Prefer a system-installed Yahboom driver library.
    from Rosmaster_Lib import Rosmaster  # type: ignore
except ImportError:  # Fall back to the vendored copy shipped in this package.
    from .vendor.Rosmaster_Lib import Rosmaster  # type: ignore


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class Yahboom2WDNode(Node):
    def __init__(self) -> None:
        super().__init__('yahboom_2wd_node')

        # Serial and Yahboom board parameters.
        self.declare_parameter('serial_port', '/dev/myserial')
        self.declare_parameter('car_type', 4)  # Yahboom Rosmaster CARTYPE_X1 in the uploaded library.
        self.declare_parameter('debug_serial', False)
        self.declare_parameter('enable_auto_report', True)
        self.declare_parameter('write_car_type_on_startup', False)

        # Commanding / robot geometry.
        self.declare_parameter('command_mode', 'motion')  # motion | pwm_diff
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_separation', 0.120)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 2.5)
        self.declare_parameter('max_wheel_linear_speed', 0.5)
        self.declare_parameter('left_motor_port', 2)
        self.declare_parameter('right_motor_port', 4)
        self.declare_parameter('left_motor_sign', 1)
        self.declare_parameter('right_motor_sign', 1)
        self.declare_parameter('linear_cmd_scale', 1.0)
        self.declare_parameter('angular_cmd_scale', 1.0)
        self.declare_parameter('cmd_timeout_s', 0.5)

        # State publication.
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('use_imu_yaw_for_odom', True)
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('reset_odom_on_start', True)
        self.declare_parameter('odom_linear_scale', 1.0)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.car_type = int(self.get_parameter('car_type').value)
        self.command_mode = str(self.get_parameter('command_mode').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_wheel_linear_speed = float(self.get_parameter('max_wheel_linear_speed').value)
        self.left_motor_port = int(self.get_parameter('left_motor_port').value)
        self.right_motor_port = int(self.get_parameter('right_motor_port').value)
        self.left_motor_sign = int(self.get_parameter('left_motor_sign').value)
        self.right_motor_sign = int(self.get_parameter('right_motor_sign').value)
        self.linear_cmd_scale = float(self.get_parameter('linear_cmd_scale').value)
        self.angular_cmd_scale = float(self.get_parameter('angular_cmd_scale').value)
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.use_imu_yaw_for_odom = bool(self.get_parameter('use_imu_yaw_for_odom').value)
        self.invert_yaw = bool(self.get_parameter('invert_yaw').value)
        self.reset_odom_on_start = bool(self.get_parameter('reset_odom_on_start').value)
        self.odom_linear_scale = float(self.get_parameter('odom_linear_scale').value)
        self.yaw_offset: Optional[float] = None

        if self.command_mode not in ('motion', 'pwm_diff'):
            raise ValueError("command_mode must be 'motion' or 'pwm_diff'")
        if self.left_motor_port not in (1, 2, 3, 4) or self.right_motor_port not in (1, 2, 3, 4):
            raise ValueError('left_motor_port and right_motor_port must be in [1, 2, 3, 4]')
        if self.left_motor_port == self.right_motor_port:
            raise ValueError('left_motor_port and right_motor_port must be different')

        self.bot = Rosmaster(car_type=self.car_type, com=self.serial_port, debug=bool(self.get_parameter('debug_serial').value))
        self.bot.create_receive_threading()
        if bool(self.get_parameter('enable_auto_report').value):
            self.bot.set_auto_report_state(True, False)
        if bool(self.get_parameter('write_car_type_on_startup').value):
            self.get_logger().warn('Writing car_type to STM32 flash/state because write_car_type_on_startup=true')
            self.bot.set_car_type(self.car_type)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos)
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', qos)
        self.encoder_pub = self.create_publisher(Int32MultiArray, 'encoder_ticks', qos)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', qos)

        self.last_cmd_time: Optional[float] = None
        self.last_update_time: Optional[float] = None
        self.last_diag_time: float = 0.0
        self.current_cmd = Twist()
        self.command_active = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_board_version = -1.0

        self.stop_robot()

        update_rate = float(self.get_parameter('update_rate_hz').value)
        self.timer = self.create_timer(1.0 / update_rate, self.update)
        self.get_logger().info(
            f'Yahboom 2WD interface started on {self.serial_port}; '
            f'command_mode={self.command_mode}; left=M{self.left_motor_port}, right=M{self.right_motor_port}'
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.current_cmd = msg
        self.last_cmd_time = time.monotonic()
        self.command_active = True

    def update(self) -> None:
        now_ros = self.get_clock().now()
        now = time.monotonic()

        if self.command_active and self.last_cmd_time is not None:
            if now - self.last_cmd_time > self.cmd_timeout_s:
                self.get_logger().warn('cmd_vel timeout; stopping robot', throttle_duration_sec=2.0)
                self.stop_robot()
                self.command_active = False
            else:
                self.apply_cmd_vel(self.current_cmd)

        vx_raw, vy_raw, wz = self.safe_get_motion_data()
        vx = vx_raw * self.odom_linear_scale
        vy = vy_raw * self.odom_linear_scale

        roll, pitch, yaw = self.safe_get_imu_attitude()
        gx, gy, gz = self.safe_get_gyro()
        ax, ay, az = self.safe_get_accel()
        encoders = self.safe_get_encoders()
        battery_voltage = self.safe_get_battery_voltage()

        if self.invert_yaw:
            yaw = -yaw
            wz = -wz
            gz = -gz
        
        # The board IMU yaw is an absolute heading at node startup. For odometry,
        # the odom frame should normally start at x=0, y=0, theta=0 when this
        # node starts. reset_odom_on_start therefore subtracts the initial yaw
        # before using the IMU heading for odom integration. The IMU message still
        # publishes the raw board orientation.
        odom_yaw = yaw
        if self.use_imu_yaw_for_odom and self.reset_odom_on_start:
            if self.yaw_offset is None:
                self.yaw_offset = yaw
                self.get_logger().info(f'Using initial IMU yaw offset for odom: {self.yaw_offset:.4f} rad')
            odom_yaw = normalize_angle(yaw - self.yaw_offset)

        if self.last_update_time is None:
            dt = 0.0
        else:
            dt = max(0.0, now - self.last_update_time)
        self.last_update_time = now

        if dt > 0.0:
            if self.use_imu_yaw_for_odom:
                self.theta = odom_yaw
            else:
                self.theta = normalize_angle(self.theta + wz * dt)
            c = math.cos(self.theta)
            s = math.sin(self.theta)
            self.x += (vx * c - vy * s) * dt
            self.y += (vx * s + vy * c) * dt

        self.publish_odom(now_ros, vx, vy, wz)
        self.publish_imu(now_ros, roll, pitch, yaw, gx, gy, gz, ax, ay, az)
        self.publish_battery(now_ros, battery_voltage)
        self.publish_encoders(encoders)
        if now - self.last_diag_time > 1.0:
            self.publish_diagnostics(now_ros, battery_voltage)
            self.last_diag_time = now

    def apply_cmd_vel(self, msg: Twist) -> None:
        vx = clamp(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        wz = clamp(msg.angular.z, -self.max_angular_speed, self.max_angular_speed)

        if self.command_mode == 'motion':
            self.bot.set_car_motion(vx * self.linear_cmd_scale, 0.0, wz * self.angular_cmd_scale)
            return

        # pwm_diff mode: direct M2/M4 PWM commissioning fallback.
        v_left = vx - 0.5 * self.wheel_separation * wz
        v_right = vx + 0.5 * self.wheel_separation * wz
        pwm_left = int(round(100.0 * v_left / max(self.max_wheel_linear_speed, 1e-6)))
        pwm_right = int(round(100.0 * v_right / max(self.max_wheel_linear_speed, 1e-6)))
        pwm_left = int(clamp(self.left_motor_sign * pwm_left, -100, 100))
        pwm_right = int(clamp(self.right_motor_sign * pwm_right, -100, 100))
        motor_values = [127, 127, 127, 127]  # 127 means leave that motor unchanged in Yahboom library.
        motor_values[self.left_motor_port - 1] = pwm_left
        motor_values[self.right_motor_port - 1] = pwm_right
        self.bot.set_motor(*motor_values)

    def stop_robot(self) -> None:
        try:
            if self.command_mode == 'motion':
                self.bot.set_car_motion(0.0, 0.0, 0.0)
            else:
                motor_values = [127, 127, 127, 127]
                motor_values[self.left_motor_port - 1] = 0
                motor_values[self.right_motor_port - 1] = 0
                self.bot.set_motor(*motor_values)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to stop robot: {exc}')

    def safe_get_motion_data(self) -> Tuple[float, float, float]:
        try:
            return tuple(float(v) for v in self.bot.get_motion_data())  # type: ignore[return-value]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Could not read motion data: {exc}', throttle_duration_sec=5.0)
            return 0.0, 0.0, 0.0

    def safe_get_imu_attitude(self) -> Tuple[float, float, float]:
        try:
            return tuple(float(v) for v in self.bot.get_imu_attitude_data(ToAngle=False))  # type: ignore[return-value]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Could not read IMU attitude: {exc}', throttle_duration_sec=5.0)
            return 0.0, 0.0, self.theta

    def safe_get_gyro(self) -> Tuple[float, float, float]:
        try:
            return tuple(float(v) for v in self.bot.get_gyroscope_data())  # type: ignore[return-value]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Could not read gyro: {exc}', throttle_duration_sec=5.0)
            return 0.0, 0.0, 0.0

    def safe_get_accel(self) -> Tuple[float, float, float]:
        try:
            return tuple(float(v) for v in self.bot.get_accelerometer_data())  # type: ignore[return-value]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Could not read accelerometer: {exc}', throttle_duration_sec=5.0)
            return 0.0, 0.0, 0.0

    def safe_get_encoders(self) -> Tuple[int, int, int, int]:
        try:
            return tuple(int(v) for v in self.bot.get_motor_encoder())  # type: ignore[return-value]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Could not read encoders: {exc}', throttle_duration_sec=5.0)
            return 0, 0, 0, 0

    def safe_get_battery_voltage(self) -> float:
        try:
            return float(self.bot.get_battery_voltage())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Could not read battery voltage: {exc}', throttle_duration_sec=5.0)
            return 0.0

    def publish_odom(self, now_ros, vx: float, vy: float, wz: float) -> None:
        msg = Odometry()
        msg.header.stamp = now_ros.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[7] = 0.02
        msg.twist.covariance[35] = 0.05
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    def publish_imu(self, now_ros, roll: float, pitch: float, yaw: float,
                    gx: float, gy: float, gz: float, ax: float, ay: float, az: float) -> None:
        msg = Imu()
        msg.header.stamp = now_ros.to_msg()
        msg.header.frame_id = self.imu_frame_id
        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.orientation_covariance[0] = 0.05
        msg.orientation_covariance[4] = 0.05
        msg.orientation_covariance[8] = 0.10
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02
        msg.linear_acceleration_covariance[0] = 0.20
        msg.linear_acceleration_covariance[4] = 0.20
        msg.linear_acceleration_covariance[8] = 0.20
        self.imu_pub.publish(msg)

    def publish_battery(self, now_ros, voltage: float) -> None:
        msg = BatteryState()
        msg.header.stamp = now_ros.to_msg()
        msg.voltage = voltage
        msg.present = voltage > 0.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery_pub.publish(msg)

    def publish_encoders(self, encoders: Tuple[int, int, int, int]) -> None:
        msg = Int32MultiArray()
        msg.data = list(encoders)
        self.encoder_pub.publish(msg)

    def publish_diagnostics(self, now_ros, battery_voltage: float) -> None:
        status = DiagnosticStatus()
        status.name = f'{self.get_name()}: Yahboom ROS control board'
        status.hardware_id = self.serial_port
        status.level = DiagnosticStatus.OK if battery_voltage == 0.0 or battery_voltage >= 10.5 else DiagnosticStatus.WARN
        status.message = 'OK' if status.level == DiagnosticStatus.OK else 'Battery voltage low'
        status.values = [
            KeyValue(key='serial_port', value=self.serial_port),
            KeyValue(key='command_mode', value=self.command_mode),
            KeyValue(key='battery_voltage', value=f'{battery_voltage:.2f}'),
            KeyValue(key='left_motor_port', value=f'M{self.left_motor_port}'),
            KeyValue(key='right_motor_port', value=f'M{self.right_motor_port}'),
            KeyValue(key='odom_linear_scale', value=f'{self.odom_linear_scale:.3f}'),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = now_ros.to_msg()
        msg.status = [status]
        self.diag_pub.publish(msg)

    def destroy_node(self) -> bool:
        self.stop_robot()
        try:
            self.bot.reset_car_state()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Yahboom2WDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
