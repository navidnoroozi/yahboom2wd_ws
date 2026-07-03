#!/usr/bin/env python3
"""Odometry-feedback path-following test node for Yahboom 2WD robots.

This node is intentionally simple and conservative. It generates a reference
trajectory for one of several standard commissioning scenarios and tracks it
using odometry feedback. It publishes geometry_msgs/Twist to the existing
Yahboom bridge topic, for example /robot1/cmd_vel.

Supported scenarios:
  straight
  pure_rotation
  arc
  circle
  stop_and_go
  sinusoidal

The reference trajectory is initialized relative to the robot pose at the first
received odometry message. This means every test starts from the robot's current
location and heading.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


@dataclass
class ReferenceState:
    """Reference state in the test-local frame."""

    x: float
    y: float
    theta: float
    v: float
    omega: float
    done: bool = False


class PathFollowerNode(Node):
    """Simple odometry-feedback path follower for single-robot tests."""

    def __init__(self) -> None:
        super().__init__('path_follower_node')

        # Robot/topic parameters.
        self.declare_parameter('robot_namespace', 'robot1')
        self.declare_parameter('cmd_vel_topic', '')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('control_rate_hz', 20.0)

        # Scenario selection.
        self.declare_parameter('scenario', 'straight')
        self.declare_parameter('linear_speed', 0.08)       # m/s
        self.declare_parameter('angular_speed', 0.20)      # rad/s, used for pure rotation
        self.declare_parameter('distance', 1.0)            # m, straight test
        self.declare_parameter('radius', 1.0)              # m, arc/circle
        self.declare_parameter('arc_angle', math.pi / 2.0) # rad, arc
        self.declare_parameter('rotation_angle', math.pi / 2.0) # rad, pure rotation
        self.declare_parameter('turn_direction', 'left')   # left | right
        self.declare_parameter('amplitude', 0.20)          # m, sinusoidal y=A sin(kx)
        self.declare_parameter('wavelength', 2.0)          # m, sinusoidal
        self.declare_parameter('path_length', 2.0)         # m along x for sinusoidal
        self.declare_parameter('move_time', 3.0)           # s, stop-and-go
        self.declare_parameter('stop_time', 2.0)           # s, stop-and-go
        self.declare_parameter('cycles', 3)                # stop-and-go cycles

        # Feedback gains. These are intentionally mild for indoor commissioning.
        self.declare_parameter('kx', 0.8)                  # longitudinal error gain
        self.declare_parameter('ky', 1.5)                  # lateral error gain
        self.declare_parameter('ktheta', 1.8)              # heading error gain

        # Safety limits.
        self.declare_parameter('max_linear_speed', 0.12)   # m/s
        self.declare_parameter('max_angular_speed', 0.60)  # rad/s
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('stop_at_end', True)
        self.declare_parameter('goal_tolerance_xy', 0.03)  # m
        self.declare_parameter('goal_tolerance_yaw', 0.05) # rad

        ns = str(self.get_parameter('robot_namespace').value).strip('/')
        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_topic = cmd_topic if cmd_topic else f'/{ns}/cmd_vel'
        self.odom_topic = odom_topic if odom_topic else f'/{ns}/odom'
        self.reference_topic = f'/{ns}/path_test/reference_pose'
        self.error_topic = f'/{ns}/path_test/tracking_error'

        self.scenario = self._normalize_scenario(str(self.get_parameter('scenario').value))
        self.linear_speed = abs(float(self.get_parameter('linear_speed').value))
        self.angular_speed = abs(float(self.get_parameter('angular_speed').value))
        self.distance = abs(float(self.get_parameter('distance').value))
        self.radius = abs(float(self.get_parameter('radius').value))
        self.arc_angle = abs(float(self.get_parameter('arc_angle').value))
        self.rotation_angle = abs(float(self.get_parameter('rotation_angle').value))
        self.turn_direction = str(self.get_parameter('turn_direction').value).lower()
        self.turn_sign = -1.0 if self.turn_direction in ('right', 'cw', 'clockwise', '-1') else 1.0
        self.amplitude = float(self.get_parameter('amplitude').value)
        self.wavelength = max(1e-6, float(self.get_parameter('wavelength').value))
        self.path_length = abs(float(self.get_parameter('path_length').value))
        self.move_time = max(0.0, float(self.get_parameter('move_time').value))
        self.stop_time = max(0.0, float(self.get_parameter('stop_time').value))
        self.cycles = max(1, int(self.get_parameter('cycles').value))
        self.kx = float(self.get_parameter('kx').value)
        self.ky = float(self.get_parameter('ky').value)
        self.ktheta = float(self.get_parameter('ktheta').value)
        self.max_linear_speed = abs(float(self.get_parameter('max_linear_speed').value))
        self.max_angular_speed = abs(float(self.get_parameter('max_angular_speed').value))
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.stop_at_end = bool(self.get_parameter('stop_at_end').value)
        self.goal_tolerance_xy = abs(float(self.get_parameter('goal_tolerance_xy').value))
        self.goal_tolerance_yaw = abs(float(self.get_parameter('goal_tolerance_yaw').value))

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_theta: Optional[float] = None

        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.start_theta: Optional[float] = None
        self.start_time: Optional[float] = None
        self.finished = False
        self.last_log_time = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, qos)
        self.ref_pub = self.create_publisher(PoseStamped, self.reference_topic, qos)
        self.err_pub = self.create_publisher(Vector3Stamped, self.error_topic, qos)

        rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate_hz, self.control_step)

        self.get_logger().info(
            f'Path follower ready: scenario={self.scenario}, cmd_topic={self.cmd_topic}, '
            f'odom_topic={self.odom_topic}, max_v={self.max_linear_speed:.3f}, '
            f'max_w={self.max_angular_speed:.3f}'
        )

    @staticmethod
    def _normalize_scenario(name: str) -> str:
        key = name.strip().lower().replace('-', '_').replace(' ', '_')
        aliases = {
            'line': 'straight',
            'straight_line': 'straight',
            'rotation': 'pure_rotation',
            'rotate': 'pure_rotation',
            'pure_rotate': 'pure_rotation',
            'constant_arc': 'arc',
            'constant_radius_arc': 'arc',
            'stopgo': 'stop_and_go',
            'stop_go': 'stop_and_go',
            'sine': 'sinusoidal',
            'sin': 'sinusoidal',
            'sinusoid': 'sinusoidal',
        }
        key = aliases.get(key, key)
        valid = {'straight', 'pure_rotation', 'arc', 'circle', 'stop_and_go', 'sinusoidal'}
        if key not in valid:
            raise ValueError(f'Unsupported scenario "{name}". Valid scenarios: {sorted(valid)}')
        return key

    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.current_theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        if self.start_time is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_theta = self.current_theta
            self.start_time = time.monotonic()
            self.get_logger().info(
                f'Test initialized at odom pose: x={self.start_x:.3f}, '
                f'y={self.start_y:.3f}, yaw={self.start_theta:.3f} rad'
            )

    def control_step(self) -> None:
        if self.current_x is None or self.current_y is None or self.current_theta is None:
            self._publish_zero()
            self.get_logger().warn('Waiting for odometry before starting path test...', throttle_duration_sec=2.0)
            return

        if self.start_time is None or self.start_x is None or self.start_y is None or self.start_theta is None:
            self._publish_zero()
            return

        elapsed = time.monotonic() - self.start_time
        ref_local = self.reference_at(elapsed)
        xr, yr, theta_r = self.local_to_global(ref_local.x, ref_local.y, ref_local.theta)

        dx = xr - self.current_x
        dy = yr - self.current_y
        th = self.current_theta
        ex = math.cos(th) * dx + math.sin(th) * dy
        ey = -math.sin(th) * dx + math.cos(th) * dy
        eth = normalize_angle(theta_r - th)

        if self.finished:
            self._publish_zero()
            return

        # Mark completion once the reference generator is done and the robot is
        # close enough to the final pose. For stop-and-go and sinusoidal tests,
        # this prevents stopping too early if the robot lags behind slightly.
        if ref_local.done:
            if abs(ex) < self.goal_tolerance_xy and abs(ey) < self.goal_tolerance_xy and abs(eth) < self.goal_tolerance_yaw:
                self.finished = True
                self.get_logger().info('Path test finished within tolerance; sending zero velocity.')
                self._publish_zero()
                return

        v_cmd, w_cmd = self.feedback_law(ref_local.v, ref_local.omega, ex, ey, eth)
        self.publish_cmd(v_cmd, w_cmd)
        self.publish_reference_and_error(xr, yr, theta_r, ex, ey, eth)

        now = time.monotonic()
        if now - self.last_log_time > 1.0:
            self.get_logger().info(
                f't={elapsed:.1f}s ref=({xr:.2f},{yr:.2f},{theta_r:.2f}) '
                f'err_body=({ex:.3f},{ey:.3f},{eth:.3f}) cmd=({v_cmd:.3f},{w_cmd:.3f})'
            )
            self.last_log_time = now

    def feedback_law(self, v_ref: float, w_ref: float, ex: float, ey: float, eth: float) -> Tuple[float, float]:
        """Unicycle tracking controller.

        The equations are a conservative variant of the standard body-frame
        tracking controller. For pure rotation, v_ref is zero and the heading
        correction term dominates angular velocity.
        """
        v_cmd = v_ref * math.cos(eth) + self.kx * ex
        w_cmd = w_ref + self.ky * v_ref * ey + self.ktheta * math.sin(eth)

        if not self.allow_reverse:
            v_cmd = max(0.0, v_cmd)

        v_cmd = clamp(v_cmd, -self.max_linear_speed, self.max_linear_speed)
        w_cmd = clamp(w_cmd, -self.max_angular_speed, self.max_angular_speed)
        return v_cmd, w_cmd

    def reference_at(self, t: float) -> ReferenceState:
        if self.scenario == 'straight':
            return self.ref_straight(t)
        if self.scenario == 'pure_rotation':
            return self.ref_pure_rotation(t)
        if self.scenario == 'arc':
            return self.ref_arc(t, self.arc_angle)
        if self.scenario == 'circle':
            return self.ref_arc(t, 2.0 * math.pi)
        if self.scenario == 'stop_and_go':
            return self.ref_stop_and_go(t)
        if self.scenario == 'sinusoidal':
            return self.ref_sinusoidal(t)
        raise RuntimeError(f'Unhandled scenario: {self.scenario}')

    def ref_straight(self, t: float) -> ReferenceState:
        v = self.linear_speed
        duration = self.distance / max(v, 1e-6)
        tau = min(t, duration)
        x = v * tau
        done = t >= duration
        return ReferenceState(x=x, y=0.0, theta=0.0, v=0.0 if done else v, omega=0.0, done=done)

    def ref_pure_rotation(self, t: float) -> ReferenceState:
        w = self.turn_sign * self.angular_speed
        duration = self.rotation_angle / max(abs(w), 1e-6)
        tau = min(t, duration)
        theta = w * tau
        done = t >= duration
        return ReferenceState(x=0.0, y=0.0, theta=theta, v=0.0, omega=0.0 if done else w, done=done)

    def ref_arc(self, t: float, target_angle: float) -> ReferenceState:
        v = self.linear_speed
        r = max(self.radius, 1e-6)
        w = self.turn_sign * v / r
        duration = target_angle / max(abs(w), 1e-6)
        tau = min(t, duration)
        phi = abs(w) * tau
        signed_phi = self.turn_sign * phi
        x = r * math.sin(phi)
        y = self.turn_sign * r * (1.0 - math.cos(phi))
        done = t >= duration
        return ReferenceState(x=x, y=y, theta=signed_phi, v=0.0 if done else v, omega=0.0 if done else w, done=done)

    def ref_stop_and_go(self, t: float) -> ReferenceState:
        period = self.move_time + self.stop_time
        total_duration = self.cycles * period
        t_clamped = min(t, total_duration)
        full_cycles = int(t_clamped // period) if period > 0.0 else self.cycles
        rem = t_clamped - full_cycles * period

        moving_time = full_cycles * self.move_time
        is_moving = rem < self.move_time and t < total_duration
        if is_moving:
            moving_time += rem
        else:
            moving_time += self.move_time

        x = self.linear_speed * moving_time
        done = t >= total_duration
        v = self.linear_speed if is_moving and not done else 0.0
        return ReferenceState(x=x, y=0.0, theta=0.0, v=v, omega=0.0, done=done)

    def ref_sinusoidal(self, t: float) -> ReferenceState:
        # Use x-progress as the independent variable. This is simple and stable
        # for small-amplitude indoor tests.
        x_speed = self.linear_speed
        duration = self.path_length / max(x_speed, 1e-6)
        tau = min(t, duration)
        x = x_speed * tau
        k = 2.0 * math.pi / self.wavelength
        y = self.amplitude * math.sin(k * x)
        dy_dx = self.amplitude * k * math.cos(k * x)
        d2y_dx2 = -self.amplitude * k * k * math.sin(k * x)
        theta = math.atan2(dy_dx, 1.0)

        # Reference translational speed along the curve and yaw rate from curvature.
        v_path = x_speed * math.sqrt(1.0 + dy_dx * dy_dx)
        curvature = d2y_dx2 / max((1.0 + dy_dx * dy_dx) ** 1.5, 1e-9)
        omega = curvature * v_path
        done = t >= duration
        return ReferenceState(x=x, y=y, theta=theta, v=0.0 if done else v_path, omega=0.0 if done else omega, done=done)

    def local_to_global(self, x_l: float, y_l: float, theta_l: float) -> Tuple[float, float, float]:
        assert self.start_x is not None
        assert self.start_y is not None
        assert self.start_theta is not None
        c = math.cos(self.start_theta)
        s = math.sin(self.start_theta)
        x_g = self.start_x + c * x_l - s * y_l
        y_g = self.start_y + s * x_l + c * y_l
        theta_g = normalize_angle(self.start_theta + theta_l)
        return x_g, y_g, theta_g

    def publish_cmd(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        self.publish_cmd(0.0, 0.0)

    def publish_reference_and_error(self, xr: float, yr: float, theta_r: float,
                                    ex: float, ey: float, eth: float) -> None:
        now = self.get_clock().now().to_msg()

        ref_msg = PoseStamped()
        ref_msg.header.stamp = now
        ref_msg.header.frame_id = self._odom_frame_id()
        ref_msg.pose.position.x = xr
        ref_msg.pose.position.y = yr
        qx, qy, qz, qw = quaternion_from_yaw(theta_r)
        ref_msg.pose.orientation.x = qx
        ref_msg.pose.orientation.y = qy
        ref_msg.pose.orientation.z = qz
        ref_msg.pose.orientation.w = qw
        self.ref_pub.publish(ref_msg)

        err_msg = Vector3Stamped()
        err_msg.header.stamp = now
        err_msg.header.frame_id = 'base_footprint'
        err_msg.vector.x = ex
        err_msg.vector.y = ey
        err_msg.vector.z = eth
        self.err_pub.publish(err_msg)

    def _odom_frame_id(self) -> str:
        ns = str(self.get_parameter('robot_namespace').value).strip('/')
        return f'{ns}/odom'

    def destroy_node(self) -> bool:
        for _ in range(3):
            self._publish_zero()
            time.sleep(0.02)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
