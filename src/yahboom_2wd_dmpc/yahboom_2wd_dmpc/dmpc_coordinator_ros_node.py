from __future__ import annotations

from dataclasses import replace
from typing import Dict, List, Optional

import math
import numpy as np
import zmq

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry

from .consensus_comm import dumps, loads, make_envelope
from .consensus_config import NetConfig


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    # yaw from quaternion, assuming planar motion
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    """Return (z, w) quaternion components for a planar yaw angle."""
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class DmpcCoordinatorRosNode(Node):
    """ROS 2 / ZeroMQ bridge for the two-robot Yahboom DMPC experiment.

    This node runs on the Ubuntu VM. It subscribes to /robot1/odom and /robot2/odom,
    sends the current multi-robot state to the local ZMQ controller nodes running
    on the Raspberry Pis, receives nominal/safe single-integrator commands, and
    converts those world-frame commands into unicycle cmd_vel messages.
    """

    def __init__(self) -> None:
        super().__init__("dmpc_coordinator_ros_node")

        # Topology / ROS interface
        self.declare_parameter("robot_namespaces", ["robot1", "robot2"])
        self.declare_parameter("agent_ids", [1, 2])
        # Scalar endpoint parameters are used by the launch file.
        # The legacy STRING_ARRAY parameter is kept for YAML/manual use.
        self.declare_parameter("robot1_controller_endpoint", "tcp://192.168.178.51:5601")
        self.declare_parameter("robot2_controller_endpoint", "tcp://192.168.178.52:5602")
        self.declare_parameter("controller_endpoints", ["tcp://192.168.178.51:5601", "tcp://192.168.178.52:5602"])
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("odom_timeout_s", 1.0)

        # Common world/map-frame initialization.  The Yahboom odometry of each
        # robot starts from its own local zero pose.  These parameters define
        # where that local zero pose is located in the shared world frame.
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("robot1_initial_x", 0.0)
        self.declare_parameter("robot1_initial_y", -0.45)
        self.declare_parameter("robot1_initial_yaw", 0.0)
        self.declare_parameter("robot2_initial_x", 0.0)
        self.declare_parameter("robot2_initial_y", 0.45)
        self.declare_parameter("robot2_initial_yaw", 0.0)

        # DMPC configuration used by coordinator-side neighbor logic and diagnostics.
        self.declare_parameter("model", "single_integrator")
        self.declare_parameter("graph", "complete")
        self.declare_parameter("objective_mode", "safe_formation")
        self.declare_parameter("auto_M", False)
        self.declare_parameter("M_manual", 5)
        self.declare_parameter("u_bound", 0.08)
        self.declare_parameter("r_bound", 5.0)
        self.declare_parameter("d_safe", 0.65)
        self.declare_parameter("formation_margin", 0.15)
        self.declare_parameter("formation_rotation_rad", 0.0)
        self.declare_parameter("formation_radius_override", 0.0)

        # Explicit-hybrid safety thresholds and hysteresis exposed from NetConfig.
        # Keep d_agent_exit below the desired formation distance. With the default
        # two-robot values, d_form = d_safe + formation_margin = 0.80 m, so
        # d_agent_enter=0.70 and d_agent_exit=0.75 are compatible.
        self.declare_parameter("safety_warning_radius", 1.20)
        self.declare_parameter("obstacle_warning_radius", 1.20)
        self.declare_parameter("d_agent_enter", 0.70)
        self.declare_parameter("d_agent_exit", 0.75)
        self.declare_parameter("d_obs_enter", 0.45)
        self.declare_parameter("d_obs_exit", 0.85)

        # Single-integrator explicit-hybrid gains.
        self.declare_parameter("modeC_repulsion_gain_si", 0.90)
        self.declare_parameter("modeO_target_gain_si", 0.95)
        self.declare_parameter("modeCO_repulsion_gain_si", 0.90)
        self.declare_parameter("modeCO_target_gain_si", 0.85)
        self.declare_parameter("nominal_blend_C_si", 0.35)
        self.declare_parameter("nominal_blend_O_si", 0.20)
        self.declare_parameter("nominal_blend_CO_si", 0.15)

        # Practical safety-filter margins.
        self.declare_parameter("pair_filter_margin", 0.04)
        self.declare_parameter("obs_filter_margin", 0.05)
        self.declare_parameter("filter_projection_passes", 4)

        self.declare_parameter("safety_enabled", True)
        self.declare_parameter("obstacles_enabled", False)
        self.declare_parameter("dt", 0.20)
        self.declare_parameter("w_track", 8.0)
        self.declare_parameter("w_du", 3.0)
        self.declare_parameter("w_u", 0.3)

        # Conversion from global single-integrator command to differential-drive Twist.
        self.declare_parameter("max_linear_speed", 0.07)
        self.declare_parameter("max_angular_speed", 0.35)
        self.declare_parameter("world_command_to_speed_scale", 1.0)
        self.declare_parameter("heading_gain", 1.8)
        self.declare_parameter("linear_deadband", 0.005)
        self.declare_parameter("stop_for_heading_error_rad", 1.20)
        self.declare_parameter("allow_reverse", False)

        self.robot_namespaces = [str(x).strip("/") for x in self.get_parameter("robot_namespaces").value]
        self.agent_ids = [int(x) for x in self.get_parameter("agent_ids").value]
        # Prefer scalar endpoint parameters to avoid ROS 2 Humble launch substitution
        # issues with STRING_ARRAY parameters. If the scalar parameters are empty,
        # fall back to the legacy controller_endpoints array.
        ep1 = str(self.get_parameter("robot1_controller_endpoint").value).strip()
        ep2 = str(self.get_parameter("robot2_controller_endpoint").value).strip()
        if ep1 and ep2:
            self.controller_endpoints = [ep1, ep2]
        else:
            self.controller_endpoints = [str(x) for x in self.get_parameter("controller_endpoints").value]
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.enable_motion = bool(self.get_parameter("enable_motion").value)
        self.odom_timeout_s = float(self.get_parameter("odom_timeout_s").value)
        self.world_frame = str(self.get_parameter("world_frame").value).strip() or "map"

        # Agent-id keyed initial pose of each robot's local odom frame in the shared world frame.
        self.initial_world_pose_by_agent = {
            1: np.array([
                float(self.get_parameter("robot1_initial_x").value),
                float(self.get_parameter("robot1_initial_y").value),
                float(self.get_parameter("robot1_initial_yaw").value),
            ], dtype=float),
            2: np.array([
                float(self.get_parameter("robot2_initial_x").value),
                float(self.get_parameter("robot2_initial_y").value),
                float(self.get_parameter("robot2_initial_yaw").value),
            ], dtype=float),
        }

        if len(self.robot_namespaces) != len(self.agent_ids):
            raise ValueError("robot_namespaces and agent_ids must have the same length")
        if len(self.controller_endpoints) != len(self.agent_ids):
            raise ValueError("controller_endpoints and agent_ids must have the same length")

        self.n_agents = len(self.agent_ids)
        self.cfg = self._make_cfg()

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.world_command_to_speed_scale = float(self.get_parameter("world_command_to_speed_scale").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.linear_deadband = float(self.get_parameter("linear_deadband").value)
        self.stop_for_heading_error_rad = float(self.get_parameter("stop_for_heading_error_rad").value)
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)

        self.odom_by_agent: Dict[int, Odometry] = {}
        self.odom_time_by_agent: Dict[int, rclpy.time.Time] = {}

        self.cmd_publishers: Dict[int, rclpy.publisher.Publisher] = {}
        self.u_debug_publishers: Dict[int, rclpy.publisher.Publisher] = {}
        self.pose_debug_publishers: Dict[int, rclpy.publisher.Publisher] = {}
        self.metrics_publisher = self.create_publisher(Vector3Stamped, "/dmpc/two_robot/metrics", 10)
        self.thresholds_publisher = self.create_publisher(Vector3Stamped, "/dmpc/two_robot/safety_thresholds", 10)
        self.odom_subscriptions = []

        for agent_id, ns in zip(self.agent_ids, self.robot_namespaces):
            self.cmd_publishers[agent_id] = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
            self.u_debug_publishers[agent_id] = self.create_publisher(Vector3Stamped, f"/dmpc/{ns}/u_world", 10)
            self.pose_debug_publishers[agent_id] = self.create_publisher(PoseStamped, f"/dmpc/{ns}/pose_world", 10)
            self.odom_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    f"/{ns}/odom",
                    self._make_odom_callback(agent_id),
                    10,
                )
            )

        self.ctx = zmq.Context.instance()
        self.req_socks: Dict[int, zmq.Socket] = {}
        self.endpoints_by_agent: Dict[int, str] = {}
        for agent_id, endpoint in zip(self.agent_ids, self.controller_endpoints):
            self.endpoints_by_agent[agent_id] = endpoint
            self.req_socks[agent_id] = self._make_req_socket(endpoint)
            self.get_logger().info(f"REQ -> controller for agent {agent_id}: {endpoint}")

        self.u_prev: Dict[int, np.ndarray] = {
            agent_id: np.zeros((self.cfg.dim,), dtype=float) for agent_id in self.agent_ids
        }
        self.outer_index = 0

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1e-6), self.control_step)

        self.get_logger().info(
            f"DMPC coordinator started for namespaces={self.robot_namespaces}, "
            f"agent_ids={self.agent_ids}, enable_motion={self.enable_motion}, "
            f"model={self.cfg.model}, n_agents={self.cfg.n_agents}, M={self.cfg.horizon_M()}"
        )
        self.get_logger().info(
            f"World-frame initialization: frame={self.world_frame}, "
            f"robot1=[{self.initial_world_pose_by_agent[1][0]:.3f}, {self.initial_world_pose_by_agent[1][1]:.3f}, {self.initial_world_pose_by_agent[1][2]:.3f}], "
            f"robot2=[{self.initial_world_pose_by_agent[2][0]:.3f}, {self.initial_world_pose_by_agent[2][1]:.3f}, {self.initial_world_pose_by_agent[2][2]:.3f}]"
        )
        d_form = self._formation_target_distance()
        self.get_logger().info(
            f"Safety/formation config: d_safe={self.cfg.d_safe:.3f} m, "
            f"formation_margin={self.cfg.formation_margin:.3f} m, "
            f"target_pair_distance={d_form:.3f} m, "
            f"d_agent_enter={self.cfg.d_agent_enter:.3f} m, "
            f"d_agent_exit={self.cfg.d_agent_exit:.3f} m, "
            f"safety_warning_radius={self.cfg.safety_warning_radius:.3f} m"
        )
        if not self.enable_motion:
            self.get_logger().warn("enable_motion=false: the node will compute commands but publish zero cmd_vel.")

    def _make_cfg(self) -> NetConfig:
        u_bound = float(self.get_parameter("u_bound").value)
        r_bound = float(self.get_parameter("r_bound").value)
        return replace(
            NetConfig(),
            n_agents=self.n_agents,
            model=str(self.get_parameter("model").value),
            graph=str(self.get_parameter("graph").value),
            objective_mode=str(self.get_parameter("objective_mode").value),
            auto_M=bool(self.get_parameter("auto_M").value),
            M_manual=int(self.get_parameter("M_manual").value),
            u_min=-u_bound,
            u_max=u_bound,
            u_mag=u_bound,
            r_min=-r_bound,
            r_max=r_bound,
            d_safe=float(self.get_parameter("d_safe").value),
            formation_margin=float(self.get_parameter("formation_margin").value),
            formation_rotation_rad=float(self.get_parameter("formation_rotation_rad").value),
            formation_radius_override=float(self.get_parameter("formation_radius_override").value),
            safety_warning_radius=float(self.get_parameter("safety_warning_radius").value),
            obstacle_warning_radius=float(self.get_parameter("obstacle_warning_radius").value),
            d_agent_enter=float(self.get_parameter("d_agent_enter").value),
            d_agent_exit=float(self.get_parameter("d_agent_exit").value),
            d_obs_enter=float(self.get_parameter("d_obs_enter").value),
            d_obs_exit=float(self.get_parameter("d_obs_exit").value),
            modeC_repulsion_gain_si=float(self.get_parameter("modeC_repulsion_gain_si").value),
            modeO_target_gain_si=float(self.get_parameter("modeO_target_gain_si").value),
            modeCO_repulsion_gain_si=float(self.get_parameter("modeCO_repulsion_gain_si").value),
            modeCO_target_gain_si=float(self.get_parameter("modeCO_target_gain_si").value),
            nominal_blend_C_si=float(self.get_parameter("nominal_blend_C_si").value),
            nominal_blend_O_si=float(self.get_parameter("nominal_blend_O_si").value),
            nominal_blend_CO_si=float(self.get_parameter("nominal_blend_CO_si").value),
            pair_filter_margin=float(self.get_parameter("pair_filter_margin").value),
            obs_filter_margin=float(self.get_parameter("obs_filter_margin").value),
            filter_projection_passes=int(self.get_parameter("filter_projection_passes").value),
            safety_enabled=bool(self.get_parameter("safety_enabled").value),
            obstacles_enabled=bool(self.get_parameter("obstacles_enabled").value),
            dt=float(self.get_parameter("dt").value),
            w_track=float(self.get_parameter("w_track").value),
            w_du=float(self.get_parameter("w_du").value),
            w_u=float(self.get_parameter("w_u").value),
        )

    def _make_req_socket(self, endpoint: str) -> zmq.Socket:
        sock = self.ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, self.cfg.req_linger_ms)
        sock.setsockopt(zmq.REQ_RELAXED, 1)
        sock.setsockopt(zmq.REQ_CORRELATE, 1)
        sock.RCVTIMEO = self.cfg.coord_controller_timeout_ms
        sock.SNDTIMEO = self.cfg.coord_controller_timeout_ms
        sock.connect(endpoint)
        return sock

    def _reset_socket(self, agent_id: int) -> None:
        try:
            self.req_socks[agent_id].close(0)
        except Exception:
            pass
        self.req_socks[agent_id] = self._make_req_socket(self.endpoints_by_agent[agent_id])

    def _make_odom_callback(self, agent_id: int):
        def _cb(msg: Odometry) -> None:
            self.odom_by_agent[agent_id] = msg
            self.odom_time_by_agent[agent_id] = self.get_clock().now()
        return _cb

    def _have_fresh_odom(self) -> bool:
        now = self.get_clock().now()
        for agent_id in self.agent_ids:
            if agent_id not in self.odom_by_agent:
                return False
            age = (now - self.odom_time_by_agent[agent_id]).nanoseconds * 1e-9
            if age > self.odom_timeout_s:
                self.get_logger().warn(f"Odometry timeout for agent {agent_id}: age={age:.2f}s", throttle_duration_sec=2.0)
                return False
        return True

    def _state_arrays_from_odom(self) -> tuple[np.ndarray, np.ndarray, Dict[int, float]]:
        r_all = np.zeros((self.n_agents, self.cfg.dim), dtype=float)
        v_all = np.zeros((self.n_agents, self.cfg.dim), dtype=float)
        yaw_by_agent: Dict[int, float] = {}

        for row, agent_id in enumerate(self.agent_ids):
            msg = self.odom_by_agent[agent_id]
            local_yaw = yaw_from_odom(msg)
            x_local = float(msg.pose.pose.position.x)
            y_local = float(msg.pose.pose.position.y)

            # Each Yahboom publishes odom in its own local frame starting near (0,0,0).
            # Transform that local odom pose into the shared world/map frame before
            # passing positions to the distributed MPC.
            x0, y0, yaw0 = self.initial_world_pose_by_agent.get(
                agent_id, np.array([0.0, 0.0, 0.0], dtype=float)
            )
            c0 = math.cos(float(yaw0))
            s0 = math.sin(float(yaw0))
            x_world = float(x0) + c0 * x_local - s0 * y_local
            y_world = float(y0) + s0 * x_local + c0 * y_local
            yaw_world = normalize_angle(float(yaw0) + local_yaw)

            yaw_by_agent[agent_id] = yaw_world
            r_all[row, 0] = x_world
            r_all[row, 1] = y_world

            # nav_msgs/Odometry twist is usually expressed in the robot body frame.
            # Rotate body-frame velocity directly into the shared world frame using yaw_world.
            vx_b = float(msg.twist.twist.linear.x)
            vy_b = float(msg.twist.twist.linear.y)
            cw = math.cos(yaw_world)
            sw = math.sin(yaw_world)
            v_all[row, 0] = vx_b * cw - vy_b * sw
            v_all[row, 1] = vx_b * sw + vy_b * cw

            # Publish the actual world/map pose that will be used by the MPC.
            # This makes dry-run verification unambiguous:
            #   /robotX/odom can still start at local (0,0,0),
            #   while /dmpc/robotX/pose_world shows the transformed common-frame pose.
            self._publish_world_pose_debug(agent_id, x_world, y_world, yaw_world)

        return r_all, v_all, yaw_by_agent

    def _request_controller(self, agent_id: int, msg_type: str, payload: dict) -> Optional[dict]:
        sock = self.req_socks[agent_id]
        try:
            sock.send(dumps(make_envelope(msg_type, payload, src="dmpc_coord", dst=f"ctrl{agent_id}")))
            reply = loads(sock.recv())
            return reply.get("payload", {})
        except zmq.Again:
            self.get_logger().error(f"Timeout talking to controller agent {agent_id} at {self.endpoints_by_agent[agent_id]}")
            self._reset_socket(agent_id)
            return None
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Controller communication error for agent {agent_id}: {exc}")
            self._reset_socket(agent_id)
            return None

    def _publish_zero_all(self) -> None:
        zero = Twist()
        for pub in self.cmd_publishers.values():
            pub.publish(zero)

    def _formation_target_distance(self) -> float:
        """Return the desired pairwise distance for the two-robot formation.

        For n_agents=2 and formation_radius_override=0, this is exactly
        d_safe + formation_margin. For other cases, it is the distance between
        the first two formation offsets.
        """
        C = self.cfg.formation_offsets()
        if C.shape[0] >= 2:
            return float(np.linalg.norm(C[0, :2] - C[1, :2]))
        return 0.0

    def _publish_two_robot_metrics(self, r_all: np.ndarray) -> None:
        """Publish simple scalar metrics for bag-based convergence evaluation.

        /dmpc/two_robot/metrics uses Vector3Stamped:
          x = current inter-robot distance [m]
          y = desired formation pair distance [m]
          z = safety margin = distance - d_safe [m]

        /dmpc/two_robot/safety_thresholds uses Vector3Stamped:
          x = d_safe [m]
          y = d_agent_enter [m]
          z = d_agent_exit [m]
        """
        if r_all.shape[0] < 2:
            return
        distance = float(np.linalg.norm(r_all[0, :2] - r_all[1, :2]))
        target = self._formation_target_distance()

        metrics = Vector3Stamped()
        metrics.header.stamp = self.get_clock().now().to_msg()
        metrics.header.frame_id = self.world_frame
        metrics.vector.x = distance
        metrics.vector.y = target
        metrics.vector.z = distance - float(self.cfg.d_safe)
        self.metrics_publisher.publish(metrics)

        thresholds = Vector3Stamped()
        thresholds.header.stamp = metrics.header.stamp
        thresholds.header.frame_id = self.world_frame
        thresholds.vector.x = float(self.cfg.d_safe)
        thresholds.vector.y = float(self.cfg.d_agent_enter)
        thresholds.vector.z = float(self.cfg.d_agent_exit)
        self.thresholds_publisher.publish(thresholds)

    def control_step(self) -> None:
        if not self._have_fresh_odom():
            self._publish_zero_all()
            return

        r_all, v_all, yaw_by_agent = self._state_arrays_from_odom()
        self._publish_two_robot_metrics(r_all)
        nbrs = self.cfg.neighbors(self.outer_index)

        nominal: Dict[int, np.ndarray] = {}
        bary_by_agent: Dict[int, np.ndarray] = {}
        ok = True

        for row, agent_id in enumerate(self.agent_ids):
            # NetConfig neighbors are 1-based agent ids.
            payload = {
                "outer_index": int(self.outer_index),
                "agent_id": int(agent_id),
                "r_i": r_all[row, :].tolist(),
                "v_i": v_all[row, :].tolist(),
                "r_neighbors": [r_all[self.agent_ids.index(k), :].tolist() for k in nbrs[agent_id] if k in self.agent_ids],
                "u_prev": self.u_prev[agent_id].tolist(),
                "neighbors": [k for k in nbrs[agent_id] if k in self.agent_ids],
            }
            reply = self._request_controller(agent_id, "mpc_request", payload)
            if not reply or not bool(reply.get("ok", False)):
                self.get_logger().error(f"MPC failed for agent {agent_id}: {reply}")
                ok = False
                break
            u_seq = np.array(reply["u_seq"], dtype=float)
            if u_seq.ndim != 2 or u_seq.shape[0] == 0:
                self.get_logger().error(f"Invalid u_seq for agent {agent_id}: shape={u_seq.shape}")
                ok = False
                break
            nominal[agent_id] = u_seq[0, :].copy()
            bary_by_agent[agent_id] = np.array(reply.get("bary_r", np.mean(r_all, axis=0)), dtype=float)

        if not ok:
            self._publish_zero_all()
            return

        safe_cmds: Dict[int, np.ndarray] = {}
        for row, agent_id in enumerate(self.agent_ids):
            payload = {
                "r_all": r_all.tolist(),
                "v_all": v_all.tolist(),
                "u_nom": nominal[agent_id].tolist(),
                "bary_r": bary_by_agent[agent_id].tolist(),
                "dt_inner": 1.0 / max(self.rate_hz, 1e-6),
            }
            reply = self._request_controller(agent_id, "hybrid_request", payload)
            if not reply or not bool(reply.get("ok", False)):
                self.get_logger().warn(f"Hybrid safety request failed for agent {agent_id}; using nominal command.")
                safe_cmds[agent_id] = nominal[agent_id]
            else:
                safe_cmds[agent_id] = np.array(reply["u_safe"], dtype=float)

        for agent_id in self.agent_ids:
            self.u_prev[agent_id] = safe_cmds[agent_id].copy()
            self._publish_world_command(agent_id, safe_cmds[agent_id], yaw_by_agent[agent_id])

        self.outer_index += 1

    def _publish_world_pose_debug(self, agent_id: int, x_world: float, y_world: float, yaw_world: float) -> None:
        """Publish the transformed world/map pose used internally by the MPC.

        This is a debug/verification topic. It lets the user confirm that local
        /robotX/odom values that start near (0,0,0) are actually transformed
        into the common world/map frame before the MPC receives r_all.
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.pose.position.x = float(x_world)
        msg.pose.position.y = float(y_world)
        msg.pose.position.z = 0.0
        z, w = quaternion_from_yaw(float(yaw_world))
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = float(z)
        msg.pose.orientation.w = float(w)
        self.pose_debug_publishers[agent_id].publish(msg)

    def _publish_world_command(self, agent_id: int, u_world: np.ndarray, yaw: float) -> None:
        debug = Vector3Stamped()
        debug.header.stamp = self.get_clock().now().to_msg()
        debug.header.frame_id = self.world_frame
        debug.vector.x = float(u_world[0])
        debug.vector.y = float(u_world[1])
        debug.vector.z = 0.0
        self.u_debug_publishers[agent_id].publish(debug)

        cmd = Twist()
        u_norm = float(np.linalg.norm(u_world[:2]))

        if u_norm > self.linear_deadband:
            desired_heading = math.atan2(float(u_world[1]), float(u_world[0]))
            heading_error = normalize_angle(desired_heading - yaw)
            angular = clamp(self.heading_gain * heading_error, -self.max_angular_speed, self.max_angular_speed)

            if abs(heading_error) > self.stop_for_heading_error_rad:
                linear = 0.0
            else:
                direction_factor = math.cos(heading_error)
                if not self.allow_reverse:
                    direction_factor = max(0.0, direction_factor)
                linear = self.world_command_to_speed_scale * u_norm * direction_factor
                linear = clamp(linear, -self.max_linear_speed if self.allow_reverse else 0.0, self.max_linear_speed)
            cmd.linear.x = float(linear)
            cmd.angular.z = float(angular)

        if not self.enable_motion:
            cmd = Twist()

        self.cmd_publishers[agent_id].publish(cmd)

    def destroy_node(self) -> bool:
        self._publish_zero_all()
        for sock in self.req_socks.values():
            try:
                sock.close(0)
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DmpcCoordinatorRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
