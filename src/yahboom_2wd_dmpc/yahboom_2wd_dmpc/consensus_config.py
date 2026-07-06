from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from typing import Dict, List, Literal, Optional, Tuple

import numpy as np

ModelType = Literal["single_integrator", "double_integrator"]


def _pairwise_diameter(X: np.ndarray) -> float:
    if X.ndim != 2 or X.shape[0] == 0:
        return 0.0
    dmax = 0.0
    for i in range(X.shape[0]):
        for k in range(i + 1, X.shape[0]):
            d = float(np.linalg.norm(X[i] - X[k]))
            if d > dmax:
                dmax = d
    return dmax


def _str2bool(v: str | bool) -> bool:
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    if s in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if s in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {v}")


@dataclass(frozen=True)
class NetConfig:
    # --- Network / model ---
    n_agents: int = 4
    dim: int = 2
    model: ModelType = "single_integrator"

    # --- DMPC ---
    outer_steps: int = 300
    auto_M: bool = True
    M_manual: int = 3
    alpha_gamma: float = 0.9

    # --- Constraints ---
    r_min: float = -10.0
    r_max: float = 10.0
    u_min: float = -2.0    # <-- Changed from -2.0
    u_max: float = 2.0     # <-- Changed from 2.0
    u_mag: float = 2.0     # <-- Changed from 2.0
    v_min: float = -20.0    # <-- Changed from -20.0
    v_max: float = 20.0     # <-- Changed from 20.0

    # --- Costs ---
    w_track: float = 10.0   # <-- Reduced from 10.0 to prevent aggressive pulling
    w_du: float = 1.0      # <-- Increased from 1.0 to heavily penalize jerky movements
    w_u: float = 0.1
    w_v: float = 0.10

    # --- Lexicographic selection ---
    use_lexicographic: bool = True
    lex_cost_tol: float = 1e-6
    phi_tol: float = 1e-19
    diam_tol: float = 1e-15
    lex_only_if_phi_zero: bool = True

    # --- ZMQ ---
    plant_to_coord_rep: str = "tcp://127.0.0.1:5555"
    ctrl_base_port: int = 5600
    graph: Literal["ring_timevarying", "complete"] = "ring_timevarying"

    startup_delay_s: float = 2.0
    req_timeout_ms: int = 12000
    coord_controller_timeout_ms: int = 15000
    hybrid_timeout_ms: int = 5000
    req_linger_ms: int = 0

    # --- Initial conditions ---
    r0_single: Tuple[Tuple[float, float], ...] = (
        (-2.0, 7.0),
        (7.5, 4.0),
        (4.5, -4.5),
        (-3.5, -4.0),
    )
    r0_double: Tuple[Tuple[float, float], ...] = (
        (-4.0, 4.0),
        (3.5, 4.0),
        (4.5, -4.5),
        (-3.5, -4.0),
    )
    v0_double: Tuple[Tuple[float, float], ...] = (
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
        (0.0, 0.0),
    )

    # --- Objective ---
    objective_mode: Literal["consensus", "safe_formation"] = "safe_formation"
    d_safe: float = 1.10
    formation_margin: float = 0.15
    formation_rotation_rad: float = (
        0.0  # rotation of the formation in radians, applied to the formation offsets
    )
    formation_radius_override: float = 0.0

    # --- Explicit hybrid controller (Solution 2) ---
    safety_enabled: bool = True
    safety_method: str = "explicit_hybrid"

    dt: float = 0.05

    # local safety graph / local sensing radii
    safety_warning_radius: float = (
        1.90  # should be > d_safe to allow for some warning before a safety violation occurs, used for both agent-agent and agent-obstacle safety
    )
    obstacle_warning_radius: float = (
        1.80  # should be > d_safe to allow for some warning before a safety violation occurs, used for agent-obstacle safety, i.e., should be < tangential_waypoint_radius to allow for tangential waypoint generation before a safety violation occurs
    )

    # mode hysteresis thresholds
    # Threshold–formation compatibility (Assumption in Section IX-H):
    #   d_agent_exit < min_{i≠k} ||c_i - c_k||
    # With corrected offsets and n=4, min dist = d_safe + formation_margin = 1.05,
    # so d_agent_exit must be < 1.05.
    d_agent_enter: float = 0.95
    d_agent_exit: float = 1.00
    d_obs_enter: float = 0.45
    d_obs_exit: float = 0.85

    # transition
    transition_steps: int = 1  # kept only for visualization; the practical safety supervisor commits directly to the safe command
    transition_lambda_start: float = 0.35  # starting value of the transition lambda parameter used for blending between modes C, O, and CO when using the explicit hybrid safety method, should be in the range [0, 1]
    transition_lambda_end: float = 1.00  # ending value of the transition lambda parameter used for blending between modes C, O, and CO when using the explicit hybrid safety method, should be in the range [0, 1] and >= transition_lambda_start

    # obstacle model: circles (cx, cy, radius)
    obstacles_enabled: bool = True
    obstacles_circles: Tuple[Tuple[float, float, float], ...] = (
        (0.0, 1.0, 1.90),
    )  # (x_o, y_o, r_o), x_o is the x-coordinate of the center, y_o is the y-coordinate of the center, and r_o is the radius of the obstacle; these parameters can be adjusted to add more obstacles or change their positions and sizes
    obstacle_margin: float = 0.20

    # single-integrator gains
    # The repulsion gain determines how strongly the agents are repelled from each other and from obstacles when they are within the safety warning radius.
    # A higher repulsion gain will result in stronger repulsive forces, which can help prevent collisions but may also lead to more aggressive maneuvers.
    # The target gain determines how strongly the agents are attracted to their desired positions or formation offsets. 
    # A higher target gain will result in stronger attractive forces, which can help the agents converge to their desired positions more quickly but may also lead to overshooting or oscillations if set too high. The nominal blend parameters determine how much weight is given to the different modes (C, O, CO) in the control law when blending between them. Adjusting these gains and blend parameters can help achieve a balance between safety and convergence performance in the multi-agent system.
    modeC_repulsion_gain_si: float = 0.90
    # The modeC_repulsion_gain_si parameter is a gain that determines the strength of the repulsive force between agents in the single integrator model when they are in mode C (collision avoidance mode). 
    # A higher value for this gain will result in stronger repulsive forces, which can help prevent collisions between agents. 
    # However, setting this gain too high may lead to overly aggressive maneuvers and instability in the system.
    # It is important to tune this gain appropriately based on the specific dynamics of the agents and the desired safety performance.
    modeO_target_gain_si: float = 0.95
    modeCO_repulsion_gain_si: float = 0.90
    modeCO_target_gain_si: float = 0.85
    # The modeO_target_gain_si parameter is a gain that determines the strength of the attractive force towards the target position in the single integrator model when the agents are in mode O (objective mode).
    # A higher value for this gain will result in stronger attractive forces, 
    # which can help the agents converge to their desired positions more quickly. However, setting this gain too high may lead to overshooting or oscillations, especially if the agents are also experiencing repulsive forces from other agents or obstacles. It is important to tune this gain carefully to achieve a balance between convergence speed and stability in the system.
    nominal_blend_C_si: float = 0.35
    nominal_blend_O_si: float = 0.20
    nominal_blend_CO_si: float = 0.15

    # double-integrator gains
    modeC_repulsion_gain_di: float = 1.00
    modeC_damping_di: float = 1.10
    modeO_kp_di: float = 0.55
    modeO_kd_di: float = 0.90
    modeCO_kp_di: float = 0.50
    modeCO_kd_di: float = 1.00
    nominal_blend_C_di: float = 0.30
    nominal_blend_O_di: float = 0.18
    nominal_blend_CO_di: float = 0.12

    ## tangential waypoint radius around inflated obstacle ##
    # What does mathematically the radius of the tangential waypoint around the inflated obstacle mean?
    # The tangential waypoint radius around the inflated obstacle is a parameter that defines the distance from the center of an obstacle at which a tangential waypoint is generated for the agents to navigate around the obstacle.
    # Mathematically, if we consider an obstacle as a circle with a certain radius (inflated by the obstacle margin), the tangential waypoint would be located on a circle centered at the obstacle's center with a radius equal to the sum of the obstacle's radius and the tangential waypoint radius.
    # This means that the tangential waypoint is generated at a distance from the obstacle that allows the agents to safely navigate around it while maintaining a certain clearance, which is determined by the tangential waypoint radius. The choice of this radius should be such that it is greater than the obstacle warning radius plus the obstacle margin to ensure that the agents have enough space to maneuver around the obstacle without violating safety constraints.
    tangential_waypoint_radius: float = (1.30) # should be > obstacle_warning_radius + obstacle_margin

    # practical safety-filter tuning
    dynamic_pair_time_margin: float = 0.75
    dynamic_pair_brake_accel: float = 1.70
    dynamic_obs_time_margin: float = 0.85
    dynamic_obs_brake_accel: float = 1.90

    pair_filter_margin: float = 0.04
    obs_filter_margin: float = 0.05
    pair_filter_margin_di: float = 0.10
    obs_filter_margin_di: float = 0.12
    pair_relvel_slack_di: float = 0.02
    obs_relvel_slack_di: float = 0.02
    filter_projection_passes: int = 4

    transition_pair_margin: float = 0.03
    transition_obs_margin: float = 0.03

    formation_hold_pos_tol: float = 0.22
    formation_hold_vel_tol: float = 0.45
    formation_hold_kp_si: float = 0.90
    formation_hold_kp_di: float = 1.00
    formation_hold_kd_di: float = 1.80
    formation_hold_blend: float = 1.00
    formation_restore_gain_di: float = 0.25

    emergency_pair_gain: float = 1.80
    emergency_obs_gain: float = 2.20
    emergency_tangent_gain: float = 0.75
    emergency_damping_di: float = 2.10
    formation_hold_steps: int = 3
    di_nominal_kp: float = 0.55
    di_nominal_kd: float = 1.25
    di_residual_damping: float = 0.18
    pair_h2_margin_di: float = 0.06
    obs_h2_margin_di: float = 0.08
    modeO_tangent_gain_di: float = 0.85
    orbit_tangent_lookahead: float = 0.90

    def controller_endpoint(self, agent_id_1based: int) -> str:
        return f"tcp://127.0.0.1:{self.ctrl_base_port + agent_id_1based}"

    def neighbors(self, outer_j: int) -> Dict[int, List[int]]:
        n = self.n_agents

        def ring_neighbors() -> Dict[int, List[int]]:
            nbrs: Dict[int, List[int]] = {}
            for i in range(1, n + 1):
                left = i - 1 if i > 1 else n
                right = i + 1 if i < n else 1
                nbrs[i] = [left, right]
            return nbrs

        if self.graph == "complete":
            return {i: [k for k in range(1, n + 1) if k != i] for i in range(1, n + 1)}

        nbrs = ring_neighbors()
        if outer_j % 5 == 0 and n >= 3 and 3 in nbrs[2]:
            nbrs[2] = [k for k in nbrs[2] if k != 3]
        return nbrs

    def initial_positions(self) -> np.ndarray:
        if self.model == "single_integrator":
            return np.array(self.r0_single, dtype=float)
        return np.array(self.r0_double, dtype=float)

    def initial_velocities(self) -> np.ndarray:
        if self.model == "double_integrator":
            return np.array(self.v0_double, dtype=float)
        return np.zeros((self.n_agents, self.dim), dtype=float)

    def V0_single(self) -> float:
        """Computes the initial position diameter for the single integrator model."""
        return _pairwise_diameter(self.initial_positions())

    def Vr0_double(self) -> float:
        """Computes the initial position diameter for the double integrator model."""
        return _pairwise_diameter(self.initial_positions())

    def horizon_M(self) -> int:
        """
        Computes the DMPC horizon M based on the initial conditions and control input magnitude.
        If auto_M is False, returns M_manual.
        Otherwise, computes M based on the model type and initial conditions.
        For single_integrator, M is computed as the ceiling of the ratio of the initial position diameter to the control input magnitude.
        For double_integrator, M is computed as the sum of two terms:
            - term1: the ceiling of the ratio of twice the maximum initial velocity to the control input magnitude.
            - term2: the ceiling of the square root of the ratio of the initial position diameter to the control input magnitude.
        The final M is the maximum of 2 and the sum of term1 and twice term2 for the double_integrator case, ensuring that M is at least 2.
        """
        if not self.auto_M:
            return int(self.M_manual)

        umin = float(self.u_mag)
        if umin <= 0:
            return int(self.M_manual)

        if self.model == "single_integrator":
            M = int(np.ceil(self.V0_single() / umin))
            return max(1, M)

        term1 = int(np.ceil((2.0 * float(self.v_max)) / umin))
        term2 = int(np.ceil(np.sqrt(max(self.Vr0_double(), 0.0) / umin)))
        return int(max(2, term1 + 2 * term2))

    def formation_offsets(self) -> np.ndarray:
        """
        This functiion mathematically computes the desired formation offsets for the safe_formation objective mode, based on the number of agents, dimension, safety distance, and formation margin.
        The formation is a regular polygon (or a line segment if n_agents=2) centered at the origin, with radius determined by the safety distance (d_safe) and formation margin (formation_margin).
        For instance, if n_agents=4 and dim=2, the formation would be a square centered at the origin, with each agent located at a corner of the square.
        The distance from the center to each corner (the radius) is calculated to ensure that the agents maintain a safe distance from each other while also fitting within the specified formation margin.
        """
        n = self.n_agents
        d = self.dim
        if n <= 1:
            return np.zeros((n, d), dtype=float)

        denom = 2.0 * np.sin(np.pi / float(n))
        auto_radius = (self.d_safe + self.formation_margin) / max(denom, 1e-9)
        radius = max(auto_radius, float(self.formation_radius_override))

        angles = self.formation_rotation_rad + 2.0 * np.pi * np.arange(n) / float(n)
        C = np.zeros((n, d), dtype=float)
        C[:, 0] = radius * np.cos(angles)
        if d >= 2:
            C[:, 1] = radius * np.sin(angles)
        C = C - np.mean(C, axis=0, keepdims=True)
        return C


def add_common_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument("--n-agents", type=int, default=None)
    parser.add_argument(
        "--model", choices=["single_integrator", "double_integrator"], default=None
    )
    parser.add_argument("--outer-steps", type=int, default=None)
    parser.add_argument("--use-lexicographic", type=_str2bool, default=None)
    parser.add_argument("--safety-enabled", type=_str2bool, default=None)
    parser.add_argument("--safety-method", default=None)
    parser.add_argument("--obstacles-enabled", type=_str2bool, default=None)
    parser.add_argument("--agent-id", type=int, default=None)
    parser.add_argument("--startup-delay-s", type=float, default=None)
    return parser


def config_from_namespace(ns: argparse.Namespace) -> NetConfig:
    cfg = NetConfig()
    updates = {}
    for key in [
        "n_agents",
        "model",
        "outer_steps",
        "use_lexicographic",
        "startup_delay_s",
    ]:
        v = getattr(ns, key, None)
        if v is not None:
            updates[key] = v

    v = getattr(ns, "safety_enabled", None)
    if v is not None:
        updates["safety_enabled"] = bool(v)

    sm = getattr(ns, "safety_method", None)
    if sm is not None:
        updates["safety_method"] = str(sm)

    oe = getattr(ns, "obstacles_enabled", None)
    if oe is not None:
        updates["obstacles_enabled"] = bool(oe)

    return replace(cfg, **updates)


def parse_config_args(
    argv: Optional[list[str]] = None,
) -> tuple[NetConfig, argparse.Namespace]:
    parser = add_common_args(argparse.ArgumentParser())
    ns = parser.parse_args(argv)
    return config_from_namespace(ns), ns
