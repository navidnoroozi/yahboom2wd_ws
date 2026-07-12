from __future__ import annotations

import argparse
from dataclasses import replace
from typing import Any

from .consensus_config import NetConfig


def str2bool(v: str | bool) -> bool:
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    if s in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if s in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {v}")


def add_yahboom_dmpc_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Common non-ROS arguments used by the ZMQ controller node.

    These parameters intentionally default to conservative two-robot values.
    They can be overridden from a ROS 2 launch file or from the command line.
    """
    parser.add_argument("--n-agents", type=int, default=2)
    parser.add_argument("--model", choices=["single_integrator", "double_integrator"], default="single_integrator")
    parser.add_argument("--graph", choices=["complete", "ring_timevarying"], default="complete")
    parser.add_argument("--objective-mode", choices=["consensus", "safe_formation"], default="safe_formation")

    parser.add_argument("--auto-M", type=str2bool, default=False)
    parser.add_argument("--M-manual", type=int, default=5)
    parser.add_argument("--alpha-gamma", type=float, default=0.9)

    parser.add_argument("--u-bound", type=float, default=0.08)
    parser.add_argument("--r-bound", type=float, default=5.0)
    parser.add_argument("--dt", type=float, default=0.20)

    parser.add_argument("--d-safe", type=float, default=0.65)
    parser.add_argument("--formation-margin", type=float, default=0.15)
    parser.add_argument("--formation-rotation-rad", type=float, default=0.0)
    parser.add_argument("--formation-radius-override", type=float, default=0.0)

    # Explicit-hybrid safety thresholds and hysteresis.  For two robots with
    # d_safe=0.65 and formation_margin=0.15, the desired formation distance is
    # 0.80 m, so d_agent_exit should stay below 0.80 m.
    parser.add_argument("--safety-warning-radius", type=float, default=1.20)
    parser.add_argument("--obstacle-warning-radius", type=float, default=0.25)
    parser.add_argument("--d-agent-enter", type=float, default=0.70)
    parser.add_argument("--d-agent-exit", type=float, default=0.75)
    parser.add_argument("--d-obs-enter", type=float, default=0.10)
    parser.add_argument("--d-obs-exit", type=float, default=0.20)

    # Single-integrator explicit-hybrid gains.
    parser.add_argument("--modeC-repulsion-gain-si", type=float, default=0.90)
    parser.add_argument("--modeO-target-gain-si", type=float, default=0.95)
    parser.add_argument("--modeCO-repulsion-gain-si", type=float, default=0.90)
    parser.add_argument("--modeCO-target-gain-si", type=float, default=0.85)
    parser.add_argument("--nominal-blend-C-si", type=float, default=0.35)
    parser.add_argument("--nominal-blend-O-si", type=float, default=0.20)
    parser.add_argument("--nominal-blend-CO-si", type=float, default=0.15)

    # Practical safety-filter margins.
    parser.add_argument("--pair-filter-margin", type=float, default=0.04)
    parser.add_argument("--obs-filter-margin", type=float, default=0.05)
    parser.add_argument("--filter-projection-passes", type=int, default=4)

    parser.add_argument("--safety-enabled", type=str2bool, default=True)
    parser.add_argument("--safety-method", default="explicit_hybrid")
    parser.add_argument("--obstacles-enabled", type=str2bool, default=False)
    parser.add_argument("--obstacle-center-x", type=float, default=1.0)
    parser.add_argument("--obstacle-center-y", type=float, default=-0.33)
    parser.add_argument("--obstacle-radius", type=float, default=0.15)
    parser.add_argument("--obstacle-margin", type=float, default=0.10)
    parser.add_argument("--tangential-waypoint-radius", type=float, default=0.12)
    parser.add_argument("--orbit-tangent-lookahead", type=float, default=0.20)

    parser.add_argument("--w-track", type=float, default=8.0)
    parser.add_argument("--w-du", type=float, default=3.0)
    parser.add_argument("--w-u", type=float, default=0.3)

    parser.add_argument("--ctrl-base-port", type=int, default=5600)
    parser.add_argument("--req-timeout-ms", type=int, default=12000)
    parser.add_argument("--coord-controller-timeout-ms", type=int, default=15000)
    parser.add_argument("--req-linger-ms", type=int, default=0)
    return parser


def cfg_from_args(args: argparse.Namespace) -> NetConfig:
    u_bound = float(getattr(args, "u_bound", 0.08))
    r_bound = float(getattr(args, "r_bound", 5.0))

    return replace(
        NetConfig(),
        n_agents=int(getattr(args, "n_agents", 2)),
        model=str(getattr(args, "model", "single_integrator")),
        graph=str(getattr(args, "graph", "complete")),
        objective_mode=str(getattr(args, "objective_mode", "safe_formation")),
        auto_M=bool(getattr(args, "auto_M", False)),
        M_manual=int(getattr(args, "M_manual", 5)),
        alpha_gamma=float(getattr(args, "alpha_gamma", 0.9)),
        u_min=-u_bound,
        u_max=u_bound,
        u_mag=u_bound,
        r_min=-r_bound,
        r_max=r_bound,
        dt=float(getattr(args, "dt", 0.20)),
        d_safe=float(getattr(args, "d_safe", 0.65)),
        formation_margin=float(getattr(args, "formation_margin", 0.15)),
        formation_rotation_rad=float(getattr(args, "formation_rotation_rad", 0.0)),
        formation_radius_override=float(getattr(args, "formation_radius_override", 0.0)),
        safety_warning_radius=float(getattr(args, "safety_warning_radius", 1.20)),
        obstacle_warning_radius=float(getattr(args, "obstacle_warning_radius", 0.25)),
        d_agent_enter=float(getattr(args, "d_agent_enter", 0.70)),
        d_agent_exit=float(getattr(args, "d_agent_exit", 0.75)),
        d_obs_enter=float(getattr(args, "d_obs_enter", 0.10)),
        d_obs_exit=float(getattr(args, "d_obs_exit", 0.20)),
        modeC_repulsion_gain_si=float(getattr(args, "modeC_repulsion_gain_si", 0.90)),
        modeO_target_gain_si=float(getattr(args, "modeO_target_gain_si", 0.95)),
        modeCO_repulsion_gain_si=float(getattr(args, "modeCO_repulsion_gain_si", 0.90)),
        modeCO_target_gain_si=float(getattr(args, "modeCO_target_gain_si", 0.85)),
        nominal_blend_C_si=float(getattr(args, "nominal_blend_C_si", 0.35)),
        nominal_blend_O_si=float(getattr(args, "nominal_blend_O_si", 0.20)),
        nominal_blend_CO_si=float(getattr(args, "nominal_blend_CO_si", 0.15)),
        pair_filter_margin=float(getattr(args, "pair_filter_margin", 0.04)),
        obs_filter_margin=float(getattr(args, "obs_filter_margin", 0.05)),
        filter_projection_passes=int(getattr(args, "filter_projection_passes", 4)),
        safety_enabled=bool(getattr(args, "safety_enabled", True)),
        safety_method=str(getattr(args, "safety_method", "explicit_hybrid")),
        obstacles_enabled=bool(getattr(args, "obstacles_enabled", False)),
        obstacles_circles=((
            float(getattr(args, "obstacle_center_x", 1.0)),
            float(getattr(args, "obstacle_center_y", -0.33)),
            float(getattr(args, "obstacle_radius", 0.15)),
        ),),
        obstacle_margin=float(getattr(args, "obstacle_margin", 0.10)),
        tangential_waypoint_radius=float(getattr(args, "tangential_waypoint_radius", 0.12)),
        orbit_tangent_lookahead=float(getattr(args, "orbit_tangent_lookahead", 0.20)),
        w_track=float(getattr(args, "w_track", 8.0)),
        w_du=float(getattr(args, "w_du", 3.0)),
        w_u=float(getattr(args, "w_u", 0.3)),
        ctrl_base_port=int(getattr(args, "ctrl_base_port", 5600)),
        req_timeout_ms=int(getattr(args, "req_timeout_ms", 12000)),
        coord_controller_timeout_ms=int(getattr(args, "coord_controller_timeout_ms", 15000)),
        req_linger_ms=int(getattr(args, "req_linger_ms", 0)),
    )
