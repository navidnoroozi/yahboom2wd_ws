from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .consensus_config import NetConfig


MODE_F = "F"
MODE_C = "C"
MODE_O = "O"
MODE_CO = "CO"
MODE_T = "T"


@dataclass
class HybridResult:
    u_safe: np.ndarray
    diag: Dict[str, Any]


class ExplicitHybridController:
    """
    Practical safety supervisor that keeps the nominal DMPC controller unchanged.

    Design choice
    -------------
    The nominal safe-formation / consensus DMPC law and the communication architecture stay as-is.
    This module only replaces the transient safety layer. The mode names F/C/O/CO/T are retained for
    logging and visualization, but the actual control law is a supervisor consisting of:

    1) mode detection with hysteresis,
    2) a mode-dependent reference command built from the nominal DMPC input plus conservative local
       separating / tangent / damping terms,
    3) a projection-based local safety filter,
    4) an emergency fallback used only if the filtered command still fails the local checks.

    In particular, for the double-integrator case we do *not* rely on transition blending anymore.
    The previous implementation showed that the T-blend was the main source of small residual safety
    dips. Here, T is kept only as an informational label; the controller commits directly to the new
    safe command.
    """

    def __init__(self, cfg: NetConfig, agent_id: int):
        self.cfg = cfg
        self.agent_id = int(agent_id)
        self.idx = self.agent_id - 1
        self.mode = MODE_F
        self.last_u = np.zeros((cfg.dim,), dtype=float)
        self._turn_memory: Dict[int, float] = {}
        self._hold_counter = 0
        self._pair_active = False
        self._obs_active = False

    @staticmethod
    def _clip(u: np.ndarray, lo: float, hi: float) -> np.ndarray:
        return np.minimum(np.maximum(u, lo), hi)

    @staticmethod
    def _norm(x: np.ndarray) -> float:
        return float(np.linalg.norm(x))

    @staticmethod
    def _safe_unit(x: np.ndarray, fallback: Optional[np.ndarray] = None) -> np.ndarray:
        n = float(np.linalg.norm(x))
        if n <= 1e-12:
            if fallback is None:
                fallback = np.array([1.0, 0.0], dtype=float)
            return np.array(fallback, dtype=float)
        return np.array(x, dtype=float) / n

    @staticmethod
    def _project_halfspace(u: np.ndarray, a: np.ndarray, b: float) -> np.ndarray:
        """Project u onto {x : a^T x >= b}."""
        den = float(np.dot(a, a))
        if den <= 1e-12:
            return u
        val = float(np.dot(a, u))
        if val >= b:
            return u
        return u + ((b - val) / den) * a

    def _inflated_obstacles(self) -> List[Tuple[float, float, float]]:
        if not self.cfg.obstacles_enabled:
            return []
        return [
            (float(cx), float(cy), float(rad) + float(self.cfg.obstacle_margin))
            for (cx, cy, rad) in self.cfg.obstacles_circles
        ]

    def _formation_target(self, bary_r: np.ndarray) -> np.ndarray:
        """Return the per-agent target position.

        After the Section-VIII coordinate-transformation fix in
        consensus_controller.py, the MPC already returns the correct
        formation reference  z_bar_i^c = bar_y_i + c_i  as ``bary_r``
        in formation mode, and the plain consensus barycenter in
        consensus mode.  So we simply pass it through.
        """
        return np.array(bary_r, dtype=float)

    def _pairwise_distances(self, r_all: np.ndarray, v_all: np.ndarray) -> List[Dict[str, Any]]:
        ri = r_all[self.idx]
        vi = v_all[self.idx]
        out: List[Dict[str, Any]] = []
        for j in range(r_all.shape[0]):
            if j == self.idx:
                continue
            rel = ri - r_all[j]
            dist = self._norm(rel)
            n = self._safe_unit(rel, fallback=np.array([1.0, 0.0], dtype=float))
            rel_v = vi - v_all[j]
            closing = max(0.0, -float(np.dot(rel_v, n)))
            out.append(
                {
                    "j": int(j),
                    "rel": rel,
                    "dist": float(dist),
                    "normal": n,
                    "clearance": float(dist - self.cfg.d_safe),
                    "rel_v": rel_v,
                    "closing": float(closing),
                }
            )
        out.sort(key=lambda item: item["dist"])
        return out

    def _obstacle_distances(self, r_i: np.ndarray, v_i: np.ndarray) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for obs_idx, (cx, cy, rad_eff) in enumerate(self._inflated_obstacles()):
            center = np.array([cx, cy], dtype=float)
            rel = r_i[:2] - center
            n = self._safe_unit(rel, fallback=np.array([1.0, 0.0], dtype=float))
            dist_center = self._norm(rel)
            clearance = float(dist_center - rad_eff)
            v2 = v_i[:2] if v_i.size >= 2 else np.zeros((2,), dtype=float)
            closing = max(0.0, -float(np.dot(v2, n)))
            out.append(
                {
                    "obs_idx": int(obs_idx),
                    "center": center,
                    "radius": float(rad_eff),
                    "dist_center": float(dist_center),
                    "clearance": float(clearance),
                    "normal": n,
                    "closing": float(closing),
                }
            )
        out.sort(key=lambda item: item["clearance"])
        return out

    def _dynamic_pair_enter_distance(self, pair: Dict[str, Any]) -> float:
        base = float(self.cfg.d_agent_enter)
        closing = float(pair["closing"])
        tau = float(getattr(self.cfg, "dynamic_pair_time_margin", 0.7))
        a_eff = max(float(getattr(self.cfg, "dynamic_pair_brake_accel", 1.6)), 1e-6)
        extra = tau * closing + 0.5 * closing * closing / a_eff
        return min(float(self.cfg.safety_warning_radius), base + extra)

    def _dynamic_obs_enter_distance(self, obs: Dict[str, Any]) -> float:
        base = float(self.cfg.d_obs_enter)
        closing = float(obs["closing"])
        tau = float(getattr(self.cfg, "dynamic_obs_time_margin", 0.8))
        a_eff = max(float(getattr(self.cfg, "dynamic_obs_brake_accel", 1.8)), 1e-6)
        extra = tau * closing + 0.5 * closing * closing / a_eff
        return min(float(self.cfg.obstacle_warning_radius), base + extra)

    def _segment_intersects_circle(self, a: np.ndarray, b: np.ndarray, circle: Tuple[float, float, float]) -> bool:
        c = np.array([circle[0], circle[1]], dtype=float)
        R = float(circle[2])
        a2 = a[:2]
        b2 = b[:2]
        ab = b2 - a2
        denom = float(np.dot(ab, ab))
        if denom <= 1e-12:
            return float(np.linalg.norm(a2 - c)) <= R
        t = float(np.dot(c - a2, ab) / denom)
        t = max(0.0, min(1.0, t))
        proj = a2 + t * ab
        return float(np.linalg.norm(proj - c)) <= R

    def _path_obstructed(self, r_i: np.ndarray, target: np.ndarray) -> tuple[bool, Optional[int]]:
        for obs_idx, (cx, cy, rad_eff) in enumerate(self._inflated_obstacles()):
            if self._segment_intersects_circle(r_i, target, (cx, cy, rad_eff + 0.05)):
                return True, int(obs_idx)
        return False, None

    def _obstacle_tangent_waypoint(
        self,
        r_i: np.ndarray,
        target: np.ndarray,
        obs: Dict[str, Any],
    ) -> np.ndarray:
        center = obs["center"]
        n = obs["normal"]
        tangent_left = np.array([-n[1], n[0]], dtype=float)
        tangent_right = -tangent_left

        pref_left = float(np.dot(target[:2] - r_i[:2], tangent_left))
        pref_right = float(np.dot(target[:2] - r_i[:2], tangent_right))

        mem = self._turn_memory.get(int(obs["obs_idx"]), 0.0)
        if abs(mem) > 1e-12:
            sgn = np.sign(mem)
        else:
            sgn = 1.0 if pref_left >= pref_right else -1.0
        self._turn_memory[int(obs["obs_idx"])] = sgn

        tangent = tangent_left if sgn >= 0.0 else tangent_right
        orbit_radius = obs["radius"] + float(getattr(self.cfg, "tangential_waypoint_radius", 1.30))
        waypoint = center + orbit_radius * n + float(getattr(self.cfg, "orbit_tangent_lookahead", 0.9)) * tangent
        return np.array(waypoint, dtype=float)

    def _formation_hold(self, r_all: np.ndarray, v_all: np.ndarray, bary_r: np.ndarray) -> Optional[np.ndarray]:
        r_i = r_all[self.idx]
        v_i = v_all[self.idx]
        target = self._formation_target(bary_r)
        pos_err = target - r_i
        if self.cfg.model == "single_integrator":
            if self._norm(pos_err) <= float(getattr(self.cfg, "formation_hold_pos_tol", 0.25)):
                return float(getattr(self.cfg, "formation_hold_kp_si", 0.90)) * pos_err
            return None

        vel_tol = float(getattr(self.cfg, "formation_hold_vel_tol", 0.45))
        pos_tol = float(getattr(self.cfg, "formation_hold_pos_tol", 0.22))
        if self._norm(pos_err) <= pos_tol and self._norm(v_i) <= vel_tol:
            self._hold_counter += 1
        else:
            self._hold_counter = 0

        if self._hold_counter >= int(getattr(self.cfg, "formation_hold_steps", 3)):
            kp = float(getattr(self.cfg, "formation_hold_kp_di", 1.00))
            kd = float(getattr(self.cfg, "formation_hold_kd_di", 1.80))
            return kp * pos_err - kd * v_i
        return None

    def _desired_mode(
        self,
        r_all: np.ndarray,
        v_all: np.ndarray,
        bary_r: np.ndarray,
        pair_data: List[Dict[str, Any]],
        obstacle_data: List[Dict[str, Any]],
    ) -> tuple[str, float, float, List[Dict[str, Any]], List[Dict[str, Any]], bool]:
        r_i = r_all[self.idx]
        target = self._formation_target(bary_r)
        obstructed, obs_idx_from_path = self._path_obstructed(r_i, target)

        d_agent_min = pair_data[0]["dist"] if pair_data else float("inf")
        d_obs_min = obstacle_data[0]["clearance"] if obstacle_data else float("inf")

        active_C = self._pair_active
        for pair in pair_data:
            if pair["dist"] <= self._dynamic_pair_enter_distance(pair):
                active_C = True
                break
        if active_C and d_agent_min >= float(self.cfg.d_agent_exit):
            active_C = False

        active_O = self._obs_active
        for obs in obstacle_data:
            if obs["clearance"] <= self._dynamic_obs_enter_distance(obs):
                active_O = True
                break
        if obstructed:
            active_O = True
            if obs_idx_from_path is not None and not any(obs["obs_idx"] == obs_idx_from_path for obs in obstacle_data):
                for obs in self._obstacle_distances(r_i, v_all[self.idx]):
                    if obs["obs_idx"] == obs_idx_from_path:
                        obstacle_data = [obs] + obstacle_data
                        d_obs_min = min(d_obs_min, obs["clearance"])
                        break
        if active_O and d_obs_min >= float(self.cfg.d_obs_exit) and not obstructed:
            active_O = False

        self._pair_active = active_C
        self._obs_active = active_O

        if active_C and active_O:
            desired = MODE_CO
        elif active_C:
            desired = MODE_C
        elif active_O:
            desired = MODE_O
        else:
            desired = MODE_F
        return desired, float(d_agent_min), float(d_obs_min), pair_data, obstacle_data, bool(obstructed)

    def _mode_reference(
        self,
        mode: str,
        r_all: np.ndarray,
        v_all: np.ndarray,
        u_nom: np.ndarray,
        bary_r: np.ndarray,
        pair_data: List[Dict[str, Any]],
        obstacle_data: List[Dict[str, Any]],
    ) -> tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Build a mode-appropriate reference command for the safety filter to track.
        For the single-integrator case, we rely more heavily on the nominal command and simply add repulsive corrections.
        For the double-integrator case, we blend less with the nominal command and add a stronger direct formation stabilization term, while the repulsive corrections are similar.
        Inputs:
        - mode: the current mode, which is used to determine which terms to include in the reference.
        - r_all, v_all: the current positions and velocities of all agents, used for computing the pairwise safety terms and the formation target.
        - u_nom: the nominal DMPC command, used as a baseline reference that we deviate from more aggressively in the double-integrator case.
        - bary_r: the current formation barycenter, used for computing the formation target.
        - pair_data, obstacle_data: the precomputed pairwise and obstacle safety data, used for computing the repulsive corrections.
        Outputs:
        - u_ref: the reference control input that the safety filter will try to track, built from the nominal command plus mode-appropriate corrections.
        - target_waypoint: an optional reference waypoint used for visualization, which is set to a tangential waypoint around the nearest obstacle if in obstacle-avoidance mode.
        """
        r_i = r_all[self.idx]
        v_i = v_all[self.idx]
        target = self._formation_target(bary_r)
        hold_u = self._formation_hold(r_all, v_all, bary_r)
        target_waypoint: Optional[np.ndarray] = None

        # Pair separation field.
        sep = np.zeros((self.cfg.dim,), dtype=float)
        for pair in pair_data:
            if pair["dist"] > float(self.cfg.safety_warning_radius):
                continue
            gap = max(pair["dist"] - float(self.cfg.d_safe), 1e-3)
            weight = min(1.8, 1.0 / gap)
            sep += (1.0 + 0.8 * pair["closing"]) * weight * pair["normal"]

        # Obstacle guidance field.
        obs_push = np.zeros((self.cfg.dim,), dtype=float)
        obs_tangent = np.zeros((self.cfg.dim,), dtype=float)
        if obstacle_data:
            obs = obstacle_data[0]
            wp = self._obstacle_tangent_waypoint(r_i, target, obs)
            target_waypoint = wp
            obs_push[:2] = (1.2 + obs["closing"]) * max(0.0, float(self.cfg.obstacle_warning_radius) - obs["clearance"]) * obs["normal"]
            obs_tangent[:2] = wp - r_i[:2]

        if self.cfg.model == "single_integrator":
            u_ref = np.array(u_nom, dtype=float)
            if hold_u is not None and mode == MODE_F:
                u_ref = float(getattr(self.cfg, "formation_hold_blend", 1.0)) * hold_u
            elif mode == MODE_C:
                u_ref = 0.35 * u_nom + float(getattr(self.cfg, "modeC_repulsion_gain_si", 0.90)) * sep
            elif mode == MODE_O:
                u_ref = 0.15 * u_nom + 0.70 * obs_push + 0.75 * obs_tangent
            elif mode == MODE_CO:
                u_ref = 0.10 * u_nom + 0.80 * obs_push + 0.70 * obs_tangent + 0.70 * sep
            return self._clip(u_ref, self.cfg.u_min, self.cfg.u_max), target_waypoint

        # Double integrator: prefer damping and direct formation stabilization.
        kp_nom = float(getattr(self.cfg, "di_nominal_kp", 0.55))
        kd_nom = float(getattr(self.cfg, "di_nominal_kd", 1.25))
        u_track = kp_nom * (target - r_i) - kd_nom * v_i
        u_ref = 0.55 * np.array(u_nom, dtype=float) + 0.45 * u_track

        if hold_u is not None and mode == MODE_F:
            u_ref = hold_u
        elif mode == MODE_C:
            u_ref = 0.15 * u_nom + 0.35 * u_track + float(getattr(self.cfg, "modeC_repulsion_gain_di", 1.20)) * sep - float(getattr(self.cfg, "modeC_damping_di", 1.40)) * v_i
        elif mode == MODE_O:
            u_ref = 0.10 * u_nom + 0.35 * u_track + 0.90 * obs_push + float(getattr(self.cfg, "modeO_tangent_gain_di", 0.85)) * obs_tangent - float(getattr(self.cfg, "modeO_kd_di", 1.10)) * v_i
        elif mode == MODE_CO:
            u_ref = 0.08 * u_nom + 0.30 * u_track + 0.95 * obs_push + float(getattr(self.cfg, "modeO_tangent_gain_di", 0.85)) * obs_tangent + 1.00 * sep - float(getattr(self.cfg, "modeCO_kd_di", 1.25)) * v_i

        return self._clip(u_ref, self.cfg.u_min, self.cfg.u_max), target_waypoint

    def _check_admissible(
        self,
        u: np.ndarray,
        r_all: np.ndarray,
        v_all: np.ndarray,
        pair_data: List[Dict[str, Any]],
        obstacle_data: List[Dict[str, Any]],
        dt_inner: float
    ) -> bool:
        dt = dt_inner
        if self.cfg.model == "single_integrator":
            r_i_next = r_all[self.idx] + dt * u
            v_i_next = np.zeros_like(v_all[self.idx])
        else:
            r_i_next = r_all[self.idx] + dt * v_all[self.idx] + 0.5 * (dt * dt) * u
            v_i_next = v_all[self.idx] + dt * u

        margin_pair = float(getattr(self.cfg, "transition_pair_margin", 0.03))
        margin_obs = float(getattr(self.cfg, "transition_obs_margin", 0.03))
        for pair in pair_data:
            j = pair["j"]
            if self.cfg.model == "single_integrator":
                r_j_next = r_all[j]
            else:
                r_j_next = r_all[j] + dt * v_all[j]
            if self._norm(r_i_next - r_j_next) < float(self.cfg.d_safe) + margin_pair:
                return False

        for obs in obstacle_data:
            if self._norm(r_i_next[:2] - obs["center"]) < obs["radius"] + margin_obs:
                return False

        if self.cfg.model == "double_integrator":
            speed_lim = max(abs(float(self.cfg.v_min)), abs(float(self.cfg.v_max)))
            if self._norm(v_i_next) > speed_lim + 1e-9:
                return False
        return True

    def _qp_filter(
        self,
        u_ref: np.ndarray,
        r_all: np.ndarray,
        v_all: np.ndarray,
        pair_data: List[Dict[str, Any]],
        obstacle_data: List[Dict[str, Any]],
        desired_mode: str,
        dt_inner: float,
    ) -> tuple[np.ndarray, int, int]:
        """
        Iterative projection safety filter.
        For the double-integrator case, a one-step position check alone is too weak. 
        We therefore add conservative multi-step radial constraints using the predicted positions under a constant
        acceleration over the first step and constant velocity afterwards. 
        This is not a theorem-level proof device; it is a practical simulation safeguard.
        """
        dt = dt_inner
        u = self._clip(np.array(u_ref, dtype=float), self.cfg.u_min, self.cfg.u_max)
        pair_count = 0
        obs_count = 0

        pair_margin_si = float(getattr(self.cfg, "pair_filter_margin", 0.04))
        pair_margin_di = float(getattr(self.cfg, "pair_filter_margin_di", 0.10))
        obs_margin_si = float(getattr(self.cfg, "obs_filter_margin", 0.05))
        obs_margin_di = float(getattr(self.cfg, "obs_filter_margin_di", 0.12))

        for _ in range(int(getattr(self.cfg, "filter_projection_passes", 4))):
            for pair in pair_data:
                active = pair["dist"] <= self._dynamic_pair_enter_distance(pair) or desired_mode in {MODE_C, MODE_CO}
                if not active:
                    continue
                n = pair["normal"]
                if self.cfg.model == "single_integrator":
                    base = float(np.dot(n, pair["rel"]))
                    a = dt * n
                    b = (float(self.cfg.d_safe) + pair_margin_si) - base
                    u = self._project_halfspace(u, a, b)
                else:
                    # step 1 predicted separation
                    base1 = float(np.dot(n, pair["rel"] + dt * pair["rel_v"]))
                    a1 = 0.5 * (dt * dt) * n
                    b1 = (float(self.cfg.d_safe) + pair_margin_di) - base1
                    u = self._project_halfspace(u, a1, b1)

                    # step 2 predicted separation, assuming agent i keeps the new velocity for one more step
                    base2 = float(np.dot(n, pair["rel"] + 2.0 * dt * pair["rel_v"]))
                    a2 = 1.5 * (dt * dt) * n
                    b2 = (float(self.cfg.d_safe) + pair_margin_di + float(getattr(self.cfg, "pair_h2_margin_di", 0.06))) - base2
                    u = self._project_halfspace(u, a2, b2)

                    # radial relative velocity should not be strongly inward
                    base_v = float(np.dot(n, pair["rel_v"]))
                    a_v = dt * n
                    b_v = -float(getattr(self.cfg, "pair_relvel_slack_di", 0.02)) - base_v
                    u = self._project_halfspace(u, a_v, b_v)
                pair_count += 1

            for obs in obstacle_data:
                active = obs["clearance"] <= self._dynamic_obs_enter_distance(obs) or desired_mode in {MODE_O, MODE_CO}
                if not active:
                    continue
                n2 = obs["normal"]
                if self.cfg.model == "single_integrator":
                    base = float(np.dot(n2, r_all[self.idx][:2] - obs["center"]))
                    a = dt * n2
                    b = (obs["radius"] + obs_margin_si) - base
                    u2 = self._project_halfspace(np.array(u[:2], dtype=float), a, b)
                    u[:2] = u2
                else:
                    base1 = float(np.dot(n2, r_all[self.idx][:2] - obs["center"] + dt * v_all[self.idx][:2]))
                    a1 = 0.5 * (dt * dt) * n2
                    b1 = (obs["radius"] + obs_margin_di) - base1
                    u2 = self._project_halfspace(np.array(u[:2], dtype=float), a1, b1)

                    base2 = float(np.dot(n2, r_all[self.idx][:2] - obs["center"] + 2.0 * dt * v_all[self.idx][:2]))
                    a2 = 1.5 * (dt * dt) * n2
                    b2 = (obs["radius"] + obs_margin_di + float(getattr(self.cfg, "obs_h2_margin_di", 0.08))) - base2
                    u2 = self._project_halfspace(u2, a2, b2)

                    base_v = float(np.dot(n2, v_all[self.idx][:2]))
                    a_v = dt * n2
                    b_v = -float(getattr(self.cfg, "obs_relvel_slack_di", 0.02)) - base_v
                    u2 = self._project_halfspace(u2, a_v, b_v)
                    u[:2] = u2
                obs_count += 1

            # mild dead-zone damping in DI to reduce repeated compress/back-off cycles
            if self.cfg.model == "double_integrator":
                u -= float(getattr(self.cfg, "di_residual_damping", 0.18)) * v_all[self.idx]
            u = self._clip(u, self.cfg.u_min, self.cfg.u_max)

        return u, pair_count, obs_count

    def _emergency_fallback(
        self,
        r_all: np.ndarray,
        v_all: np.ndarray,
        pair_data: List[Dict[str, Any]],
        obstacle_data: List[Dict[str, Any]],
        bary_r: np.ndarray,
    ) -> np.ndarray:
        u = np.zeros((self.cfg.dim,), dtype=float)
        v_i = v_all[self.idx]
        r_i = r_all[self.idx]

        if pair_data:
            crit_pair = min(pair_data, key=lambda item: item["clearance"])
            u += float(getattr(self.cfg, "emergency_pair_gain", 1.8 + crit_pair["closing"])) * crit_pair["normal"]

        if obstacle_data:
            crit_obs = min(obstacle_data, key=lambda item: item["clearance"])
            outward = np.zeros((self.cfg.dim,), dtype=float)
            outward[:2] = crit_obs["normal"]
            u += float(getattr(self.cfg, "emergency_obs_gain", 2.2 + crit_obs["closing"])) * outward
            wp = self._obstacle_tangent_waypoint(r_i, self._formation_target(bary_r), crit_obs)
            tvec = np.zeros((self.cfg.dim,), dtype=float)
            tvec[:2] = wp - r_i[:2]
            u += float(getattr(self.cfg, "emergency_tangent_gain", 0.75)) * tvec

        if self.cfg.model == "double_integrator":
            u -= float(getattr(self.cfg, "emergency_damping_di", 2.1)) * v_i
        else:
            u += 0.2 * (self._formation_target(bary_r) - r_i)

        return self._clip(u, self.cfg.u_min, self.cfg.u_max)

    def step(self, payload: Dict[str, Any]) -> HybridResult:
        """
        Main entry point for the controller.
        The payload is expected to contain the following keys:
        - "r_all": list of all agent positions, shape (N, dim)
        - "v_all": list of all agent velocities, shape (N, dim)
        - "u_nom": nominal control input from the DMPC layer, shape (dim,)
        - "bary_r": barycenter of all agent positions, shape (dim,)
        outputs a HybridResult containing:
        - u_safe: the safe control input after filtering, shape (dim,)
        - diag: a dictionary of diagnostic information for logging and visualization
        """
        r_all = np.array(payload["r_all"], dtype=float)
        v_all = np.array(payload["v_all"], dtype=float)
        u_nom = np.array(payload["u_nom"], dtype=float)
        bary_r = np.array(payload["bary_r"], dtype=float)
        # Extract the exact inner step size for local safety projections
        dt_inner = float(payload.get("dt_inner", self.cfg.dt / self.cfg.horizon_M()))

        pair_data = self._pairwise_distances(r_all, v_all)
        obstacle_data = self._obstacle_distances(r_all[self.idx], v_all[self.idx])
        desired, d_agent_min, d_obs_min, pair_data, obstacle_data, obstructed = self._desired_mode(
            r_all, v_all, bary_r, pair_data, obstacle_data
        )

        # Keep T only as a one-sample informational tag on actual mode changes for visualization.
        previous_mode = self.mode
        effective_mode = desired
        self.mode = MODE_T if desired != previous_mode and previous_mode != MODE_T else desired

        u_ref, target_waypoint = self._mode_reference(
            desired,
            r_all,
            v_all,
            u_nom,
            bary_r,
            pair_data,
            obstacle_data,
        )
        u_safe, h_pair_num, circle_barrier_num = self._qp_filter(
            u_ref, r_all, v_all, pair_data, obstacle_data, desired_mode=desired, dt_inner=dt_inner
        )

        if not self._check_admissible(u_safe, r_all, v_all, pair_data, obstacle_data, dt_inner=dt_inner):
            u_safe = self._emergency_fallback(r_all, v_all, pair_data, obstacle_data, bary_r)
            u_safe, _, _ = self._qp_filter(u_safe, r_all, v_all, pair_data, obstacle_data, desired_mode=MODE_CO, dt_inner=dt_inner)

        # Commit to the desired mode after control computation.
        self.mode = desired
        self.last_u = np.array(u_safe, dtype=float)

        diag = {
            "mode": MODE_T if desired != previous_mode and previous_mode != MODE_T else self.mode,
            "desired_mode": desired,
            "effective_mode": effective_mode,
            "d_agent_min": float(d_agent_min),
            "d_obs_min": float(d_obs_min),
            "active_pairs": int(sum(1 for pair in pair_data if pair["dist"] <= float(self.cfg.safety_warning_radius))),
            "active_obstacles": int(sum(1 for obs in obstacle_data if obs["clearance"] <= float(self.cfg.obstacle_warning_radius))),
            "obstructed": bool(obstructed),
            "target_waypoint": target_waypoint.tolist() if target_waypoint is not None else [np.nan, np.nan],
            "_h_pair_num": int(h_pair_num),
            "_circle_barrier_num": int(circle_barrier_num),
            "formation_hold_active": bool(self._formation_hold(r_all, v_all, bary_r) is not None),
        }
        return HybridResult(u_safe=u_safe, diag=diag)
