# consensus_controller.py
from __future__ import annotations
import time
import argparse
from typing import Any, Dict, List, Tuple

import cvxpy as cp
import numpy as np
import zmq

from .consensus_comm import dumps, loads, make_envelope
from .consensus_config import NetConfig


def _clamp_box(var, lo: float, hi: float) -> List[Any]:
    return [var >= lo, var <= hi]


# --------------------------
# Geometry helpers (2D only)
# --------------------------
def _cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])

def _diameter(points: np.ndarray) -> float:
    """Diameter of a finite set: max_{i,k} ||p_i - p_k||_2."""
    if points.ndim != 2 or points.shape[0] <= 1:
        return 0.0
    dmax = 0.0
    for i in range(points.shape[0]):
        for k in range(i + 1, points.shape[0]):
            d = float(np.linalg.norm(points[i] - points[k]))
            if d > dmax:
                dmax = d
    return float(dmax)


def _convex_hull_2d(points: np.ndarray) -> np.ndarray:
    """
    Monotone chain convex hull.
    points: (m,2)
    returns hull vertices in CCW order (k,2), without duplicating first vertex at end.
    Degenerate cases:
      - m==1 -> that point
      - collinear -> endpoints in order
    """
    pts = np.unique(points, axis=0)
    if pts.shape[0] <= 1:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def build_half(pts_arr):
        half = []
        for p in pts_arr:
            while len(half) >= 2:
                a = half[-2]
                b = half[-1]
                if _cross2(b - a, p - b) <= 1e-12:
                    half.pop()
                else:
                    break
            half.append(p)
        return half

    lower = build_half(pts)
    upper = build_half(pts[::-1])

    hull = np.array(lower[:-1] + upper[:-1], dtype=float)
    if hull.shape[0] == 0:
        return pts[:1]
    return hull


def _dist_point_to_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Euclidean distance from p to segment [a,b]."""
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-15:
        return float(np.linalg.norm(p - a))
    t = float(np.dot(p - a, ab) / denom)
    t = max(0.0, min(1.0, t))
    proj = a + t * ab
    return float(np.linalg.norm(p - proj))


def _phi_dist_to_boundary_conv2d(points: np.ndarray, x: np.ndarray) -> float:
    """
    phi(x) = dist(x, boundary of conv(points)) interpreted in R^2.
    For full-dim polygon: min distance to edges.
    For line segment: boundary is endpoints -> min distance to endpoints.
    For single point: 0.
    """
    hull = _convex_hull_2d(points)
    if hull.shape[0] <= 1:
        return 0.0
    if hull.shape[0] == 2:
        # Relative boundary of a segment in its affine hull are endpoints
        return float(min(np.linalg.norm(x - hull[0]), np.linalg.norm(x - hull[1])))

    # polygon edges
    dmin = float("inf")
    for i in range(hull.shape[0]):
        a = hull[i]
        b = hull[(i + 1) % hull.shape[0]]
        d = _dist_point_to_segment(x, a, b)
        if d < dmin:
            dmin = d
    return float(dmin)

def _prob_metrics(prob):
    """Return (n_var, n_eq, n_ineq) from a CVXPY Problem."""
    sm = prob.size_metrics
    n_var = int(getattr(sm, "num_scalar_variables", 0))
    n_eq = int(getattr(sm, "num_scalar_eq_constr", 0))
    n_ineq = int(getattr(sm, "num_scalar_leq_constr", 0))
    return n_var, n_eq, n_ineq


# --------------------------
# MPC Solvers
# --------------------------

def _solve_single_integrator(
    cfg: NetConfig,
    agent_id: int,
    r_i: np.ndarray,
    r_neighbors: List[np.ndarray],
    u_prev: np.ndarray,
    neighbor_ids: List[int] = None,
) -> Tuple[Dict[str, Any], bool]:
    """
    Condensed single-integrator MPC:
      r(k+1) = r(k) + u(k)
    Decision variables: u(0..M-1), a (terminal convex weights), (optional tmin in lex).
    Predicted states r(k) are affine expressions of u.

    When cfg.objective_mode == "safe_formation", the OCP is posed in
    shape-compensated coordinates y_i = r_i - c_i  (Section VIII of the
    manuscript).  The terminal hull constraint becomes
        r(M) - c_i  in  conv({y_i} ∪ {y_k : k in N_i})
    and the tracking target becomes the formation barycenter
        z_bar_i^c = mean_y + c_i.
    The control sequence u is identical in both coordinate systems
    because c_i is constant.
    """
    import time
    import cvxpy as cp
    import numpy as np

    M = cfg.horizon_M()
    d = cfg.dim

    # --- Formation coordinate transformation (Section VIII) -----------
    use_formation = (cfg.objective_mode == "safe_formation"
                     and neighbor_ids is not None
                     and len(neighbor_ids) == len(r_neighbors))
    if use_formation:
        offsets = cfg.formation_offsets()
        c_i = offsets[agent_id - 1]
        c_nbrs = [offsets[k - 1] for k in neighbor_ids]
        # Transformed coordinates y = r - c
        y_i = r_i - c_i
        y_neighbors = [r_k - c_k for r_k, c_k in zip(r_neighbors, c_nbrs)]
        # Hull is built in y-coordinates
        S = [y_i] + y_neighbors
        S_mat = np.stack(S, axis=0)  # (m, d)  -- y-coords
        bary_y = np.mean(S_mat, axis=0)
        # Formation barycenter in original coordinates (Eq. 34)
        bary_target = bary_y + c_i
    else:
        c_i = np.zeros(d, dtype=float)
        S = [r_i] + list(r_neighbors)
        S_mat = np.stack(S, axis=0)  # (m, d)  -- raw coords
        bary_target = np.mean(S_mat, axis=0)

    m = S_mat.shape[0]
    rhs0 = float(np.linalg.norm(r_i - bary_target))
    rhs = cfg.alpha_gamma * rhs0

    # Decision variables
    u = cp.Variable((M, d))
    a = cp.Variable((m,))
    tmin = cp.Variable()

    # Predicted states as expressions
    # r(0) fixed, r(k) = r_i + sum_{l=0}^{k-1} u(l)
    r_expr = []
    r_expr.append(cp.Constant(r_i))
    for k in range(1, M + 1):
        r_expr.append(cp.Constant(r_i) + cp.sum(u[:k, :], axis=0))

    cons: List[Any] = []

    # bounds for u and all predicted r(k)
    for k in range(M):
        cons += _clamp_box(u[k, :], cfg.u_min, cfg.u_max)
        cons += _clamp_box(r_expr[k], cfg.r_min, cfg.r_max)
    cons += _clamp_box(r_expr[M], cfg.r_min, cfg.r_max)

    # Terminal convex hull constraint
    # In formation mode: r(M) - c_i must be in conv(S_mat) where S_mat
    # holds y-coordinates.  Equivalently: r(M) = c_i + a @ S_mat.
    # In consensus mode: c_i == 0, so this reduces to r(M) = a @ S_mat.
    cons += [a >= 0.0, cp.sum(a) == 1.0]
    cons += [r_expr[M] == cp.Constant(c_i) + a @ S_mat]
    cons += [cp.norm(r_expr[M] - bary_target, 2) <= rhs]

    # Primary cost -- track formation barycenter (or consensus barycenter)
    J = 0
    for k in range(M):
        J += cfg.w_track * cp.sum_squares(r_expr[k] - bary_target)
        J += cfg.w_u * cp.sum_squares(u[k, :])
        if k == 0:
            J += cfg.w_du * cp.sum_squares(u[k, :] - u_prev)
        else:
            J += cfg.w_du * cp.sum_squares(u[k, :] - u[k - 1, :])

    prob1 = cp.Problem(cp.Minimize(J), cons)

    # size metrics + timing
    n_var_prim, n_eq_prim, n_ineq_prim = _prob_metrics(prob1)
    t0 = time.perf_counter()
    try:
        prob1.solve(solver=cp.ECOS, warm_start=True, verbose=False)
    except Exception:
        try:
            prob1.solve(solver=cp.SCS, warm_start=True, verbose=False)
        except Exception:
            return {}, False
    t_primary_ms = 1e3 * (time.perf_counter() - t0)

    if prob1.status not in ("optimal", "optimal_inaccurate"):
        return {}, False

    J_star = float(prob1.value)
    u_sol = np.array(u.value)
    a_sol = np.array(a.value).reshape(-1)

    # Build numeric predicted r for logging
    r_sol = np.zeros((M + 1, d), dtype=float)
    r_sol[0, :] = r_i
    for k in range(M):
        r_sol[k + 1, :] = r_sol[k, :] + u_sol[k, :]

    # compute phi from primary -- in formation mode, phi must be
    # evaluated in the *transformed* hull (y-coordinates)
    r_term_primary = r_sol[M, :].copy()
    if use_formation:
        r_term_primary_y = r_term_primary - c_i
        phi_primary = _phi_dist_to_boundary_conv2d(S_mat, r_term_primary_y) if cfg.dim == 2 else 0.0
    else:
        phi_primary = _phi_dist_to_boundary_conv2d(S_mat, r_term_primary) if cfg.dim == 2 else 0.0
    ri_primary = phi_primary > cfg.phi_tol

    diam_C = _diameter(S_mat)

    # Lex stage (optional)
    t_lex_ms = 0.0
    n_var_lex = n_eq_lex = n_ineq_lex = 0
    lex_used = False

    run_lex = bool(cfg.use_lexicographic)
    if run_lex and bool(getattr(cfg, "lex_only_if_phi_zero", False)):
        phi_ok = (phi_primary <= float(getattr(cfg, "phi_tol", 1e-9)))
        diam_ok = (diam_C > float(getattr(cfg, "diam_tol", 1e-6)))
        run_lex = bool(phi_ok and diam_ok)

    if run_lex:
        cons2 = list(cons)
        cons2 += [J <= J_star + cfg.lex_cost_tol]
        cons2 += [tmin >= 0.0, a >= tmin]
        prob2 = cp.Problem(cp.Maximize(tmin), cons2)

        n_var_lex, n_eq_lex, n_ineq_lex = _prob_metrics(prob2)
        t1 = time.perf_counter()
        try:
            prob2.solve(solver=cp.ECOS, warm_start=True, verbose=False)
        except Exception:
            try:
                prob2.solve(solver=cp.SCS, warm_start=True, verbose=False)
            except Exception:
                prob2 = None
        t_lex_ms = 1e3 * (time.perf_counter() - t1)

        if prob2 is not None and prob2.status in ("optimal", "optimal_inaccurate"):
            lex_used = True
            u_sol = np.array(u.value)
            a_sol = np.array(a.value).reshape(-1)

            # recompute r_sol with updated u
            r_sol[0, :] = r_i
            for k in range(M):
                r_sol[k + 1, :] = r_sol[k, :] + u_sol[k, :]

    r_term = r_sol[M, :].copy()
    if use_formation:
        r_term_y = r_term - c_i
        phi = _phi_dist_to_boundary_conv2d(S_mat, r_term_y) if cfg.dim == 2 else 0.0
    else:
        phi = _phi_dist_to_boundary_conv2d(S_mat, r_term) if cfg.dim == 2 else 0.0
    ri_flag = phi > cfg.phi_tol

    out = {
        "ok": True,
        "agent_id": int(agent_id),
        "model": cfg.model,
        "M": int(M),
        "bary_r": bary_target.tolist(),
        "r_pred": r_sol.tolist(),
        "u_seq": u_sol.tolist(),
        "a_weights": a_sol.tolist(),
        "lex_used": bool(lex_used),
        "objective_primary": float(J_star),
        "r_term": r_term.tolist(),
        "phi_terminal": float(phi),
        "ri_terminal": bool(ri_flag),
        "phi_primary": float(phi_primary),
        "ri_primary": bool(ri_primary),
        "diam_C": float(diam_C),
        "rhs0_norm_r0_minus_bary": float(rhs0),
        "rhs_norm_bound": float(rhs),

        # timing + complexity
        "t_primary_ms": float(t_primary_ms),
        "t_lex_ms": float(t_lex_ms),
        "t_total_ms": float(t_primary_ms + t_lex_ms),
        "n_var_primary": int(n_var_prim),
        "n_eq_primary": int(n_eq_prim),
        "n_ineq_primary": int(n_ineq_prim),
        "n_var_lex": int(n_var_lex),
        "n_eq_lex": int(n_eq_lex),
        "n_ineq_lex": int(n_ineq_lex),
    }
    return out, True


def _solve_double_integrator(
    cfg: NetConfig,
    agent_id: int,
    r_i: np.ndarray,
    v_i: np.ndarray,
    r_neighbors: List[np.ndarray],
    u_prev: np.ndarray,
    neighbor_ids: List[int] = None,
) -> Tuple[Dict[str, Any], bool]:
    """
    Condensed double-integrator MPC:
        r(k+1) = r(k) + v(k)
        v(k+1) = v(k) + u(k)
    Decision variables: u(0..M-1), a (terminal convex weights), (optional tmin in lex).
    Predicted states v(k), r(k) are affine expressions of u.

    When cfg.objective_mode == "safe_formation", the OCP is posed in
    shape-compensated coordinates y_i = r_i - c_i  (Section VIII).
    The terminal hull constraint becomes
        r(M) - c_i  in  conv({y_i} ∪ {y_k : k in N_i})
    and the tracking target becomes the formation barycenter
        z_bar_i^c = mean_y + c_i.
    """
    import time
    import cvxpy as cp
    import numpy as np

    M = cfg.horizon_M()
    d = cfg.dim

    # --- Formation coordinate transformation (Section VIII) -----------
    use_formation = (cfg.objective_mode == "safe_formation"
                     and neighbor_ids is not None
                     and len(neighbor_ids) == len(r_neighbors))
    if use_formation:
        offsets = cfg.formation_offsets()
        c_i = offsets[agent_id - 1]
        c_nbrs = [offsets[k - 1] for k in neighbor_ids]
        y_i = r_i - c_i
        y_neighbors = [r_k - c_k for r_k, c_k in zip(r_neighbors, c_nbrs)]
        Rset = [y_i] + y_neighbors
        R_mat = np.stack(Rset, axis=0)  # (m, d) -- y-coords
        bary_y = np.mean(R_mat, axis=0)
        bary_r = bary_y + c_i  # formation barycenter in original coords
    else:
        c_i = np.zeros(d, dtype=float)
        Rset = [r_i] + list(r_neighbors)
        R_mat = np.stack(Rset, axis=0)
        bary_r = np.mean(R_mat, axis=0)

    m = R_mat.shape[0]
    rhs0 = float(np.linalg.norm(r_i - bary_r))
    rhs = cfg.alpha_gamma * rhs0

    # Decision variables
    u = cp.Variable((M, d))
    a = cp.Variable((m,))
    tmin = cp.Variable()

    # Predicted velocities and positions as expressions
    # v(0) = v_i
    # v(k) = v_i + sum_{l=0}^{k-1} u(l), for k>=1
    v_expr = [cp.Constant(v_i)]
    for k in range(1, M + 1):
        v_expr.append(cp.Constant(v_i) + cp.sum(u[:k, :], axis=0))

    # r(0) = r_i
    # r(k) = r_i + sum_{s=0}^{k-1} v(s)
    #      = r_i + k*v_i + sum_{l=0}^{k-2} (k-1-l) u(l)    (for k>=1)
    r_expr = [cp.Constant(r_i)]
    for k in range(1, M + 1):
        # sum of past velocities v(0..k-1)
        # Using v_expr avoids manual coefficients; it's still affine.
        r_expr.append(cp.Constant(r_i) + cp.sum(cp.vstack(v_expr[:k]), axis=0))

    cons: List[Any] = []

    # bounds for u and all predicted r(k), v(k)
    for k in range(M):
        cons += _clamp_box(u[k, :], cfg.u_min, cfg.u_max)
        cons += _clamp_box(r_expr[k], cfg.r_min, cfg.r_max)
        cons += _clamp_box(v_expr[k], cfg.v_min, cfg.v_max)
    cons += _clamp_box(r_expr[M], cfg.r_min, cfg.r_max)
    cons += _clamp_box(v_expr[M], cfg.v_min, cfg.v_max)

    # Terminal convex hull constraint in position
    # Formation mode: r(M) - c_i in conv(R_mat), i.e. r(M) = c_i + a @ R_mat
    # Consensus mode:  c_i == 0, reduces to r(M) = a @ R_mat
    cons += [a >= 0.0, cp.sum(a) == 1.0]
    cons += [r_expr[M] == cp.Constant(c_i) + a @ R_mat]
    cons += [cp.norm(r_expr[M] - bary_r, 2) <= rhs]

    # Primary cost: track bary in position + regularize v + smooth u
    J = 0
    for k in range(M):
        J += cfg.w_track * cp.sum_squares(r_expr[k] - bary_r)
        J += cfg.w_v * cp.sum_squares(v_expr[k])
        J += cfg.w_u * cp.sum_squares(u[k, :])
        if k == 0:
            J += cfg.w_du * cp.sum_squares(u[k, :] - u_prev)
        else:
            J += cfg.w_du * cp.sum_squares(u[k, :] - u[k - 1, :])

    prob1 = cp.Problem(cp.Minimize(J), cons)

    # size metrics + timing
    n_var_prim, n_eq_prim, n_ineq_prim = _prob_metrics(prob1)
    t0 = time.perf_counter()
    try:
        prob1.solve(solver=cp.ECOS, warm_start=True, verbose=False)
    except Exception:
        try:
            prob1.solve(solver=cp.SCS, warm_start=True, verbose=False)
        except Exception:
            return {}, False
    t_primary_ms = 1e3 * (time.perf_counter() - t0)

    if prob1.status not in ("optimal", "optimal_inaccurate"):
        return {}, False

    J_star = float(prob1.value)
    u_sol = np.array(u.value)
    a_sol = np.array(a.value).reshape(-1)

    # Build numeric predicted r, v for logging
    v_sol = np.zeros((M + 1, d), dtype=float)
    r_sol = np.zeros((M + 1, d), dtype=float)
    v_sol[0, :] = v_i
    r_sol[0, :] = r_i
    for k in range(M):
        v_sol[k + 1, :] = v_sol[k, :] + u_sol[k, :]
        r_sol[k + 1, :] = r_sol[k, :] + v_sol[k, :]

    # compute phi from primary terminal point in position hull
    # In formation mode, phi is evaluated in the transformed hull
    r_term_primary = r_sol[M, :].copy()
    if use_formation:
        r_term_primary_y = r_term_primary - c_i
        phi_primary = _phi_dist_to_boundary_conv2d(R_mat, r_term_primary_y) if cfg.dim == 2 else 0.0
    else:
        phi_primary = _phi_dist_to_boundary_conv2d(R_mat, r_term_primary) if cfg.dim == 2 else 0.0
    ri_primary = phi_primary > cfg.phi_tol

    diam_C = _diameter(R_mat)

    # Lex stage (optional)
    t_lex_ms = 0.0
    n_var_lex = n_eq_lex = n_ineq_lex = 0
    lex_used = False

    run_lex = bool(cfg.use_lexicographic)
    if run_lex and bool(getattr(cfg, "lex_only_if_phi_zero", False)):
        phi_ok = (phi_primary <= float(getattr(cfg, "phi_tol", 1e-9)))
        diam_ok = (diam_C > float(getattr(cfg, "diam_tol", 1e-6)))
        run_lex = bool(phi_ok and diam_ok)

    if run_lex:
        cons2 = list(cons)
        cons2 += [J <= J_star + cfg.lex_cost_tol]
        cons2 += [tmin >= 0.0, a >= tmin]
        prob2 = cp.Problem(cp.Maximize(tmin), cons2)

        n_var_lex, n_eq_lex, n_ineq_lex = _prob_metrics(prob2)
        t1 = time.perf_counter()
        try:
            prob2.solve(solver=cp.ECOS, warm_start=True, verbose=False)
        except Exception:
            try:
                prob2.solve(solver=cp.SCS, warm_start=True, verbose=False)
            except Exception:
                prob2 = None
        t_lex_ms = 1e3 * (time.perf_counter() - t1)

        if prob2 is not None and prob2.status in ("optimal", "optimal_inaccurate"):
            lex_used = True
            u_sol = np.array(u.value)
            a_sol = np.array(a.value).reshape(-1)

            # recompute r_sol, v_sol with updated u
            v_sol[0, :] = v_i
            r_sol[0, :] = r_i
            for k in range(M):
                v_sol[k + 1, :] = v_sol[k, :] + u_sol[k, :]
                r_sol[k + 1, :] = r_sol[k, :] + v_sol[k, :]

    r_term = r_sol[M, :].copy()
    if use_formation:
        r_term_y = r_term - c_i
        phi = _phi_dist_to_boundary_conv2d(R_mat, r_term_y) if cfg.dim == 2 else 0.0
    else:
        phi = _phi_dist_to_boundary_conv2d(R_mat, r_term) if cfg.dim == 2 else 0.0
    ri_flag = phi > cfg.phi_tol

    out = {
        "ok": True,
        "agent_id": int(agent_id),
        "model": cfg.model,
        "M": int(M),
        "bary_r": bary_r.tolist(),
        "r_pred": r_sol.tolist(),
        "v_pred": v_sol.tolist(),
        "u_seq": u_sol.tolist(),
        "a_weights": a_sol.tolist(),
        "lex_used": bool(lex_used),
        "objective_primary": float(J_star),
        "r_term": r_term.tolist(),
        "phi_terminal": float(phi),
        "ri_terminal": bool(ri_flag),
        "phi_primary": float(phi_primary),
        "ri_primary": bool(ri_primary),
        "diam_C": float(diam_C),
        "rhs0_norm_r0_minus_bary": float(rhs0),
        "rhs_norm_bound": float(rhs),

        # timing + complexity
        "t_primary_ms": float(t_primary_ms),
        "t_lex_ms": float(t_lex_ms),
        "t_total_ms": float(t_primary_ms + t_lex_ms),
        "n_var_primary": int(n_var_prim),
        "n_eq_primary": int(n_eq_prim),
        "n_ineq_primary": int(n_ineq_prim),
        "n_var_lex": int(n_var_lex),
        "n_eq_lex": int(n_eq_lex),
        "n_ineq_lex": int(n_ineq_lex),
    }
    return out, True


def solve_mpc_request(cfg: NetConfig, payload: Dict[str, Any]) -> Tuple[Dict[str, Any], bool]:
    """
    Compatibility wrapper used by controller_node.py.

    The hybrid debugged app still imports solve_mpc_request from this module.
    Some edited versions accidentally removed this public wrapper and kept only
    the internal _solve_single_integrator / _solve_double_integrator helpers,
    which triggers ImportError in controller_node.py.
    """
    agent_id = int(payload["agent_id"])
    r_i = np.array(payload["r_i"], dtype=float)
    v_i = np.array(payload.get("v_i", np.zeros((cfg.dim,), dtype=float)), dtype=float)
    r_neighbors = [np.array(x, dtype=float) for x in payload.get("r_neighbors", [])]
    u_prev = np.array(payload.get("u_prev", np.zeros((cfg.dim,), dtype=float)), dtype=float)
    neighbor_ids = [int(k) for k in payload.get("neighbors", [])]

    if cfg.model == "single_integrator":
        return _solve_single_integrator(cfg, agent_id, r_i, r_neighbors, u_prev,
                                        neighbor_ids=neighbor_ids)
    return _solve_double_integrator(cfg, agent_id, r_i, v_i, r_neighbors, u_prev,
                                    neighbor_ids=neighbor_ids)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, required=True, help="Agent id (1..n)")
    args = parser.parse_args()

    cfg = NetConfig()
    agent_id = int(args.id)
    endpoint = cfg.controller_endpoint(agent_id)

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REP)
    sock.bind(endpoint)
    print(f"[Controller {agent_id}] REP bound at {endpoint} | model={cfg.model} | M={cfg.horizon_M()}")

    while True:
        msg = loads(sock.recv())
        mtype = msg.get("type")

        if mtype == "shutdown":
            sock.send(dumps(make_envelope("shutdown_ack", {"ok": True}, src=f"ctrl{agent_id}")))
            break

        if mtype != "mpc_request":
            sock.send(dumps(make_envelope("error", {"error": "unknown msg type"}, src=f"ctrl{agent_id}")))
            continue

        p = msg["payload"]
        u_prev = np.array(p["u_prev"], dtype=float)
        r_neighbors = [np.array(v, dtype=float) for v in p["r_neighbors"]]
        neighbor_ids = [int(k) for k in p.get("neighbors", [])]

        if cfg.model == "single_integrator":
            r_i = np.array(p["r_i"], dtype=float)
            out, ok = _solve_single_integrator(cfg, agent_id, r_i, r_neighbors, u_prev,
                                               neighbor_ids=neighbor_ids)
        else:
            r_i = np.array(p["r_i"], dtype=float)
            v_i = np.array(p["v_i"], dtype=float)
            out, ok = _solve_double_integrator(cfg, agent_id, r_i, v_i, r_neighbors, u_prev,
                                              neighbor_ids=neighbor_ids)

        if not ok:
            sock.send(
                dumps(
                    make_envelope(
                        "mpc_reply",
                        {"ok": False, "agent_id": agent_id, "reason": "infeasible_or_solver_failed"},
                        src=f"ctrl{agent_id}",
                    )
                )
            )
        else:
            sock.send(dumps(make_envelope("mpc_reply", out, src=f"ctrl{agent_id}")))


if __name__ == "__main__":
    main()