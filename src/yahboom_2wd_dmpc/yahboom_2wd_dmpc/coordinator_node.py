from __future__ import annotations

import argparse
from typing import Any, Dict
import logging

import numpy as np
import zmq

from .consensus_comm import dumps, loads, make_envelope
from .consensus_config import add_common_args, config_from_namespace


logging.basicConfig(level=logging.INFO)


def _make_req_socket(ctx: zmq.Context, endpoint: str, timeout_ms: int, linger_ms: int) -> zmq.Socket:
    s = ctx.socket(zmq.REQ)
    s.setsockopt(zmq.LINGER, linger_ms)
    s.setsockopt(zmq.REQ_RELAXED, 1)
    s.setsockopt(zmq.REQ_CORRELATE, 1)
    s.RCVTIMEO = timeout_ms
    s.SNDTIMEO = timeout_ms
    s.connect(endpoint)
    return s


def _reset_socket(ctx: zmq.Context, sock: zmq.Socket, endpoint: str, timeout_ms: int, linger_ms: int) -> zmq.Socket:
    try:
        sock.close(0)
    except Exception:
        pass
    return _make_req_socket(ctx, endpoint, timeout_ms, linger_ms)


def main() -> None:
    parser = add_common_args(argparse.ArgumentParser())
    ns = parser.parse_args()
    cfg = config_from_namespace(ns)

    M = cfg.horizon_M()
    ctx = zmq.Context.instance()

    rep = ctx.socket(zmq.REP)
    rep.setsockopt(zmq.LINGER, cfg.req_linger_ms)
    rep.bind(cfg.plant_to_coord_rep)
    rep.RCVTIMEO = cfg.req_timeout_ms
    rep.SNDTIMEO = cfg.req_timeout_ms
    print(f"[Coordinator] REP bound at {cfg.plant_to_coord_rep} | model={cfg.model} | M={M}")

    req_socks: Dict[int, zmq.Socket] = {}
    endpoints: Dict[int, str] = {}
    for i in range(1, cfg.n_agents + 1):
        ep = cfg.controller_endpoint(i)
        endpoints[i] = ep
        req_socks[i] = _make_req_socket(ctx, ep, cfg.coord_controller_timeout_ms, cfg.req_linger_ms)
        print(f"[Coordinator] REQ -> controller_node{i} at {ep}")

    u_prev = {i: np.zeros((cfg.dim,), dtype=float) for i in range(1, cfg.n_agents + 1)}

    while True:
        try:
            msg = loads(rep.recv())
        except zmq.Again:
            continue

        mtype = msg.get("type")

        if mtype == "shutdown":
            print("[Coordinator] Shutdown requested by plant.")
            for i, s in req_socks.items():
                try:
                    s.send(dumps(make_envelope("shutdown", {}, src="coord", dst=f"ctrl{i}")))
                    _ = loads(s.recv())
                except Exception:
                    pass
            rep.send(dumps(make_envelope("shutdown_ack", {"ok": True}, src="coord")))
            break

        if mtype != "plant_step":
            rep.send(dumps(make_envelope("error", {"error": "unknown msg type"}, src="coord")))
            continue

        p = msg["payload"]
        outer_j = int(p["outer_index"])
        r_all = np.array(p["r_all"], dtype=float)
        v_all = np.array(p["v_all"], dtype=float)

        nbrs = cfg.neighbors(outer_j)
        replies: Dict[int, Dict[str, Any]] = {}
        missing: Dict[int, str] = {}

        for i in range(1, cfg.n_agents + 1):
            payload = {
                "outer_index": outer_j,
                "agent_id": i,
                "r_i": r_all[i - 1, :].tolist(),
                "v_i": v_all[i - 1, :].tolist(),
                "r_neighbors": [r_all[k - 1, :].tolist() for k in nbrs[i]],
                "u_prev": u_prev[i].tolist(),
                "neighbors": nbrs[i],
            }
            try:
                req_socks[i].send(dumps(make_envelope("mpc_request", payload, src="coord", dst=f"ctrl{i}")))
                logging.info(f"[Coordinator] Sent MPC request to controller_node{i} at t = {cfg.dt * outer_j}.")
                logging.info(f"  payload: agent_id={payload['agent_id']}, r_i={payload['r_i']}")
                rep_i = loads(req_socks[i].recv())["payload"]
                replies[i] = rep_i
            except zmq.Again:
                missing[i] = "timeout"
                replies[i] = {"ok": False, "agent_id": i, "reason": "timeout"}
                req_socks[i] = _reset_socket(ctx, req_socks[i], endpoints[i], cfg.coord_controller_timeout_ms, cfg.req_linger_ms)
            except Exception as e:
                missing[i] = f"error: {e}"
                replies[i] = {"ok": False, "agent_id": i, "reason": f"error: {e}"}
                req_socks[i] = _reset_socket(ctx, req_socks[i], endpoints[i], cfg.coord_controller_timeout_ms, cfg.req_linger_ms)

        if not all(bool(replies[i].get("ok", False)) for i in range(1, cfg.n_agents + 1)):
            if missing:
                print(f"[Coordinator] outer_j={outer_j}: failed controllers: {missing}")
            U = np.zeros((M, cfg.n_agents, cfg.dim), dtype=float)
            rep.send(
                dumps(
                    make_envelope(
                        "coord_reply",
                        {
                            "ok": False,
                            "outer_index": outer_j,
                            "neighbors": nbrs,
                            "model": cfg.model,
                            "M": M,
                            "U_seq": U.tolist(),
                            "replies": replies,
                            "note": "At least one controller failed; returning zeros.",
                        },
                        src="coord",
                    )
                )
            )
            continue

        U = np.zeros((M, cfg.n_agents, cfg.dim), dtype=float)
        diag: Dict[str, Dict[int, Any]] = {
            k: {}
            for k in [
                "objective_primary",
                "lex_used",
                "phi_terminal",
                "ri_terminal",
                "r_term",
                "diam_C",
                "t_primary_ms",
                "t_lex_ms",
                "t_total_ms",
                "n_var_primary",
                "n_eq_primary",
                "n_ineq_primary",
                "n_var_lex",
                "n_eq_lex",
                "n_ineq_lex",
                "nominal_fallback_used",
                "nominal_status",
                "bary_r",
            ]
        }

        for i in range(1, cfg.n_agents + 1):
            rep_i = replies[i]
            u_seq = np.array(rep_i["u_seq"], dtype=float)
            U[:, i - 1, :] = u_seq
            for k in diag.keys():
                diag[k][i] = rep_i.get(k, np.nan)
            u_prev[i] = u_seq[-1, :].copy()

        rep.send(
            dumps(
                make_envelope(
                    "coord_reply",
                    {"ok": True, "outer_index": outer_j, "neighbors": nbrs, "U_seq": U.tolist(), "diag": diag},
                    src="coord",
                )
            )
        )


if __name__ == "__main__":
    main()