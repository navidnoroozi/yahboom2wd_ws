from __future__ import annotations

import argparse
import signal
import sys
from typing import Optional

import zmq

from .config_utils import add_yahboom_dmpc_args, cfg_from_args
from .consensus_comm import dumps, loads, make_envelope
from .consensus_controller import solve_mpc_request
from .explicit_hybrid_controller import ExplicitHybridController


def _endpoint_from_args(bind_host: str, bind_port: int, endpoint: Optional[str]) -> str:
    if endpoint:
        return endpoint
    host = bind_host.strip()
    if host in {"", "0.0.0.0", "*"}:
        host = "*"
    return f"tcp://{host}:{int(bind_port)}"


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "ZeroMQ REP node for one Yahboom robot's local DMPC controller. "
            "Run this on the Raspberry Pi belonging to the robot."
        )
    )
    add_yahboom_dmpc_args(parser)
    parser.add_argument("--agent-id", type=int, required=True, help="1-based agent id, e.g. 1 for robot1.")
    parser.add_argument("--bind-host", default="0.0.0.0", help="Bind host. Use 0.0.0.0 or * for remote VM access.")
    parser.add_argument("--bind-port", type=int, default=None, help="Bind port. Defaults to ctrl_base_port + agent_id.")
    parser.add_argument("--endpoint", default=None, help="Full ZMQ bind endpoint, e.g. tcp://*:5601.")

    args = parser.parse_args()
    cfg = cfg_from_args(args)

    agent_id = int(args.agent_id)
    bind_port = int(args.bind_port) if args.bind_port is not None else int(cfg.ctrl_base_port + agent_id)
    endpoint = _endpoint_from_args(args.bind_host, bind_port, args.endpoint)

    hybrid = ExplicitHybridController(cfg, agent_id)

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REP)
    sock.setsockopt(zmq.LINGER, cfg.req_linger_ms)
    sock.bind(endpoint)

    stop_requested = {"value": False}

    def _handle_signal(signum, frame):  # noqa: ANN001
        stop_requested["value"] = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    print(
        f"[YahboomDMPCController {agent_id}] REP bound at {endpoint} | "
        f"model={cfg.model} | n_agents={cfg.n_agents} | M={cfg.horizon_M()} | "
        f"objective_mode={cfg.objective_mode} | safety_enabled={cfg.safety_enabled}",
        flush=True,
    )

    while not stop_requested["value"]:
        try:
            msg = loads(sock.recv(flags=zmq.NOBLOCK))
        except zmq.Again:
            # Avoid blocking forever so SIGTERM from ros2 launch shuts down cleanly.
            zmq.sleep(0.02)
            continue

        mtype = msg.get("type")

        if mtype == "shutdown":
            sock.send(dumps(make_envelope("shutdown_ack", {"ok": True}, src=f"ctrl{agent_id}")))
            break

        if mtype == "mpc_request":
            payload = msg["payload"]
            out, ok = solve_mpc_request(cfg, payload)
            if ok:
                sock.send(dumps(make_envelope("mpc_reply", out, src=f"ctrl{agent_id}")))
            else:
                sock.send(
                    dumps(
                        make_envelope(
                            "mpc_reply",
                            {"ok": False, "agent_id": agent_id, "reason": "infeasible_or_solver_failed"},
                            src=f"ctrl{agent_id}",
                        )
                    )
                )
            continue

        if mtype == "hybrid_request":
            payload = msg["payload"]
            if cfg.safety_enabled:
                res = hybrid.step(payload)
                sock.send(
                    dumps(
                        make_envelope(
                            "hybrid_reply",
                            {
                                "ok": True,
                                "agent_id": agent_id,
                                "u_safe": res.u_safe.tolist(),
                                "diag": res.diag,
                            },
                            src=f"ctrl{agent_id}",
                        )
                    )
                )
            else:
                sock.send(
                    dumps(
                        make_envelope(
                            "hybrid_reply",
                            {
                                "ok": True,
                                "agent_id": agent_id,
                                "u_safe": payload["u_nom"],
                                "diag": {"mode": "disabled", "desired_mode": "disabled"},
                            },
                            src=f"ctrl{agent_id}",
                        )
                    )
                )
            continue

        sock.send(dumps(make_envelope("error", {"error": f"unknown msg type: {mtype}"}, src=f"ctrl{agent_id}")))

    try:
        sock.close(0)
    finally:
        ctx.term()


if __name__ == "__main__":
    main()
