from __future__ import annotations

import argparse
import zmq

from .consensus_comm import dumps, loads, make_envelope
from .consensus_config import add_common_args, config_from_namespace
from .consensus_controller import solve_mpc_request
from .explicit_hybrid_controller import ExplicitHybridController


def main() -> None:
    parser = add_common_args(argparse.ArgumentParser())
    ns = parser.parse_args()

    if ns.agent_id is None:
        raise SystemExit("controller_node.py requires --agent-id")

    cfg = config_from_namespace(ns)
    agent_id = int(ns.agent_id)
    endpoint = cfg.controller_endpoint(agent_id)

    hybrid = ExplicitHybridController(cfg, agent_id)

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REP)
    sock.setsockopt(zmq.LINGER, cfg.req_linger_ms)
    sock.bind(endpoint)

    print(
        f"[ControllerNode {agent_id}] REP bound at {endpoint} | "
        f"model={cfg.model} | objective_mode={cfg.objective_mode} | "
        f"safety_enabled={cfg.safety_enabled} | safety_method={cfg.safety_method}"
    )

    while True:
        msg = loads(sock.recv())
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
            res = hybrid.step(payload)
            sock.send(
                dumps(
                    make_envelope(
                        "hybrid_reply",
                        {"ok": True, "agent_id": agent_id, "u_safe": res.u_safe.tolist(), "diag": res.diag},
                        src=f"ctrl{agent_id}",
                    )
                )
            )
            continue

        sock.send(dumps(make_envelope("error", {"error": f"unknown msg type: {mtype}"}, src=f"ctrl{agent_id}")))


if __name__ == "__main__":
    main()