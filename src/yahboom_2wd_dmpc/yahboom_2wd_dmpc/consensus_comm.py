# consensus_comm.py
import json
import time
from typing import Any, Dict, Optional

def make_envelope(msg_type: str, payload: Dict[str, Any], src: Optional[str] = None, dst: Optional[str] = None) -> Dict[str, Any]:
    return {
        "type": msg_type,
        "ts": time.time(),
        "src": src,
        "dst": dst,
        "payload": payload,
    }

def dumps(msg: Dict[str, Any]) -> bytes:
    return json.dumps(msg).encode("utf-8")

def loads(b: bytes) -> Dict[str, Any]:
    return json.loads(b.decode("utf-8"))
