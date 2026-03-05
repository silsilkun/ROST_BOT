from __future__ import annotations

import math
from typing import List, Tuple

from estimation.utils.utils import validate_waste_coordinates_flat


def ok_coords(data: List[float]) -> Tuple[bool, str]:
    """validate_*가 bool 또는 (bool, reason) 계약인 점을 안전하게 처리."""
    ret = validate_waste_coordinates_flat(data)
    if isinstance(ret, tuple):
        return bool(ret[0]), str(ret[1])
    return bool(ret), "invalid coords"


def count_from_coords(coords: List[float]) -> int:
    # [수정 포인트] coords 묶음이 바뀌면 5만 수정
    return len(coords) // 5


def sanitize_ids(ids: List[float], unknown_type_id: float) -> List[float]:
    out: List[float] = []
    for v in (ids or []):
        try:
            f = float(v)
            out.append(unknown_type_id if (math.isnan(f) or math.isinf(f)) else f)
        except Exception:
            out.append(unknown_type_id)
    return out


def pack_pickup_commands(coords: List[float], ids: List[float]) -> Tuple[List[float], str]:
    # [수정 포인트] 출력 형태가 바뀌면 여기만
    n = count_from_coords(coords)
    if n <= 0:
        return [], "no objects"
    if len(ids) != n:
        return [], f"length mismatch coords={len(coords)} types={len(ids)}"

    out: List[float] = []
    for i in range(n):
        b = 5 * i
        out += [ids[i], coords[b + 1], coords[b + 2], coords[b + 3], coords[b + 4]]
    return [float(x) for x in out], "ok"
