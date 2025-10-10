from dataclasses import dataclass
from typing import Dict, List, Tuple
import time
import math
import json

@dataclass
class PlanConfig:
    frame_id: str = "map"
    pipeline_length: float = 100.0     # meters (easy knob)
    num_points: int = 5                # full cycle count
    period_s: float = 30.0             # 30 seconds between waypoints
    start_buffer_s: float = 5.0        # small future buffer
    depth_underwater: float = -2.0     # subs travel depth (z)
    mid_surface_z: float = 0.0         # mid surfaces (z)

def _time_to_msgs(ts: float) -> Dict[str, int]:
    sec = int(math.floor(ts))
    nsec = int((ts - sec) * 1e9)
    return {"sec": sec, "nanosec": nsec}

def _ps_dict(x: float, y: float, z: float, frame_id: str, ts: float) -> Dict:
    return {
        "header": {
            "stamp": _time_to_msgs(ts),
            "frame_id": frame_id
        },
        "pose": {
            "position": {"x": float(x), "y": float(y), "z": float(z)},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        }
    }

def build_plans_for_cycle(cfg: PlanConfig,
                          origin_xy: Tuple[float, float] = (0.0, 0.0),
                          pipeline_direction_deg: float = 0.0
                          ) -> Dict[str, List[Dict]]:
    assert cfg.num_points >= 5, "num_points must be >= 5 for these constraints"

    theta = math.radians(pipeline_direction_deg)
    ux, uy = math.cos(theta), math.sin(theta)

    step = cfg.pipeline_length / (cfg.num_points - 1)

    x0, y0 = origin_xy
    def along_pipeline(k: float) -> Tuple[float, float]:
        return (x0 + ux * (k * step), y0 + uy * (k * step))

    t0 = time.time() + cfg.start_buffer_s
    T = [t0 + i * cfg.period_s for i in range(cfg.num_points)]

    # Sub-A: k = i
    A_xy = [along_pipeline(i) for i in range(cfg.num_points)]
    A = [_ps_dict(x, y, cfg.depth_underwater, cfg.frame_id, T[i]) for i, (x, y) in enumerate(A_xy)]

    # Sub-B: one step behind A → k = i-1, with explicit equalities:
    B_xy = [along_pipeline(i - 1) for i in range(cfg.num_points)]
    B_xy[1] = A_xy[0]   # B[1] = A[0]
    B_xy[2] = A_xy[1]   # B[2] = A[1]
    B = [_ps_dict(x, y, cfg.depth_underwater, cfg.frame_id, T[i]) for i, (x, y) in enumerate(B_xy)]

    # MID with two surfacings:
    #   Mid[0] : staging (underwater) behind origin
    #   Mid[1] : FIRST surfacing "in front" (k=0.5), z = surface
    #   Mid[2] : meet A[2] (same XY), underwater
    #   Mid[3] : meet B[3] (same XY), underwater
    #   Mid[4] : SECOND surfacing after second meetup (ahead at k=2.5), z = surface
    mid0_xy = along_pipeline(-0.5)
    mid1_xy = along_pipeline(0.5)       # first surfacing
    mid2_xy = A_xy[2]                   # meet Sub-A
    mid3_xy = B_xy[3]                   # meet Sub-B
    mid4_xy = along_pipeline(2.5)       # second surfacing location

    mid_xyz = [
        (mid0_xy[0], mid0_xy[1], cfg.depth_underwater),  # Mid[0] underwater
        (mid1_xy[0], mid1_xy[1], cfg.mid_surface_z),     # Mid[1] surface
        (mid2_xy[0], mid2_xy[1], cfg.depth_underwater),  # Mid[2] meet A
        (mid3_xy[0], mid3_xy[1], cfg.depth_underwater),  # Mid[3] meet B
        (mid4_xy[0], mid4_xy[1], cfg.mid_surface_z),     # Mid[4] SECOND surfacing
    ]
    MID = [_ps_dict(x, y, z, cfg.frame_id, T[i]) for i, (x, y, z) in enumerate(mid_xyz)]

    # Enforce exact equality constraints (XY/Z copy to avoid drift)
    MID[2]["pose"]["position"] = A[2]["pose"]["position"]  # Mid[2] == A[2]
    MID[3]["pose"]["position"] = B[3]["pose"]["position"]  # Mid[3] == B[3]
    B[2]["pose"]["position"]   = A[1]["pose"]["position"]  # B[2]  == A[1]

    return {"mid": MID, "subA": A, "subB": B}
