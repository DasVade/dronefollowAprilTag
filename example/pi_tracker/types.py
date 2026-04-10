from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class Detection:
    tag_id: int
    corners: np.ndarray  # (4,2) in lb-rb-rt-lt order
    margin: float
    homography: Optional[np.ndarray] = None
    raw: Optional[dict] = None


@dataclass
class PoseResult:
    position_m: np.ndarray
    euler_rad: np.ndarray


@dataclass
class KinematicsState:
    t: float
    p_cam: np.ndarray
    v_cam: np.ndarray
