from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np

from .types import KinematicsState


def rvec_tvec_to_pose(rvec: np.ndarray, tvec: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rot_mat, _ = cv2.Rodrigues(rvec)
    yaw = math.atan2(rot_mat[1, 0], rot_mat[0, 0])
    pitch = math.atan2(-rot_mat[2, 0], math.sqrt(rot_mat[2, 1] ** 2 + rot_mat[2, 2] ** 2))
    roll = math.atan2(rot_mat[2, 1], rot_mat[2, 2])
    euler = np.array([roll, pitch, yaw], dtype=np.float64)
    return tvec.reshape(3), euler


def rotation_to_euler(rot_mat: np.ndarray) -> np.ndarray:
    yaw = math.atan2(rot_mat[1, 0], rot_mat[0, 0])
    pitch = math.atan2(-rot_mat[2, 0], math.sqrt(rot_mat[2, 1] ** 2 + rot_mat[2, 2] ** 2))
    roll = math.atan2(rot_mat[2, 1], rot_mat[2, 2])
    return np.array([roll, pitch, yaw], dtype=np.float64)


def estimate_velocity(prev: Optional[KinematicsState], t: float, p_cam: np.ndarray, alpha: float) -> KinematicsState:
    if prev is None:
        return KinematicsState(t=t, p_cam=p_cam, v_cam=np.zeros(3, dtype=np.float64))
    dt = max(t - prev.t, 1e-3)
    v_raw = (p_cam - prev.p_cam) / dt
    v_filt = alpha * v_raw + (1.0 - alpha) * prev.v_cam
    return KinematicsState(t=t, p_cam=p_cam, v_cam=v_filt)
