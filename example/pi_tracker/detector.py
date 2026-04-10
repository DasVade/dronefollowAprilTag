from __future__ import annotations

import importlib.util
from dataclasses import dataclass
from typing import Protocol

import cv2
import numpy as np

from .kinematics import rotation_to_euler, rvec_tvec_to_pose
from .types import Detection, PoseResult


class DetectorBackend(Protocol):
    name: str

    def detect(self, gray: np.ndarray) -> list[Detection]: ...

    def estimate_pose(self, det: Detection, tag_size: float, K: np.ndarray, dist: np.ndarray) -> PoseResult: ...


@dataclass
class OpenCVAprilTagBackend:
    family: str

    def __post_init__(self):
        aruco = cv2.aruco
        family_map = {
            "tag36h11": aruco.DICT_APRILTAG_36h11,
            "tag25h9": aruco.DICT_APRILTAG_25h9,
            "tag16h5": aruco.DICT_APRILTAG_16h5,
        }
        if self.family not in family_map:
            raise ValueError("OpenCV backend supports: tag36h11/tag25h9/tag16h5")
        self._dict = aruco.getPredefinedDictionary(family_map[self.family])
        self._params = aruco.DetectorParameters()
        self._detector = aruco.ArucoDetector(self._dict, self._params)
        self.name = "opencv"

    @staticmethod
    def _reorder_corners(corners: np.ndarray) -> np.ndarray:
        # OpenCV is lt-rt-rb-lb, convert to lb-rb-rt-lt to align with repository docs.
        return corners[[3, 2, 1, 0], :]

    def detect(self, gray: np.ndarray) -> list[Detection]:
        corners, ids, _rejected = self._detector.detectMarkers(gray)
        if ids is None:
            return []
        out: list[Detection] = []
        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i].reshape(4, 2).astype(np.float64)
            out.append(Detection(tag_id=int(marker_id), corners=self._reorder_corners(c), margin=-1.0))
        return out

    def estimate_pose(self, det: Detection, tag_size: float, K: np.ndarray, dist: np.ndarray) -> PoseResult:
        half = tag_size / 2.0
        obj_pts = np.array([[-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0]], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, det.corners, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            raise RuntimeError("OpenCV solvePnP failed")
        p_cam, euler = rvec_tvec_to_pose(rvec, tvec)
        return PoseResult(position_m=p_cam, euler_rad=euler)


@dataclass
class ApriltagBindingBackend:
    family: str

    def __post_init__(self):
        if importlib.util.find_spec("apriltag") is None:
            raise RuntimeError("apriltag python binding not available")
        from apriltag import apriltag as detector_type

        self._detector = detector_type(self.family)
        self.name = "apriltag"

    def detect(self, gray: np.ndarray) -> list[Detection]:
        ds = self._detector.detect(gray)
        return [
            Detection(
                tag_id=int(d["id"]),
                corners=np.array(d["lb-rb-rt-lt"], dtype=np.float64),
                margin=float(d.get("margin", -1.0)),
                homography=np.array(d.get("homography"), dtype=np.float64) if d.get("homography") is not None else None,
                raw=d,
            )
            for d in ds
        ]

    def estimate_pose(self, det: Detection, tag_size: float, K: np.ndarray, dist: np.ndarray) -> PoseResult:
        _ = dist
        pose = self._detector.estimate_tag_pose(
            det.raw,
            tagsize=tag_size,
            fx=float(K[0, 0]),
            fy=float(K[1, 1]),
            cx=float(K[0, 2]),
            cy=float(K[1, 2]),
        )
        rot_mat = np.array(pose["R"], dtype=np.float64)
        p_cam = np.array(pose["t"], dtype=np.float64).reshape(3)
        return PoseResult(position_m=p_cam, euler_rad=rotation_to_euler(rot_mat))


def create_backend(prefer_opencv: bool, family: str) -> DetectorBackend:
    if prefer_opencv and hasattr(cv2, "aruco") and hasattr(cv2.aruco, "DICT_APRILTAG_36h11"):
        try:
            return OpenCVAprilTagBackend(family)
        except Exception:
            pass
    return ApriltagBindingBackend(family)
