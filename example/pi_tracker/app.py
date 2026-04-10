from __future__ import annotations

import argparse
import json
import signal
import sys
import time

import cv2
import numpy as np

from .camera import FrameSource
from .detector import create_backend
from .kinematics import estimate_velocity
from .types import KinematicsState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AprilTag pose + velocity tracker for Raspberry Pi drone use")
    parser.add_argument("--tag-size", type=float, required=True, help="AprilTag size in meters")
    parser.add_argument("--fx", type=float, required=True, help="Camera fx in pixels")
    parser.add_argument("--fy", type=float, required=True, help="Camera fy in pixels")
    parser.add_argument("--cx", type=float, required=True, help="Camera cx in pixels")
    parser.add_argument("--cy", type=float, required=True, help="Camera cy in pixels")
    parser.add_argument("--family", type=str, default="tag36h11", help="AprilTag family")
    parser.add_argument("--input", type=str, help="Image or video file input. If omitted, use a live camera source.")
    parser.add_argument("--camera-id", type=int, default=0, help="OpenCV camera ID fallback")
    parser.add_argument("--use-picamera2", action="store_true", help="Use PiCamera2 if installed")
    parser.add_argument("--prefer-opencv", action="store_true", help="Prefer OpenCV AprilTag module when available")
    parser.add_argument("--v-alpha", type=float, default=0.45, help="Velocity low-pass alpha in [0,1]")
    parser.add_argument("--draw", action="store_true", help="Display annotated image")
    parser.add_argument("--debug", action="store_true", help="Print debug logs to stderr")
    return parser.parse_args()


def debug_log(enabled: bool, message: str) -> None:
    if enabled:
        print(f"[DEBUG] {message}", file=sys.stderr, flush=True)


def run() -> int:
    args = parse_args()
    if not (0.0 <= args.v_alpha <= 1.0):
        raise ValueError("--v-alpha must be in [0,1]")

    K = np.array([[args.fx, 0.0, args.cx], [0.0, args.fy, args.cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.zeros((4, 1), dtype=np.float64)

    backend = create_backend(args.prefer_opencv, args.family)
    debug_log(args.debug, f"backend={backend.name}, family={args.family}, prefer_opencv={args.prefer_opencv}")
    src = FrameSource(use_picamera2=args.use_picamera2, camera_id=args.camera_id, input_path=args.input)

    state: KinematicsState | None = None
    running = True

    def stop_handler(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop_handler)

    try:
        while running:
            frame = src.next()
            if frame is None:
                debug_log(args.debug, "input exhausted")
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = backend.detect(gray)
            debug_log(args.debug, f"detections={len(detections)}")

            output = {"timestamp": time.time(), "backend": backend.name, "targets": []}
            now = time.monotonic()

            for det in detections:
                try:
                    pose = backend.estimate_pose(det, tag_size=args.tag_size, K=K, dist=dist)
                except Exception:
                    debug_log(args.debug, f"pose failed for tag_id={det.tag_id}")
                    continue

                state = estimate_velocity(state, now, pose.position_m, args.v_alpha)
                target = {
                    "tag_id": det.tag_id,
                    "position_m": {
                        "x": float(pose.position_m[0]),
                        "y": float(pose.position_m[1]),
                        "z": float(pose.position_m[2]),
                    },
                    "velocity_mps": {
                        "vx": float(state.v_cam[0]),
                        "vy": float(state.v_cam[1]),
                        "vz": float(state.v_cam[2]),
                    },
                    "euler_rad": {
                        "roll": float(pose.euler_rad[0]),
                        "pitch": float(pose.euler_rad[1]),
                        "yaw": float(pose.euler_rad[2]),
                    },
                    "margin": float(det.margin),
                }
                output["targets"].append(target)

                if args.draw:
                    pts = det.corners.astype(np.int32).reshape(-1, 1, 2)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                    cv2.putText(
                        frame,
                        f"{backend.name}:id={target['tag_id']} z={target['position_m']['z']:.2f}m",
                        tuple(pts[0, 0]),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        1,
                        cv2.LINE_AA,
                    )

            print(json.dumps(output, ensure_ascii=False), flush=True)

            if args.draw:
                cv2.imshow("pi_apriltag_tracker", frame)
                if cv2.waitKey(1) == 27:
                    break
    finally:
        src.close()
        if args.draw:
            cv2.destroyAllWindows()

    return 0
