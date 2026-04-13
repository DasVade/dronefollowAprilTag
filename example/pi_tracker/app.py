from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from .camera import FrameSource
from .detector import create_backend
from .kinematics import estimate_velocity
from .types import KinematicsState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AprilTag pose + velocity tracker for Raspberry Pi / ESP32 Wi-Fi stream")
    parser.add_argument(
        "--config",
        type=str,
        default=str(Path(__file__).with_name("defaults.json")),
        help="JSON defaults file (tag_size/fx/fy/cx/cy/family/prefer_opencv/v_alpha/dist_coeffs)",
    )
    parser.add_argument("--tag-size", type=float, help="AprilTag size in meters")
    parser.add_argument("--fx", type=float, help="Camera fx in pixels")
    parser.add_argument("--fy", type=float, help="Camera fy in pixels")
    parser.add_argument("--cx", type=float, help="Camera cx in pixels")
    parser.add_argument("--cy", type=float, help="Camera cy in pixels")
    parser.add_argument("--dist-coeffs", type=str, help="Comma-separated distortion coeffs, e.g. 'k1,k2,p1,p2,k3'")
    parser.add_argument("--family", type=str, help="AprilTag family")
    parser.add_argument("--input", type=str, help="Image/video file or stream URL. If omitted, use live camera source.")
    parser.add_argument("--input-url", type=str, help="Camera stream URL (e.g. http://<esp-ip>:81/stream)")
    parser.add_argument("--camera-id", type=int, default=0, help="OpenCV camera ID fallback")
    parser.add_argument("--use-picamera2", action="store_true", help="Use PiCamera2 if installed")
    parser.add_argument(
        "--prefer-opencv",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Prefer OpenCV AprilTag module when available",
    )
    parser.add_argument(
        "--undistort",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Undistort each frame before detection/pose",
    )
    parser.add_argument("--v-alpha", type=float, help="Velocity low-pass alpha in [0,1]")
    parser.add_argument("--draw", action="store_true", help="Display annotated image")
    parser.add_argument("--debug", action="store_true", help="Print debug logs to stderr")
    return parser.parse_args()


def debug_log(enabled: bool, message: str) -> None:
    if enabled:
        print(f"[DEBUG] {message}", file=sys.stderr, flush=True)


def load_defaults(config_path: str) -> dict:
    defaults = {
        "tag_size": None,
        "fx": None,
        "fy": None,
        "cx": None,
        "cy": None,
        "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
        "family": "tag36h11",
        "prefer_opencv": False,
        "undistort": False,
        "v_alpha": 0.45,
    }
    path = Path(config_path)
    if not path.exists():
        return defaults
    with path.open("r", encoding="utf-8") as f:
        loaded = json.load(f)
    if not isinstance(loaded, dict):
        raise ValueError(f"Config file must contain a JSON object: {config_path}")
    for k in defaults:
        if k in loaded:
            defaults[k] = loaded[k]
    return defaults


def parse_dist_coeffs(value: str | None, fallback: list[float]) -> np.ndarray:
    if value is None:
        coeffs = fallback
    else:
        coeffs = [float(x.strip()) for x in value.split(",") if x.strip()]
    if len(coeffs) not in (4, 5, 8, 12, 14):
        raise ValueError("Distortion coeff count must be one of: 4,5,8,12,14")
    return np.array(coeffs, dtype=np.float64).reshape(-1, 1)


def run() -> int:
    args = parse_args()

    if args.input is not None and args.input_url is not None:
        raise ValueError("Use only one of --input and --input-url")

    defaults = load_defaults(args.config)
    tag_size = args.tag_size if args.tag_size is not None else defaults["tag_size"]
    fx = args.fx if args.fx is not None else defaults["fx"]
    fy = args.fy if args.fy is not None else defaults["fy"]
    cx = args.cx if args.cx is not None else defaults["cx"]
    cy = args.cy if args.cy is not None else defaults["cy"]
    family = args.family if args.family is not None else defaults["family"]
    prefer_opencv = args.prefer_opencv if args.prefer_opencv is not None else bool(defaults["prefer_opencv"])
    undistort = args.undistort if args.undistort is not None else bool(defaults["undistort"])
    v_alpha = args.v_alpha if args.v_alpha is not None else float(defaults["v_alpha"])
    input_path = args.input_url if args.input_url is not None else args.input

    missing = [name for name, value in [("tag_size", tag_size), ("fx", fx), ("fy", fy), ("cx", cx), ("cy", cy)] if value is None]
    if missing:
        raise ValueError(f"Missing required values: {', '.join(missing)}. Set them in --config or via CLI flags.")

    if not (0.0 <= v_alpha <= 1.0):
        raise ValueError("--v-alpha must be in [0,1]")

    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = parse_dist_coeffs(args.dist_coeffs, defaults["dist_coeffs"])

    backend = create_backend(prefer_opencv, family)
    debug_log(args.debug, f"backend={backend.name}, family={family}, prefer_opencv={prefer_opencv}, config={args.config}")
    debug_log(args.debug, f"source={input_path if input_path else f'camera:{args.camera_id}'}, undistort={undistort}")

    src = FrameSource(use_picamera2=args.use_picamera2, camera_id=args.camera_id, input_path=input_path)

    state: KinematicsState | None = None
    running = True

    undistort_ready = False
    map1 = None
    map2 = None
    K_pose = K
    dist_pose = dist

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

            if undistort and not undistort_ready:
                h, w = frame.shape[:2]
                K_opt, _roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=0)
                map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, K_opt, (w, h), cv2.CV_16SC2)
                K_pose = K_opt
                dist_pose = np.zeros((5, 1), dtype=np.float64)
                undistort_ready = True
                debug_log(args.debug, f"undistort map ready: {w}x{h}")

            frame_for_detect = frame
            if undistort and undistort_ready:
                frame_for_detect = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

            gray = cv2.cvtColor(frame_for_detect, cv2.COLOR_BGR2GRAY)
            detections = backend.detect(gray)
            debug_log(args.debug, f"detections={len(detections)}")

            output = {
                "timestamp": time.time(),
                "backend": backend.name,
                "undistort": bool(undistort),
                "targets": [],
            }
            now = time.monotonic()

            for det in detections:
                try:
                    pose = backend.estimate_pose(det, tag_size=tag_size, K=K_pose, dist=dist_pose)
                except Exception:
                    debug_log(args.debug, f"pose failed for tag_id={det.tag_id}")
                    continue

                state = estimate_velocity(state, now, pose.position_m, v_alpha)
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
                    cv2.polylines(frame_for_detect, [pts], True, (0, 255, 0), 2)
                    cv2.putText(
                        frame_for_detect,
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
                cv2.imshow("pi_apriltag_tracker", frame_for_detect)
                if cv2.waitKey(1) == 27:
                    break
    finally:
        src.close()
        if args.draw:
            cv2.destroyAllWindows()

    return 0
