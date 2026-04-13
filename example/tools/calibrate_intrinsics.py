#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate camera intrinsics using checkerboard frames")
    parser.add_argument("--source", required=True, help="Camera source: index, file path, or URL")
    parser.add_argument("--rows", type=int, default=6, help="Checkerboard inner-corner rows")
    parser.add_argument("--cols", type=int, default=9, help="Checkerboard inner-corner cols")
    parser.add_argument("--square-size", type=float, required=True, help="Checkerboard square size in meters")
    parser.add_argument("--samples", type=int, default=30, help="Required successful checkerboard detections")
    parser.add_argument("--min-delay-ms", type=int, default=300, help="Min time between accepted samples")
    parser.add_argument("--output", type=str, default="example/pi_tracker/camera_calibration.json", help="Output JSON file")
    parser.add_argument("--show", action="store_true", help="Preview detection window")
    return parser.parse_args()


def open_capture(source: str) -> cv2.VideoCapture:
    if source.isdigit():
        cap = cv2.VideoCapture(int(source))
    else:
        cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open source: {source}")
    return cap


def main() -> int:
    args = parse_args()
    board_size = (args.cols, args.rows)

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0 : args.cols, 0 : args.rows].T.reshape(-1, 2)
    objp *= args.square_size

    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []

    cap = open_capture(args.source)
    frame_size: tuple[int, int] | None = None
    last_accept_tick = 0
    min_delay_ticks = int(args.min_delay_ms / 1000.0 * cv2.getTickFrequency())

    print("Controls: SPACE=force sample, q=quit")

    try:
        while len(objpoints) < args.samples:
            ok, frame = cap.read()
            if not ok:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_size = (gray.shape[1], gray.shape[0])

            flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            found, corners = cv2.findChessboardCorners(gray, board_size, flags)
            display = frame.copy()

            if found:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                cv2.drawChessboardCorners(display, board_size, corners, found)

                now_tick = cv2.getTickCount()
                if now_tick - last_accept_tick >= min_delay_ticks:
                    objpoints.append(objp.copy())
                    imgpoints.append(corners)
                    last_accept_tick = now_tick
                    print(f"sample {len(objpoints)}/{args.samples}")

            if args.show:
                cv2.putText(
                    display,
                    f"samples: {len(objpoints)}/{args.samples}",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("calibration", display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" ") and found:
                    objpoints.append(objp.copy())
                    imgpoints.append(corners)
                    print(f"sample {len(objpoints)}/{args.samples} (manual)")
            else:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        if len(objpoints) < 8:
            raise RuntimeError("Not enough valid samples. Capture at least 8, preferably 25-40.")
        if frame_size is None:
            raise RuntimeError("No frame captured.")

        rms, K, dist, _rvecs, _tvecs = cv2.calibrateCamera(objpoints, imgpoints, frame_size, None, None)

        out = {
            "image_width": frame_size[0],
            "image_height": frame_size[1],
            "rms_reprojection_error": float(rms),
            "fx": float(K[0, 0]),
            "fy": float(K[1, 1]),
            "cx": float(K[0, 2]),
            "cy": float(K[1, 2]),
            "dist_coeffs": [float(x) for x in dist.reshape(-1)],
        }

        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(out, indent=2), encoding="utf-8")

        print("Calibration complete")
        print(json.dumps(out, indent=2))
        print(f"Saved: {output_path}")
        return 0
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
