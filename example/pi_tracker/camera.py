from __future__ import annotations

import importlib
import importlib.util

import cv2


if importlib.util.find_spec("picamera2") is not None:
    Picamera2 = importlib.import_module("picamera2").Picamera2
else:
    Picamera2 = None


class FrameSource:
    def __init__(self, use_picamera2: bool, camera_id: int):
        self._close = None
        if use_picamera2:
            if Picamera2 is None:
                raise RuntimeError("--use-picamera2 set but picamera2 is not installed")
            cam = Picamera2()
            config = cam.create_video_configuration(main={"size": (1280, 720), "format": "RGB888"})
            cam.configure(config)
            cam.start()

            self._next = lambda: cv2.cvtColor(cam.capture_array(), cv2.COLOR_RGB2BGR)
            self._close = cam.stop
            return

        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera id {camera_id}")

        def next_frame():
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Failed to capture frame")
            return frame

        self._next = next_frame
        self._close = cap.release

    def next(self):
        return self._next()

    def close(self):
        if self._close:
            self._close()
