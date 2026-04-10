from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path

import cv2


if importlib.util.find_spec("picamera2") is not None:
    Picamera2 = importlib.import_module("picamera2").Picamera2
else:
    Picamera2 = None


class FrameSource:
    def __init__(self, use_picamera2: bool, camera_id: int, input_path: str | None = None):
        self._close = None
        self._exhausted = False

        if input_path is not None:
            path = Path(input_path)
            if not path.exists():
                raise RuntimeError(f"Input path does not exist: {input_path}")

            if path.is_file() and self._is_image_file(path):
                frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if frame is None:
                    raise RuntimeError(f"Failed to load image file: {input_path}")

                def next_image():
                    if self._exhausted:
                        return None
                    self._exhausted = True
                    return frame.copy()

                self._next = next_image
                return

            cap = cv2.VideoCapture(str(path))
            if not cap.isOpened():
                raise RuntimeError(f"Cannot open video file: {input_path}")

            def next_video():
                ok, frame = cap.read()
                if not ok:
                    self._exhausted = True
                    return None
                return frame

            self._next = next_video
            self._close = cap.release
            return

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

    @staticmethod
    def _is_image_file(path: Path) -> bool:
        return path.suffix.lower() in {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}

    def next(self):
        return self._next()

    def close(self):
        if self._close:
            self._close()
