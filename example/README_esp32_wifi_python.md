# ESP32 Wi-Fi Camera + Python AprilTag Pipeline

This guide wires your **XIAO ESP32S3 Sense** camera stream to the Python tracker in this repository.

## 1) Flash ESP32 Camera Firmware

Files:
- `example/esp32_xiao_wifi_camera/esp32_xiao_wifi_camera.ino`
- `example/esp32_xiao_wifi_camera/secrets.example.h`

Steps:
1. Copy `secrets.example.h` to `secrets.h` and set your Wi-Fi SSID/password.
2. Open `esp32_xiao_wifi_camera.ino` in Arduino IDE.
3. Select board: `Seeed XIAO ESP32S3`.
4. Upload and open Serial Monitor at `115200`.
5. Note the printed IP address, for example `192.168.1.50`.

Quick checks from your computer:
- `http://192.168.1.50/` (status text)
- `http://192.168.1.50/capture` (single JPEG)
- `http://192.168.1.50/stream` (MJPEG stream)

## 2) Calibrate Intrinsics (OpenCV)

Use a checkerboard (default: 9x6 inner corners).

```bash
python3 example/tools/calibrate_intrinsics.py \
  --source http://192.168.1.50/stream \
  --rows 6 --cols 9 \
  --square-size 0.024 \
  --samples 30 \
  --show \
  --output example/pi_tracker/camera_calibration.json
```

Use the output values (`fx`, `fy`, `cx`, `cy`, `dist_coeffs`) in your tracker config.

## 3) Run AprilTag Tracking on PC

The tracker now accepts stream URLs directly.

```bash
python3 example/run_pi_tracker.py \
  --input-url http://192.168.1.50/stream \
  --tag-size 0.16 \
  --fx <fx> --fy <fy> --cx <cx> --cy <cy> \
  --dist-coeffs "<k1>,<k2>,<p1>,<p2>,<k3>" \
  --family tag36h11 \
  --prefer-opencv \
  --draw \
  --debug
```

## 4) Distortion Handling Modes

Mode A (recommended):
- Keep raw frames.
- Use `--prefer-opencv` and pass real `--dist-coeffs`.
- Pose uses `solvePnP(K, dist)`.

Mode B:
- Add `--undistort`.
- Frames are undistorted once maps are initialized.
- Pose uses undistorted camera matrix with zero distortion.

Example:

```bash
python3 example/run_pi_tracker.py \
  --input-url http://192.168.1.50/stream \
  --tag-size 0.16 \
  --fx <fx> --fy <fy> --cx <cx> --cy <cy> \
  --dist-coeffs "<k1>,<k2>,<p1>,<p2>,<k3>" \
  --prefer-opencv \
  --undistort \
  --draw
```

## 5) Notes

- Keep runtime resolution the same as calibration resolution.
- If FPS is low, reduce ESP frame size to `QVGA` in the `.ino` file.
- If your board package has a different camera pin map, adjust pin macros in the sketch.
