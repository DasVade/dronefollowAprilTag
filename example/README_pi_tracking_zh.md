# Raspberry Pi + Pi Camera + AprilTag 跟随项目（模块化版）

按你的要求，已把之前单文件脚本拆成模块化结构，并且在 OpenCV 自带 AprilTag 功能可用时优先使用 OpenCV 后端。

## 1) 当前结构

```text
example/
├── run_pi_tracker.py           # 入口脚本
└── pi_tracker/
    ├── __init__.py
    ├── app.py                  # 参数解析 + 主循环
    ├── camera.py               # 相机输入层（PiCamera2/VideoCapture）
    ├── detector.py             # 检测与位姿后端（OpenCV优先 + apriltag fallback）
    ├── kinematics.py           # 速度估计、旋转转换
    └── types.py                # 公共数据结构
```

## 2) 后端选择策略

- `--prefer-opencv` 打开后：
  1. 若 `cv2.aruco` 含 AprilTag 字典且 family 支持，则使用 OpenCV AprilTag；
  2. 否则自动回退到本仓库 `apriltag` Python 绑定。
- 如果不加 `--prefer-opencv`，默认直接走本仓库 `apriltag` 绑定。

## 3) 运行

```bash
python3 example/run_pi_tracker.py \
  --use-picamera2 \
  --tag-size 0.12 \
  --fx 920 --fy 920 --cx 640 --cy 360 \
  --family tag36h11 \
  --prefer-opencv \
  --draw
```

## 4) 输出

每帧输出一行 JSON，包含：
- `backend`: 当前实际使用的后端（`opencv` / `apriltag`）
- `targets[].position_m`: 相对位置（米）
- `targets[].velocity_mps`: 相对速度（米/秒）
- `targets[].euler_rad`: 姿态欧拉角（弧度）
- `targets[].margin`: 解码质量（OpenCV 后端不可用时可能为 -1）

## 5) 说明

OpenCV 后端目前直接支持的 family 主要是：
- `tag36h11`
- `tag25h9`
- `tag16h5`

如果你要用 `tagStandard41h12`，建议先使用本仓库 `apriltag` 绑定后端，或后续扩展 OpenCV 字典映射。

## 6) 调试策略（建议按顺序执行）

1. **先验证环境依赖**
   - `python3 -c "import cv2; print(cv2.__version__)"`
   - `python3 -c "import apriltag; print('apriltag ok')"`（若走 fallback）
   - `python3 -c "from picamera2 import Picamera2; print('picamera2 ok')"`（若走树莓派相机）
2. **确认后端是否符合预期**
   - 启动时加 `--debug`，日志会打印当前实际后端（`opencv`/`apriltag`）和每帧检测数量。
3. **先排“看不到标签”问题**
   - 在 `--draw` 模式下确认绿色框是否稳定；
   - 优先用 `tag36h11` + `--prefer-opencv`，再扩展到其它 family；
   - 先固定距离（例如 1.5m）和光照，再调参数。
4. **再排“位姿跳变/速度噪声”问题**
   - 先检查内参（`fx fy cx cy`）是否来自同一分辨率；
   - 逐步减小 `--v-alpha`（例如 0.45 → 0.3 → 0.2）观察速度稳定性；
   - 记录 30 秒 JSON，离线画 `z` 和 `vz` 曲线看是否存在周期性抖动。
5. **飞行前联调策略**
   - 先地面手持标签做静态/匀速测试；
   - 再上机但不开闭环控制，仅记录输出；
   - 最后再开启低增益闭环，逐步提高控制带宽。
