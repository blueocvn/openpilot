So sánh các cấu hình chạy simulator
===================================

* **A. openpilot**: `openpilot/tools/sim` của openpilot đời mới (gốc repo này).
* **B. bukapilot cũ**: `tools/sim` cũ (`bridge.py`, `start_bukapilot_docker.sh`), đã xóa, xem git history.
* **C. bukapilot mới**: `bukapilot/tools/sim` (bridge của A port sang bukapilot).

A và B là hai cấu hình đang có. C là kết quả của việc tích hợp.

## Môi trường

| | A. openpilot | B. bukapilot cũ | C. bukapilot mới |
|---|---|---|---|
| Nền | openpilot master (2025+) | openpilot 0.8.13 | openpilot 0.8.13 |
| Chạy ở đâu | native (`.carla/run-linux.sh`) hoặc WSL2 + CARLA trên Windows (`carla/run.sh`) | Docker `bukapilot-sim:local` | Docker `bukapilot-sim:local` (dùng lại) |
| Python | 3.12 (uv, `.venv`) | 3.8 (theo `Pipfile.lock`) | 3.8 |
| CARLA server / client | 0.9.16 / 0.9.16 | 0.9.12 (Docker, off-screen) / 0.9.12 | 0.9.12 / 0.9.12 |
| Simulator khác | MetaDrive (mặc định), CARLA | chỉ CARLA | chỉ CARLA |
| Lệnh chạy | `bash .carla/run-linux.sh` | `./start_carla.sh` + `./start_bukapilot_docker.sh` | `./start_carla.sh` + `./start_docker.sh` |
| Ai khởi động manager | bridge (`--launch-openpilot`) | tmux, 2 cửa sổ | bridge (`--launch-openpilot`), không cần tmux |
| Smoke test không cần bàn phím | có (`test_duration`) | không | `--duration N` |

## Xe giả lập và CAN

| | A | B | C |
|---|---|---|---|
| Fingerprint | `TESLA_MODEL_3` (CARLA), `HONDA_CIVIC_2022` (MetaDrive) | `HONDA CIVIC 2016` | `HONDA CIVIC 2016` |
| DBC | `tesla_model3_party` | `honda_civic_touring_2016_can_generated` + `acura_ilx_2016_nidec` | như B |
| Longitudinal | stock ACC giả lập hoặc `--openpilot-longitudinal` | openpilot (comma pedal 0x201) | openpilot (comma pedal) |
| Ga/phanh từ `actuators.accel` | vòng PI quanh `aEgo`, giới hạn trên bằng tốc độ cruise | mở vòng: `accel/1.6`, `accel/4.0`, rồi ga `/0.6` | vòng PI quanh `aEgo`, giới hạn trên bằng `vCruise` |
| Lái | `carOutput.actuatorsOutput.steeringAngleDeg` | `actuators.steeringAngleDeg`, giới hạn 0.5°/bước | `actuators.steeringAngleDeg`, giới hạn 0.5°/bước |
| Góc lái báo lại cho openpilot | góc bánh xe thực tế trong CARLA | lặp lại đúng lệnh vừa gửi | góc bánh xe thực tế trong CARLA |
| Engage | tự động | phím 1/2 | tự động (SET rồi nhả), hoặc phím 1/2 |
| Ga/phanh/đánh lái/xi-nhan của người lái gửi lên CAN | có | không (bridge tự tắt engage khi bấm phím) | có: `INTERCEPTOR_GAS`, `BRAKE_PRESSED`, `STEER_TORQUE_SENSOR`, xi-nhan |
| `pandaStates` safety | lấy theo `carParams` | cố định `hondaNidec` | lấy theo `carParams.safetyConfigs` |
| Vật lý xe CARLA | làm phẳng `steering_curve` | `mass=2326`, torque curve phẳng 500 | làm phẳng `steering_curve` |

## Cảm biến và camera

| | A | B | C |
|---|---|---|---|
| IMU | topic `accelerometer`, `gyroscope` | `sensorEvents` với sensor=4, không có timestamp: **locationd bỏ qua toàn bộ** | `sensorEvents` với sensor 1/5, type 1/16, có timestamp: locationd dùng được |
| Hệ trục IMU | đổi sang device frame | thô từ CARLA | đổi sang device frame (như A) |
| GPS | GNSS CARLA, 10 lần mỗi vòng lặp | GNSS CARLA, mỗi tick | GNSS CARLA, 10 Hz |
| Driver monitoring | giả lập `driverStateV2` | giả lập `driverState` | giả lập `driverState` |
| Định dạng ảnh cho modeld | NV12 có padding, `VISION_STREAM_NARROW_ROAD` | I420, `VISION_STREAM_ROAD` | I420, `VISION_STREAM_ROAD` |
| Chuyển màu | libyuv (Cython) | kernel OpenCL (cần pyopencl và ICD của NVIDIA; pocl không build được trên Ryzen Zen 5) | NumPy, trùng từng bit với kernel của B, không cần OpenCL |
| Timestamp ảnh | monotonic thật | `frame_id * 0.05` (gần 0) | monotonic thật |
| Camera | road (thêm wide nếu `--dual_camera`), x=0.4 z=1.22, FOV 40° | road, x=0.8 z=1.13, FOV 40° | road, x=0.8 z=1.13 (chỉnh được), FOV 40° |
| Đồng bộ ảnh với tick | đợi frame đúng tick | callback, không đồng bộ | đợi frame đúng tick |

## Màn hình

| | A | B | C |
|---|---|---|---|
| UI openpilot | UI mới (Python/raylib) | UI Qt của bukapilot | UI Qt của bukapilot |
| Cửa sổ CARLA | có, camera chase bám xe | không (`-RenderOffScreen`) | có, camera chase bám xe (image `carla-sim:0.9.12`) |

## Những gì C giữ nguyên của bukapilot

Không sửa file nào ngoài `tools/sim/`. C dùng cereal 0.8.13, VisionIPC,
manager, controlsd, plannerd, modeld, locationd, CarState Honda và UI Qt của
bukapilot đúng như hiện có. Image Docker và cách build bằng scons của B được
dùng lại.
