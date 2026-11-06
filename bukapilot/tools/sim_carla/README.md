bukapilot trong CARLA (`tools/sim_carla`)
=========================================

Đây là bản port bridge `openpilot/tools/sim` (openpilot đời mới) sang bukapilot
(nền openpilot 0.8.13, Python 3.8). Cấu trúc giữ nguyên như upstream:
`run_bridge.py`, `bridge/common.py` (`SimulatorBridge`), `bridge/carla_world.py`
(`CarlaWorld`), và `lib/` gồm `simulated_car`, `simulated_sensors`, `camerad`,
`keyboard_ctrl`. Code chỉ đổi những chỗ gọi API để khớp với cereal, CAN và
VisionIPC của bukapilot. Bộ máy của bukapilot (manager, controlsd, modeld, UI Qt)
chạy nguyên bản.

`tools/sim` (bridge cũ) vẫn giữ nguyên, dùng cái nào cũng được (nhưng không chạy cùng lúc:
hai bên dùng chung tên topic msgq). Bảng so sánh xem
ở [COMPARISON.md](COMPARISON.md).

## Chạy

Dùng 2 image Docker (giống `tools/sim`):
* `bukapilot-sim:local`: Ubuntu 20.04, Python 3.8, client carla 0.9.12.
  Build một lần từ gốc bukapilot:
  `docker build -t bukapilot-sim:local -f tools/sim/Dockerfile.bukapilot .`
* CARLA server 0.9.12. Lần đầu `start_carla.sh` tự build image `carla-sim:0.9.12`
  từ `Dockerfile.carla` để mở được cửa sổ CARLA.

```bash
cd tools/sim_carla
./start_carla.sh        # terminal 1: cửa sổ CARLA nhìn xe từ phía sau
./start_docker.sh       # terminal 2: scons + bukapilot + bridge, mở UI Qt
```

Hai cửa sổ hiện ra (giống cách openpilot chạy CARLA):
1. **CARLA**: camera chase bám sau xe (`CARLA_SPECTATOR_MODE=free` để tự điều
   khiển góc nhìn).
2. **UI bukapilot**: camera trên xe kèm đường đi, làn, xe phía trước, cảnh báo
   và tốc độ đặt.

Xe tự engage khi bukapilot sẵn sàng (bấm SET rồi nhả). Muốn engage bằng tay thì
dùng `--no-auto-engage`.

| Biến / tham số | Ý nghĩa |
|---|---|
| `CARLA_HEADLESS=1 ./start_carla.sh` | không mở cửa sổ CARLA (`-RenderOffScreen`) |
| `CARLA_QUALITY=Low` | chất lượng render (mặc định `High`) |
| `CARLA_DETACH=1` | chạy CARLA ngầm (`docker logs -f carla_server`) |
| `--town`, `--spawn_point` | map và điểm xuất phát (mặc định `Town04_Opt`, 16) |
| `--low_quality` | gỡ cây, nhà, props khỏi map để nhẹ máy |
| `--duration 120` | chạy không cần bàn phím, thoát sau 120 s (smoke test) |
| `--joystick` | đọc vô-lăng/tay cầm `/dev/input/js0` |
| `SKIP_BUILD=1` / `SHELL_ONLY=1` | bỏ qua scons / chỉ mở shell trong container |
| `CARLA_CAMERA_X`, `CARLA_CAMERA_Z` | vị trí camera (mặc định 0.8 / 1.13 m như bridge cũ) |

Log của bukapilot nằm ở `.carla/logs/bukapilot.log` (trong gốc bukapilot).

## Phím

| phím | chức năng |
|---|---|
| 1 / 2 / 3 | cruise RES+ / SET- / cancel |
| w a s d | ga / lái trái / phanh / lái phải (đè lên openpilot) |
| z / x | xi-nhan trái / phải |
| h | giữ / thả xe |
| i | bật/tắt ignition |
| r | reset về điểm xuất phát |
| q | thoát |

## Test

```bash
docker run --rm --gpus all -u $(id -u):$(id -g) -e HOME=/home/sim -e PYTHONPATH=$PWD \
  -v $PWD:$PWD -w $PWD bukapilot-sim:local python3 -m unittest tools.sim_carla.tests.test_sim_carla -v
```
(chạy ở gốc bukapilot). Test kiểm tra:
* chuyển màu BGRA→I420 bằng NumPy cho ra kết quả trùng từng bit với kernel
  OpenCL `rgb_to_yuv.cl` của bridge cũ;
* CAN giả lập được chính `CarState` Honda của bukapilot đọc đúng: số D, tốc độ,
  góc lái, cruise available, có comma pedal, và nhận các thao tác ga, phanh,
  đánh lái, xi-nhan.

## Hạn chế

* Thân xe trong CARLA là Tesla Model 3, nhưng bukapilot thấy một chiếc Honda
  Civic 2016 qua CAN (giống bridge cũ).
* Hệ số ga/phanh (`bridge/carla_bridge.py`) lấy theo upstream (CARLA 0.9.16).
  Vòng PI quanh `carState.aEgo` bù phần chênh, nhưng chưa tinh chỉnh lại cho 0.9.12.
* Phần joystick port nguyên từ bản cũ, chưa thử với phần cứng thật.
