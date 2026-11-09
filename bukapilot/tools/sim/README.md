bukapilot trong CARLA (`tools/sim`)
=========================================

Đây là bản port bridge `openpilot/tools/sim` (openpilot đời mới) sang bukapilot
(nền openpilot 0.8.13, Python 3.8). Cấu trúc giữ nguyên như upstream:
`run_bridge.py`, `bridge/common.py` (`SimulatorBridge`), `bridge/carla_world.py`
(`CarlaWorld`), và `lib/` gồm `simulated_car`, `simulated_sensors`, `camerad`,
`keyboard_ctrl`. Code chỉ đổi những chỗ gọi API để khớp với cereal, CAN và
VisionIPC của bukapilot. Bộ máy của bukapilot (manager, controlsd, modeld, UI Qt)
chạy nguyên bản.

Bridge Docker cũ (`bridge.py`, `start_bukapilot_docker.sh`) đã bị xóa; thư mục này
thay thế nó. Bảng so sánh xem ở [COMPARISON.md](COMPARISON.md).

## Chạy

Không dùng Docker. Cấu trúc theo `openpilot/tools/sim/carla`: server CARLA 0.9.12
chạy **native trên Windows**, bukapilot + bridge chạy trong **WSL Ubuntu** (venv
Python 3.8 do `uv` tạo). Một điểm vào duy nhất: `run.sh` (gọi `run.ps1` để điều
khiển phía Windows).

```bash
wsl -d Ubuntu-22.04
cd /mnt/d/openpilot/bukapilot/tools/sim     # trong WSL
bash run.sh setup                                  # một lần: gói apt, venv 3.8, libyuv, scons, CARLA
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 16
```

Lần đầu cần mở cổng tường lửa cho WSL (PowerShell **Administrator**):

```powershell
& 'D:\openpilot\bukapilot\tools\sim\run.ps1' -Command firewall-add -WslDistro Ubuntu-22.04
```

Sau khi Windows/WSL khởi động lại mà bridge không nối được CARLA, tạo lại rule
(`-Command firewall-remove` rồi `firewall-add`), vì IP của WSL đổi.

| Lệnh | Việc |
|---|---|
| `bash run.sh setup` | cài gói, venv Python 3.8, build libyuv, `scons`, tải CARLA |
| `bash run.sh all [tham số bridge]` | CARLA + bukapilot + bridge |
| `bash run.sh server` / `bridge [tham số]` | chỉ một nửa |
| `bash run.sh status` / `stop` | xem / dừng |

Hai cửa sổ hiện ra (giống cách openpilot chạy CARLA):
1. **CARLA**: camera chase bám sau xe (`CARLA_SPECTATOR_MODE=free` để tự điều
   khiển góc nhìn).
2. **UI bukapilot** (qua WSLg): camera trên xe kèm đường đi, làn, xe phía trước,
   cảnh báo và tốc độ đặt.

Xe tự engage khi bukapilot sẵn sàng (bấm SET rồi nhả). Muốn engage bằng tay thì
dùng `--no-auto-engage`. Danh sách scenario và map xem [demo.md](demo.md).

| Biến / tham số | Ý nghĩa |
|---|---|
| `CARLA_HEADLESS=1` | không mở cửa sổ CARLA (`-RenderOffScreen`) |
| `CARLA_QUALITY=Low\|High` | chất lượng render (mặc định `Low`) |
| `CARLA_ROOT=/mnt/e/CARLA/0.9.12/WindowsNoEditor` | dùng bản CARLA đã giải nén sẵn thay vì tải |
| `CARLA_HOST`, `CARLA_PORT` | địa chỉ server (mặc định gateway của WSL, cổng 2000) |
| `SKIP_BUILD=1` | bỏ qua `scons` |
| `CARLA_CAMERA_X`, `CARLA_CAMERA_Z` | vị trí camera (mặc định 0.8 / 1.13 m như bridge cũ) |

Tham số truyền cho bridge (sau `bash run.sh all`):

| Tham số | Ý nghĩa |
|---|---|
| `--carla-town`, `--carla-spawn-point` | map và điểm xuất phát (mặc định `Town04_Opt`, 16); viết cũ `--town`, `--spawn_point` vẫn dùng được |
| `--weather` | thời tiết (mặc định `ClearNoon`) |
| `--low_quality` | gỡ cây, nhà, props khỏi map để nhẹ máy |
| `--duration 120` | chạy không cần bàn phím, thoát sau 120 s (smoke test) |
| `--joystick` | đọc vô-lăng/tay cầm `/dev/input/js0` |

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
cd /mnt/d/openpilot/bukapilot          # gốc bukapilot, trong WSL
PYTHONPATH=$PWD ~/.cache/bukapilot-carla/venv/bin/python -m unittest tools.sim.tests.test_sim_carla -v
```
Test kiểm tra:
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
