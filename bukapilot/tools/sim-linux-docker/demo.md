# Lệnh demo bukapilot + CARLA (Docker)

```bash
cd /home/ubuntu/openpilot/bukapilot/tools/sim-linux-docker
```

## Chuẩn bị (một lần)

```bash
bash run.sh setup        # build image bukapilot-sim:local, carla-sim:0.9.12
bash run.sh setup-ui     # tùy chọn: UI openpilot (OP_UI=1)
```

## Tình huống

```bash
# Đường cong cao tốc (mặc định)
bash run.sh all
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 16

# Đường chữ L
bash run.sh all --carla-town Town05_Opt --carla-spawn-point 1

# Đường thẳng tắp
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170

# Nhiều xe máy - đường thẳng
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 --motorcycle

# Nhiều xe máy - đường cong
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 16 --motorcycle

# Xe máy tự định nghĩa (loại:khoảng_cách_m:tốc_độ_kmh; loại = same|left|right|oncoming)
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 --motorcycle-spec "same:40:25,oncoming:120:55"

# Map khác
bash run.sh all --carla-town Town03 --carla-spawn-point <N>
```

## Tùy chọn (đặt sau `bash run.sh all` hoặc `bash run.sh bridge`)

| Tùy chọn | Giá trị | Mặc định | Ý nghĩa |
|---|---|---|---|
| `--carla-town` (cũ: `--town`) | `Town04_Opt`, `Town05_Opt`, `Town01_Opt`, `Town02`, `Town03`, `Town10HD_Opt` | `Town04_Opt` | map CARLA |
| `--carla-spawn-point` (cũ: `--spawn_point`) | số nguyên `0 .. (số điểm của map - 1)`, vd `Town04_Opt`: 16 cong, 170 thẳng; `Town05_Opt`: 1 chữ L | `16` | điểm xuất phát; ngoài khoảng sẽ báo lỗi |
| `--weather` | `ClearNoon`, `CloudyNoon`, `WetNoon`, `WetCloudyNoon`, `SoftRainNoon`, `MidRainyNoon`, `HardRainNoon`, các bản `...Sunset` và `...Night` tương ứng (`MidRainSunset` không có `Rainy`), `Default` | `ClearNoon` | thời tiết |
| `--vehicle` | blueprint xe của CARLA, vd `vehicle.tesla.model3` | `vehicle.tesla.model3` | xe của mình |
| `--low_quality` | cờ (không có giá trị) | tắt | gỡ cây, nhà, xe đỗ, props, đèn đường, hạt cho nhẹ máy |
| `--motorcycle` | cờ | tắt | thêm bộ 7 xe máy mặc định |
| `--motorcycle-spec` | `loại:khoảng_cách_m:tốc_độ_kmh,...`; loại = `same` / `left` / `right` / `oncoming` | bộ mặc định: `same:35:20,same:75:45,left:25:60,right:50:35,oncoming:90:50,oncoming:160:30,oncoming:230:60` | xe máy tự định nghĩa (tự bật `--motorcycle`) |
| `--motorcycle-model` | `vehicle.kawasaki.ninja`, `vehicle.yamaha.yzf`, `vehicle.harley-davidson.low_rider` | xoay vòng cả ba | ép một mẫu xe máy cho tất cả |
| `--no-auto-engage` | cờ | tự engage | không tự bấm SET; engage bằng phím `2` (hoặc `1`) |
| `--duration` | số giây, vd `60` | không đặt (chạy đến khi bấm `q`) | chạy không cần bàn phím rồi tự thoát |
| `--joystick` | cờ | tắt (điều khiển bằng bàn phím) | dùng vô-lăng/tay cầm `/dev/input/js0` |
| `--host` (`--carla-host`) | địa chỉ IP | `127.0.0.1` (hoặc `$CARLA_HOST`) | server CARLA |
| `--port` (`--carla-port`) | số cổng | `2000` (hoặc `$CARLA_PORT`) | cổng server CARLA |
| `--yellow-path` / `--no-yellow-path` | cờ (do `run.sh` xử lý, không gửi cho bridge) | tắt | đường đi vàng trong UI openpilot (`OP_UI=1`) |

Ví dụ kết hợp:

```bash
bash run.sh all --carla-town Town05_Opt --carla-spawn-point 1 --weather MidRainyNoon --motorcycle --yellow-path
```

## Biến môi trường

**Đặt biến TRƯỚC `bash run.sh`**, không đặt sau (sau lệnh sẽ bị coi là tham số của bridge và báo
`unrecognized arguments`):

```bash
# đúng
CARLA_QUALITY=Low YELLOW_PATH=1 bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 --motorcycle
# sai: bash run.sh all ... CARLA_QUALITY=Low YELLOW_PATH=1
```

| Biến | Giá trị | Mặc định | Ý nghĩa |
|---|---|---|---|
| `CARLA_HEADLESS` | `0` / `1` | `0` (tự chuyển sang `1` nếu không có `DISPLAY`) | `1`: CARLA chạy off-screen, không mở cửa sổ |
| `CARLA_QUALITY` | `Low` / `High` | `High` | chất lượng render của CARLA |
| `CARLA_PORT` | số cổng | `2000` | cổng server CARLA |
| `CARLA_HOST` | địa chỉ IP | `127.0.0.1` | server CARLA mà bridge nối tới |
| `CARLA_RES_X`, `CARLA_RES_Y` | số điểm ảnh | `1280`, `720` | kích thước cửa sổ CARLA (không áp dụng khi headless) |
| `CARLA_SPECTATOR_MODE` | `chase` / `free` | `chase` | `chase`: góc nhìn bám sau xe; `free`: tự điều khiển góc nhìn |
| `CARLA_CAMERA_X`, `CARLA_CAMERA_Z` | mét | `0.8`, `1.13` | vị trí camera đường của xe (tiến, cao) |
| `CARLA_TM_PORT` | số cổng | một cổng trống tự chọn | cổng Traffic Manager cho xe máy |
| `CARLA_NO_ASSET_PATCH` | `0` / `1` | `0` | `1`: dùng asset CARLA gốc, bỏ logo BlueOC trong `carla-assets/` |
| `OP_UI` | `0` / `1` | `1` nếu có `/home/ubuntu/openpilot/.venv`, không thì `0` | `1`: UI openpilot trên host; `0`: UI Qt của bukapilot |
| `OP_UI_PORT` | số cổng | `8765` | cổng relay giữa bukapilot và UI openpilot |
| `YELLOW_PATH` | `0` / `1` | `0` | `1`: đường đi màu vàng, giống `--yellow-path` (chỉ có tác dụng với `OP_UI=1`) |
| `SKIP_BUILD` | `1` | không đặt | bỏ qua `scons` khi khởi động bridge |
| `SHELL_ONLY` | `1` | không đặt | chỉ mở shell trong container bukapilot |
| `IMAGE` | tên image | `bukapilot-sim:local` | image chạy bukapilot và bridge |
| `CARLA_IMAGE` | tên image | `carla-sim:0.9.12` (headless: `carlasim/carla:0.9.12`) | image server CARLA |
| `CARLA_NAME`, `NAME` | tên container | `carla_server`, `bukapilot_sim_carla` | tên container CARLA và bukapilot |

## Chạy từng phần / quản lý

```bash
bash run.sh server                                              # chỉ CARLA
bash run.sh bridge --carla-town Town04_Opt --carla-spawn-point 16   # chỉ bukapilot + bridge
bash run.sh status
bash run.sh stop
docker logs carla_server
```

## Phím điều khiển

| Phím | Chức năng |
|---|---|
| 1 / 2 / 3 | cruise RES+ / SET- / cancel |
| w a s d | ga / lái trái / phanh / lái phải |
| z / x | xi-nhan trái / phải |
| h | giữ / thả xe |
| i | bật/tắt ignition |
| r | reset về điểm xuất phát |
| q | thoát |
