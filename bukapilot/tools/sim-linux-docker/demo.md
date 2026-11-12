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

# Thử đổi làn để vượt một xe máy (xem mục "Đổi làn bằng xi-nhan" bên dưới)
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 --motorcycle-spec "same:40:60" --cruise-kph 80

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
| `ALC_MIN_SPEED_KPH` | km/h | `30` | tốc độ thấp nhất để ALC đổi làn trong mô phỏng. Xe thật không đặt biến này nên vẫn là 30 mph (48,3 km/h) |
| `CARLA_KEYS` | `0` / `1` | `1` | `0`: không đọc phím từ cửa sổ CARLA, chỉ dùng terminal |
| `CARLA_WINDOW_TITLE` | danh sách cách nhau bằng dấu phẩy | `CarlaUE4,=UI` | tên cửa sổ nhận phím: `CarlaUE4` khớp một phần tên, `=UI` khớp đúng tên `UI` |
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

Phím dùng được ngay trong cửa sổ CARLA và cửa sổ `UI` của openpilot (bấm vào một trong hai rồi gõ), và vẫn dùng được ở terminal.
Phím gõ ở cửa sổ khác không có tác dụng. Chỉ chạy trên Docker/X11; `CARLA_KEYS=0` để tắt.

| Phím | Chức năng |
|---|---|
| 1 / 2 / 3 | cruise RES+ / SET- / cancel |
| w a s d | ga / lái trái / phanh / lái phải (giữ phím được) |
| z / x | xi-nhan trái / phải. Bấm một lần để bật và giữ, bấm lại để tắt, bấm phím bên kia để đổi bên |
| h | giữ / thả xe |
| i | bật/tắt ignition |
| r | reset về điểm xuất phát |
| q | thoát |

## Đổi làn bằng xi-nhan

Xe dùng tính năng đổi làn có hỗ trợ (ALC) sẵn có của bukapilot, chưa có chế độ tự quyết định vượt.

1. Bấm `z` (trái) hoặc `x` (phải): xi-nhan bật và được giữ.
2. Khi xe đang chạy bằng openpilot, bấm `a` (trái) hoặc `d` (phải) cùng hướng xi-nhan để "đánh lái nhẹ".
   Lúc này phím chỉ gửi lực tay lái, còn openpilot tự lái và đổi làn. Bấm ngược hướng, hoặc khi
   không có xi-nhan, phím vẫn lái tay như trước.
3. Xi-nhan không tự tắt: đổi làn xong, bấm `z` / `x` lần nữa để tắt. Khi xi-nhan còn bật, bấm `a` / `d`
   cùng hướng sẽ đổi làn thêm lần nữa.
4. Khi chưa thể đổi làn (ví dụ xe chạy dưới tốc độ tối thiểu), `a` / `d` vẫn lái tay bình thường dù xi-nhan
   đang bật.

**Điều kiện:** trong mô phỏng, ALC chỉ bắt đầu khi xe chạy từ 30 km/h trở lên (biến `ALC_MIN_SPEED_KPH`,
mặc định `30`). Trên xe thật mức này vẫn là 30 mph, khoảng 48,3 km/h (`LANE_CHANGE_SPEED_MIN` trong
`selfdrive/controls/lib/desire_helper.py`). Dưới mức này xi-nhan còn làm xe ngừng tự lái ngang. Xe phải
được phép chạy nhanh hơn xe máy thì mới vượt được, nên đặt tốc độ cruise cao hơn tốc độ xe máy, ví dụ
80 km/h: thêm `--cruise-kph 80` vào lệnh (sau khi engage, bridge tự giữ nút RES+ để đưa `vCruise` lên mức đó; engage lúc xe đứng yên chỉ đặt 40 km/h). Có thể bấm `1` tay, mỗi lần chỉ +1 km/h, giữ lâu hơn 0,5 s thì +5 km/h; xem dòng `vCruise=` trong log.
Mô phỏng này không báo điểm mù: tự nhìn làn bên trước khi đổi làn, và không đặt xe máy ở làn trái khi thử.

**Khoảng cách trước khi vượt:** xe không tự giảm tốc khi xi-nhan bật, vì giảm tốc đột ngột có thể gây nguy
hiểm cho xe phía sau. Người lái tự mở khoảng cách: bấm `2` để hạ tốc độ cruise (mỗi lần -1 km/h, giữ lâu
hơn 0,5 s thì -5 km/h), chờ khoảng cách đủ rộng, bấm `1` để tăng lại rồi mới đổi làn. Phím `s` (phanh) cũng
giảm tốc nhưng làm openpilot ngắt; bấm `2` để bật lại. Trong lúc chuyển làn xe vẫn bám theo xe máy ở khoảng
cách bình thường cho đến khi xe máy ra khỏi đường đi của xe, rồi mới tăng lên tốc độ cruise.
