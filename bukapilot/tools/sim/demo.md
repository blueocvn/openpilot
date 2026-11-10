Hướng dẫn demo bukapilot trong CARLA
====================================

Chạy demo từ đầu: chuẩn bị, khởi động, chọn map và scenario, điều khiển. Chi tiết
kỹ thuật xem [README.md](README.md).

Mô hình: server CARLA 0.9.12 chạy **native trên Windows**; bukapilot, bridge và UI
chạy trong **WSL Ubuntu**. Mọi lệnh bên dưới chạy trong WSL, ở thư mục
`tools/sim`, trừ lệnh tường lửa (PowerShell Administrator).

## 0. Mở WSL (Ubuntu)

Các lệnh `bash run.sh ...` phải gõ trong cửa sổ **Ubuntu**, không gõ trong PowerShell.
Có 3 cách mở, chọn một:

1. **Start Menu:** bấm phím Windows, gõ `Ubuntu-22.04` rồi Enter.
2. **Từ PowerShell / Windows Terminal:** gõ
   ```powershell
   wsl -d Ubuntu-22.04
   ```
3. **Windows Terminal:** bấm mũi tên xuống cạnh dấu `+` trên thanh tab, chọn `Ubuntu-22.04`.

Dấu nhắc lệnh đổi thành dạng `tên@máy:~$` là đã vào Ubuntu. Ổ Windows nằm dưới `/mnt`:
`D:\` là `/mnt/d`, `E:\` là `/mnt/e`. Vào thư mục demo:

```bash
cd /mnt/d/openpilot/bukapilot/tools/sim
```

Kiểm tra distro dùng WSL2 (gõ trong PowerShell):

```powershell
wsl -l -v
```

Cột `VERSION` của `Ubuntu-22.04` phải là `2`. Thoát về Windows bằng `exit`. Nếu chưa có
Ubuntu-22.04 thì cài bằng `wsl --install -d Ubuntu-22.04` (PowerShell Administrator) rồi
mở lại máy.

Mỗi lệnh `bash run.sh ...` mở trong một cửa sổ Ubuntu; muốn xem log hay chạy
`bash run.sh stop` thì mở thêm một cửa sổ Ubuntu khác.

## 1. Chuẩn bị (làm một lần)

Yêu cầu: Windows 11, WSL2 + WSLg, GPU NVIDIA, khoảng 60 GB trống trên ổ chứa repo.

```bash
cd /mnt/d/openpilot/bukapilot/tools/sim
bash run.sh setup
```

`setup` cài gói apt, tạo venv Python 3.8 (`uv`) từ `Pipfile.lock`, build `libyuv`,
biên dịch bukapilot bằng `scons` và tải CARLA 0.9.12 về `bukapilot/.carla/`.
Có thể chạy lại an toàn nếu bị ngắt. Dùng sẵn bản CARLA đã giải nén:

```bash
CARLA_ROOT=/mnt/e/CARLA/0.9.12/WindowsNoEditor bash run.sh setup
```

(đặt cùng `CARLA_ROOT` cho các lệnh `run.sh` sau đó).

**Mở cổng tường lửa** cho WSL (PowerShell **Administrator**):

```powershell
& 'D:\openpilot\bukapilot\tools\sim\run.ps1' -Command firewall-add -WslDistro Ubuntu-22.04
```

Nếu CARLA đã lên mà bridge không nối được (thường sau khi khởi động lại Windows/WSL
vì IP của WSL đổi), tạo lại rule: `-Command firewall-remove` rồi `-Command firewall-add`.

## 2. Chạy demo

```bash
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 16
```

Thứ tự tự động: mở CARLA trên Windows, chạy bukapilot (manager, modeld, controlsd,
UI), rồi bridge nối CARLA, engage và bắt đầu chạy. Hai cửa sổ hiện ra: **CARLA**
(camera bám sau xe) và **UI bukapilot** (đường đi, làn, xe phía trước, cảnh báo).

| Lệnh | Việc |
|---|---|
| `bash run.sh all [tham số bridge]` | CARLA + bukapilot + bridge |
| `bash run.sh server` | chỉ mở CARLA |
| `bash run.sh bridge [tham số]` | chỉ bridge (CARLA đã chạy) |
| `bash run.sh status` | xem CARLA / manager / bridge |
| `bash run.sh stop` | dừng tất cả |

Biến môi trường (đặt trước `bash run.sh`):

| Biến | Ý nghĩa |
|---|---|
| `CARLA_HEADLESS=1` | CARLA không mở cửa sổ |
| `CARLA_QUALITY=Low\|High` | chất lượng render (mặc định `Low`) |
| `CARLA_PORT`, `CARLA_HOST` | cổng và địa chỉ server (mặc định 2000, gateway của WSL) |
| `CARLA_ROOT` | thư mục CARLA đã giải nén sẵn |
| `SKIP_BUILD=1` | bỏ qua `scons` |
| `CARLA_SPECTATOR_MODE=free` | tự điều khiển góc nhìn CARLA |

## 3. Tham số chọn map và scenario

Truyền sau `bash run.sh all`:

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `--carla-town` | `Town04_Opt` | tên map CARLA |
| `--carla-spawn-point` | `16` | chỉ số điểm xuất phát |
| `--weather` | `ClearNoon` | `ClearNoon`, `CloudyNoon`, `WetNoon`, `MidRainyNoon`, `HardRainNoon`, `ClearSunset`, ... |
| `--vehicle` | `vehicle.tesla.model3` | xe của mình |
| `--low_quality` | tắt | gỡ cây, nhà, props cho nhẹ máy |
| `--motorcycle` | tắt | thêm xe máy phía trước cùng làn (xem bên dưới) |
| `--no-auto-engage` | tự engage | tự bấm `2` để engage |
| `--duration N` | không | chạy N giây không cần bàn phím |
| `--joystick` | tắt | điều khiển bằng vô-lăng/tay cầm |

(`--town` và `--spawn_point` là tên cũ, vẫn dùng được.)

### Scenario và map

**Đã hoàn thiện (2 kịch bản):**

| Scenario | Map | `--carla-spawn-point` | Lệnh |
|---|---|---|---|
| Đường cong cao tốc (mặc định) | `Town04_Opt` | 16 | `bash run.sh all --carla-town Town04_Opt --carla-spawn-point 16` |
| Đường chữ L | `Town05_Opt` | 1 | `bash run.sh all --carla-town Town05_Opt --carla-spawn-point 1` |

Lệnh mặc định `bash run.sh all` (không tham số) chạy đúng kịch bản đường cong cao tốc ở trên.

**Thẳng tắp** dùng `Town04_Opt` điểm 170 (đã chạy ổn). `Town06`/`Town07` nằm trong gói AdditionalMaps,
không có trong bản CARLA cơ bản nên không dùng. Lưu ý: chỉ số điểm xuất phát khác giữa CARLA 0.9.12
và 0.9.16, ví dụ điểm 40 của `Town04_Opt` là đường cong ở 0.9.12 chứ không phải đường thẳng.

| Scenario | Map | `--carla-spawn-point` | Lệnh |
|---|---|---|---|
| Thẳng tắp | `Town04_Opt` | 170 | `bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170` |

Các điểm khác cùng dạng (đổi hướng dưới 1° trong 300 m): 119–121, 150–153, 155–157, 174–176, 263–265.
Đường cong khác (không có giao lộ): 17–19, 343–346 (cong khoảng 70°).

**Nhiều xe máy** (đã viết, **chưa chạy thử với CARLA**): nhiều xe máy cùng chiều và ngược chiều, mỗi xe một
tốc độ, để xem bukapilot xử lý ra sao. Do Traffic Manager của CARLA điều khiển (không đổi làn, bỏ qua đèn).
Thêm `--motorcycle` vào bất kỳ kịch bản nào:

| Scenario | Lệnh |
|---|---|
| Nhiều xe máy trên đường thẳng | `bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 --motorcycle` |
| Nhiều xe máy trên đường cong | `bash run.sh all --carla-town Town04_Opt --carla-spawn-point 16 --motorcycle` |
| Tự định nghĩa | `... --motorcycle-spec "same:40:25,oncoming:120:55"` |

`--motorcycle` dùng bộ mặc định (7 xe):
`same:35:20, same:75:45, left:25:60, right:50:35, oncoming:90:50, oncoming:160:30, oncoming:230:60`.

Mỗi phần tử của `--motorcycle-spec` là `loại:khoảng_cách_m:tốc_độ_kmh`:

| Loại | Ý nghĩa |
|---|---|
| `same` | cùng làn với xe mình, phía trước |
| `left` / `right` | làn bên cạnh, cùng chiều |
| `oncoming` | làn ngược chiều, đi về phía xe mình |

`khoảng_cách_m` là số mét phía trước điểm xuất phát của xe mình (dọc hướng xe mình chạy). Nếu điểm xuất phát
không có làn đó (ví dụ không có làn ngược chiều), xe máy đó bị bỏ qua và bridge in thông báo `skipped`.
`--motorcycle-model` ép dùng một mẫu xe cho tất cả (mặc định xoay vòng `kawasaki.ninja`, `yamaha.yzf`,
`harley-davidson.low_rider`). Phím `r` reset cả các xe máy về chỗ cũ.

Các map khác: `Town01_Opt`, `Town02`, `Town03`, `Town10HD_Opt`.

**Chọn `<N>`.** Liệt kê các điểm xuất phát của một map rồi chọn điểm nằm đúng đoạn
đường cần thử (CARLA đang chạy):

```bash
~/.cache/bukapilot-carla/venv/bin/python - <<'PY'
import os, carla
c = carla.Client(os.environ.get("CARLA_HOST", "127.0.0.1"), 2000); c.set_timeout(20)
print(c.get_available_maps())
w = c.load_world("Town06")            # đổi tên map ở đây
for i, sp in enumerate(w.get_map().get_spawn_points()):
    print(i, sp.location, sp.rotation.yaw)
PY
```

(`CARLA_HOST` lấy bằng `ip route show default | awk '{print $3}'`.)

## 4. Điều khiển khi chạy

| Phím | Chức năng |
|---|---|
| 1 / 2 / 3 | cruise RES+ / SET- / cancel |
| w a s d | ga / lái trái / phanh / lái phải |
| z / x | xi-nhan trái / phải |
| h | giữ / thả xe |
| i | bật/tắt ignition |
| r | reset về điểm xuất phát |
| q | thoát |

Bấm phím khi cửa sổ terminal đang chạy `run.sh` được focus.

Log: `bukapilot/.carla/logs/` (`bridge.log`, `bukapilot.log`, `CarlaUE4.log`).

## 5. Dừng demo

Bấm `q` trong terminal chạy `run.sh`, hoặc `bash run.sh stop`.
