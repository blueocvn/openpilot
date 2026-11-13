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
| `--obstacle-spec` | `loại:khoảng_cách_m[:tốc_độ_kmh],...`; loại = `motorcycle` / `car` / `car-across` / `barrier` / `container` | không đặt | vật cản trong làn của xe mình, cách điểm xuất phát `khoảng_cách_m` theo đường. Vật đứng yên; riêng `motorcycle` có thể kèm tốc độ để chạy cùng chiều. Ví dụ `container:150`. Chấm điểm tự động nhiều lần chạy: xem [`bench.md`](bench.md) |
| `--cruise-kph` | km/h | không đặt (engage lúc đứng yên là 40) | sau khi engage, tự bấm nút để đưa tốc độ cài đặt về giá trị này (tăng hoặc giảm) |
| `--no-auto-engage` | cờ | tự engage | không tự bấm SET; engage bằng phím `2` (hoặc `1`) |
| `--duration` | số giây, vd `60` | không đặt (chạy đến khi bấm `q`) | chạy **không có bàn phím** rồi tự thoát; chỉ dùng cho kịch bản tự động, không dùng khi cần bấm `z`, `a`, `x`... |
| `--joystick` | cờ | tắt (điều khiển bằng bàn phím) | dùng vô-lăng/tay cầm `/dev/input/js0` |
| `--host` (`--carla-host`) | địa chỉ IP | `127.0.0.1` (hoặc `$CARLA_HOST`) | server CARLA |
| `--port` (`--carla-port`) | số cổng | `2000` (hoặc `$CARLA_PORT`) | cổng server CARLA |
| `--yellow-path` / `--no-yellow-path` | cờ (do `run.sh` xử lý, không gửi cho bridge) | tắt | đường đi vàng trong UI openpilot (`OP_UI=1`) |
| `--lead-log` / `--no-lead-log` | cờ (do `run.sh` xử lý, không gửi cho bridge) | tắt | log chi tiết: xe thấy gì và thực tế ra sao, xem mục "Log kiểm tra khoảng cách" |

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
| `LEAD_LOG` | `0` / `1` | `0` | `1`: log chi tiết về xe phía trước, giống `--lead-log` |
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
| z | xi-nhan trái, bật và giữ (bấm lại vẫn giữ nguyên) |
| x | về giữa: tắt xi-nhan |
| c | xi-nhan phải, bật và giữ (bấm lại vẫn giữ nguyên) |
| h | giữ / thả xe |
| i | bật/tắt ignition |
| r | reset về điểm xuất phát |
| q | thoát |

## Đổi làn bằng xi-nhan

Xe dùng tính năng đổi làn có hỗ trợ (ALC) sẵn có của bukapilot, chưa có chế độ tự quyết định vượt.

1. Bấm `z` (trái) hoặc `c` (phải): xi-nhan bật và được giữ.
2. Khi xe đang chạy bằng openpilot, bấm `a` (trái) hoặc `d` (phải) cùng hướng xi-nhan để "đánh lái nhẹ".
   Lúc này phím chỉ gửi lực tay lái, còn openpilot tự lái và đổi làn. Bấm ngược hướng, hoặc khi
   không có xi-nhan, phím vẫn lái tay như trước.
3. Xi-nhan không tự tắt: đổi làn xong, bấm `x` (về giữa) để tắt. Khi xi-nhan còn bật, bấm `a` / `d`
   cùng hướng sẽ đổi làn thêm lần nữa.
4. Khi chưa thể đổi làn (ví dụ xe chạy dưới tốc độ tối thiểu), `a` / `d` vẫn lái tay bình thường dù xi-nhan
   đang bật. Phím lái chỉ lấy vô lăng; ga và phanh vẫn do openpilot giữ (còn giữ khoảng cách, còn phanh cho xe máy phía trước). Muốn tự đạp ga hoặc phanh thì dùng `w` / `s`.

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

## Log kiểm tra khoảng cách với xe phía trước

Mặc định bridge in một dòng ngắn mỗi giây (`engaged=... speed=... throttle=...`). Để kiểm tra xe có giữ
được khoảng cách với xe máy phía trước hay không, bật log chi tiết:

```bash
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 --motorcycle --lead-log
LEAD_LOG=1 bash run.sh all ...        # cùng ý nghĩa; --no-lead-log để tắt
```

Mỗi dòng tách rõ thông tin do openpilot thu và thông tin do mô phỏng thu:

```
[OP cmd] engaged=True vCruise=80kph plan=lead0 accel=+0.40 throttle=0.67 brake=0.00 steer=-1.3 fcw=False alert=''
| [OP sees] lead dRel=9.4m vLead=12.0 prob=1.00
| [SIM] speed=11.2 lead gap=1.7m vLead=10.6 closing=+0.6 ttc=2.8s
| [ERR] dRel=+7.7m vLead=+1.4
```

| Nhãn | Do ai thu | Nội dung |
|---|---|---|
| `[OP cmd]` | openpilot | lệnh của openpilot: `plan` là nguồn kế hoạch (`lead0`/`lead1` bám xe phía trước, `cruise` chạy theo tốc độ cài đặt), `accel` là gia tốc yêu cầu, `fcw` cảnh báo va chạm. `throttle`/`brake` là ga và phanh mà bridge suy ra từ `accel` |
| `[OP sees]` | openpilot (camera + model) | xe phía trước mà openpilot nhìn thấy: `dRel` khoảng cách, `vLead` vận tốc, `prob` xác suất của model. `NO LEAD` nghĩa là openpilot không thấy xe nào |
| `[SIM]` | CARLA | số thật: `speed` tốc độ xe mình, `gap` khoảng cách giữa hai đầu xe tới xe máy gần nhất trong làn, `vLead` vận tốc xe máy, `closing` tốc độ tiến lại gần (dương là đang tới gần), `ttc` thời gian tới va chạm (`CONTACT` là đã chạm, `-` là không đang tới gần) |
| `[ERR]` | tính ra | openpilot thấy trừ số thật. `dRel` dương là openpilot tưởng xe máy ở xa hơn thực tế; `lead missed` là openpilot bỏ lỡ xe máy |

Lưu ý khi đọc:

- Số `[SIM]` chỉ được in ra, không bao giờ đưa vào openpilot: xe thật chỉ có camera nên mô phỏng cũng vậy.
- `[SIM]` và `[ERR]` chỉ xuất hiện khi có xe máy trong làn của xe mình (lệch ngang tối đa 2 m); không có thì chỉ in `[OP cmd]`, `[OP sees]` và `speed`.
- `dRel` của openpilot tính từ camera trừ 1,52 m, còn `gap` tính giữa hai đầu xe, nên hai số có thể chênh 1 đến 2 m ngay cả khi model đúng. Nên xem xu hướng, không so từng mét.
- Tần suất: mỗi giây một dòng, và 10 dòng mỗi giây khi khoảng cách thật dưới 40 m (`CLOSE_LEAD_LOG_M` trong `tools/sim/bridge/common.py`).
- Khi xe của mình va chạm, log in một khối chữ HOA có viền `#` (nội dung: va chạm với vật gì, tốc độ xe, lực va chạm và openpilot lúc đó thấy lead nào hoặc không thấy lead nào). Sự kiện này lấy từ cảm biến va chạm của CARLA, không phụ thuộc vào `gap`, và chỉ in tối đa một khối mỗi giây dù xe vẫn đang tì vào vật cản.
- Khi khoảng cách thật dưới 15 m, bridge lưu thêm khung hình camera mà model thực sự nhận (JPEG, đóng dấu `SIM gap=... | OP dRel=...` ở đầu ảnh) vào `.carla/lead-frames/` (đổi bằng biến `LEAD_FRAME_DIR`), tối đa 2 ảnh mỗi giây và 200 ảnh mỗi lần chạy. Tên file có số thứ tự, khoảng cách thật và khoảng cách openpilot thấy, ví dụ `0003_gap008.1_op23.2.jpg`.
- Cách nhận biết vấn đề: `NO LEAD` trong khi `gap` còn nhỏ nghĩa là model bỏ lỡ xe máy; `plan=cruise` và `accel` dương khi xe máy ở gần nghĩa là planner không coi nó là vật cản.

## Kịch bản: nhiều xe máy làm vật cản giữa đường

Kiểm tra ba việc của xe: dừng trước vật cản, giữ khoảng cách với xe phía trước, và đánh lái tránh vật cản.
Kịch bản 1 đến 4 chạy trên đường thẳng `Town04_Opt` điểm 170, kịch bản 5 chạy trên `Town01_Opt`. Tất cả bật `--lead-log`
để so cái openpilot thấy với số thật của CARLA (cách đọc log: xem mục "Log kiểm tra khoảng cách" ở trên).

`--obstacle-spec` đặt xe máy **đúng giữa làn của xe mình**, `khoảng_cách_m` tính từ điểm xuất phát theo đường. Xe máy
không có tốc độ thì đứng yên; có tốc độ thì chạy thẳng với tốc độ không đổi. Mỗi xe máy rộng chừng 0,8 m và dài khoảng
2 m, nên đặt cách nhau 4 đến 6 m là một hàng xe nối đuôi.

Hai giới hạn cần biết:

- Xe máy chạy (`motorcycle:khoảng_cách:km/h`) không phanh và không né, nên đi xuyên qua xe máy đứng yên nếu chạm tới.
  Không đặt xe máy đứng yên ở nơi xe máy chạy sẽ tới trong thời gian chạy; hai kịch bản bám theo và dừng bên dưới
  vì vậy được tách riêng.
- Vật cản chỉ nằm ở giữa làn xe mình. Làn bên cạnh và làn ngược chiều dùng `--motorcycle-spec`
  (`left` / `right` / `oncoming`).

### Ý nghĩa từng phần của lệnh

| Phần của lệnh | Ý nghĩa |
|---|---|
| `--carla-town Town04_Opt` | map CARLA. `Town04_Opt` là cao tốc nhiều làn có dải phân cách; `Town01_Opt` là đường hai chiều hai làn |
| `--carla-spawn-point 170` | điểm xuất phát của xe mình trên map đó. 170 của `Town04_Opt` là đoạn thẳng khoảng 450 m; 181 của `Town01_Opt` là đoạn thẳng khoảng 340 m, làn bên trái là làn ngược chiều |
| `--obstacle-spec "motorcycle:200,motorcycle:205"` | đặt xe máy **trong làn của xe mình**. Mỗi mục là `loại:khoảng_cách_m[:km/h]`: `motorcycle:200` là một xe máy đứng yên cách điểm xuất phát 200 m; `motorcycle:60:35` là xe máy ở 60 m và chạy thẳng 35 km/h. Dùng để kiểm tra xe dừng hay bám theo |
| `--motorcycle-spec "oncoming:150:40"` | thêm xe máy ở **làn khác**, do CARLA tự lái. Mỗi mục là `loại:khoảng_cách_m:km/h`, loại là `same` (cùng làn), `left` / `right` (làn bên cạnh cùng chiều) hoặc `oncoming` (làn ngược chiều). `oncoming:150:40` là xe máy chạy ngược chiều 40 km/h, xuất phát ở 150 m phía trước |
| `--cruise-kph 60` | sau khi xe tự engage, tự bấm nút để đưa tốc độ cài đặt của cruise lên 60 km/h. Không có cờ này thì cruise chỉ đặt 40 km/h, thấp hơn xe máy nhanh và xe không bám được vào tình huống cần |
| `--lead-log` | in log chi tiết mỗi giây: `[OP cmd]` openpilot ra lệnh gì, `[OP sees]` openpilot thấy xe phía trước cách bao xa, `[SIM]` khoảng cách thật từ CARLA, `[ERR]` chênh lệch giữa hai số đó. Dùng để chấm điểm đạt hay không |

Cờ `--duration N` không có trong các lệnh này. Nó làm bridge chạy N giây không có bàn phím rồi tự thoát, để chạy tự động
(`bench.sh`, smoke test). Các kịch bản ở đây cần bạn bấm `z`, `a`, `x` hoặc `q` nên không dùng nó.

### 1. Dừng trước một hàng xe máy đứng yên

Ba xe máy nối đuôi ở 200 m, 205 m và 210 m. Xe chạy cruise 60 km/h, phải phát hiện, giảm tốc và dừng trước xe máy đầu
tiên, không chạm.

```bash
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 \
  --obstacle-spec "motorcycle:200,motorcycle:205,motorcycle:210" \
  --cruise-kph 60 --lead-log
```

Đạt khi: log không có khối `#` va chạm, `speed` về 0 và `[SIM] gap` cuối cùng còn dương (vài mét). Nếu thấy
`NO LEAD` trong khi `gap` đã dưới 40 m, model bỏ lỡ xe máy.

### 2. Giữ khoảng cách sau hai xe máy chạy chậm

Hai xe máy cùng làn, xe sau chậm (15 km/h) ở 60 m, xe trước nhanh hơn (30 km/h) ở 120 m. Xe trước chạy xa dần nên
hai xe máy không chạm nhau. Xe mình chạy cruise 60 km/h, đuổi kịp xe máy 15 km/h rồi phải giảm xuống 15 km/h và giữ
khoảng cách ổn định.

```bash
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 \
  --obstacle-spec "motorcycle:60:15,motorcycle:120:30" \
  --cruise-kph 60 --lead-log
```

Đạt khi: `plan=lead0` khi đã đuổi kịp, `[SIM] speed` về gần 15 km/h, `gap` ổn định (không dao động mạnh, không dưới
vài mét), `closing` về gần 0, không có va chạm. Xe không được ép xe máy 15 km/h bằng phanh gấp: xem `accel` ở dòng
`[OP cmd]`.

### 3. Đánh lái tránh hàng xe máy

Hàng xe máy đứng yên chắn làn ở 250 m. Xe chạy cruise 50 km/h (trên mức tối thiểu để đổi làn) và người lái
đổi làn bằng xi-nhan trước khi tới sát (cách làm: mục "Đổi làn bằng xi-nhan").

```bash
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 \
  --obstacle-spec "motorcycle:250,motorcycle:255,motorcycle:260" \
  --cruise-kph 50 --lead-log
```

1. Chờ xe đạt tốc độ cruise, khi `[SIM] gap` còn khoảng 100 m thì bấm `z` (đổi sang trái; bên phải điểm 170 chỉ là làn lề đường).
2. Bấm `a` (hoặc `d`) cùng hướng xi-nhan để openpilot đổi làn.
3. Xe qua hàng xe máy, bấm `x` tắt xi-nhan.

Đạt khi: xe sang làn bên mà không chạm xe máy nào (không có khối `#`). Nếu xe đã giảm xuống dưới 30 km/h trước khi
bấm thì ALC không bắt đầu và `a` / `d` chỉ lái tay; tăng `--cruise-kph` hoặc bấm sớm hơn. Mô phỏng không báo điểm mù,
tự nhìn làn bên trước khi đổi.

### 4. Hàng xe máy chắn đường cùng xe ngược chiều

Như kịch bản 1 nhưng có thêm hai xe máy ngược chiều chạy qua trong lúc xe mình đang giảm tốc, để kiểm tra xe không nhận
nhầm xe ngược chiều thành lead.

```bash
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 \
  --obstacle-spec "motorcycle:200,motorcycle:205,motorcycle:210" \
  --motorcycle-spec "oncoming:150:50,oncoming:260:40" \
  --cruise-kph 60 --lead-log
```

Đạt khi: xe vẫn dừng trước hàng xe máy đứng yên như kịch bản 1, và `plan` không chuyển sang `lead0` vì xe ngược
chiều (xe ngược chiều không nằm trong làn của xe mình, nên `[SIM]` không tính chúng).

### 5. Vượt xe máy chậm trên đường hai chiều, làn bên trái có xe ngược chiều

Dùng `Town01_Opt` điểm 181 (đã đọc từ bản đồ CARLA): đường hai chiều, đúng hai làn, rộng 4 m, không có dải phân cách. Xe
mình ở làn phải (làn `-1`), bên phải là lề đường, **làn ngay bên trái là làn xe ngược chiều** (làn `1`). Muốn vượt xe
máy chậm thì phải sang làn đó. Đoạn thẳng dài khoảng 340 m (điểm 67 và 63 tương tự, 360 đến 390 m).

```bash
bash run.sh all --carla-town Town01_Opt --carla-spawn-point 181 \
  --obstacle-spec "motorcycle:60:35" \
  --motorcycle-spec "oncoming:150:40,oncoming:300:40" \
  --cruise-kph 50 --lead-log
```

Có một xe máy chậm 35 km/h trong làn mình (đủ nhanh để xe giữ trên 30 km/h lúc bám theo, là mức tối thiểu để đổi
làn), và hai xe máy ngược chiều 40 km/h chạy trong làn trái, xuất phát ở 150 m và 300 m. Xe máy chậm rẽ theo đường
ở khoảng 340 m (khoảng 30 giây sau khi xuất phát), nên chạy xong thì bấm `q` thoát hoặc `r` về điểm xuất phát.
Không thêm `--duration` vào lệnh này, vì nó tắt bàn phím (xem bảng ở đầu mục).

Thứ tự gần đúng (ước tính của tôi, chưa đo): xe ngược chiều thứ nhất lướt qua sau khoảng 6 giây, khi xe mình còn
đang đuổi theo xe máy chậm; xe thứ hai tới ngang xe mình sau khoảng 12 giây, đúng lúc xe mình bám sát xe máy chậm.
Xem vị trí thật bằng cửa sổ CARLA (góc nhìn bám sau xe), không cần chờ theo số giây.

**Lần 1, chờ cho thoáng.** Bám xe máy chậm (`plan=lead0`), đợi cả hai xe ngược chiều đi qua, rồi bấm `z`, `a` để sang
trái và vượt, rồi `x`. Đạt khi xe vượt xong, về làn cũ hoặc ở lại làn trái không va chạm, không có khối `#`.

**Lần 2, sang trái khi xe ngược chiều còn khoảng 100 m.** Bấm `z`, `a` ngay lúc xe ngược chiều đang lao tới. Đây là ca để
xem hệ thống có chặn hay không: hiện tại **không có gì chặn**. Điều kiện bắt đầu đổi làn trong
`desire_helper.py` chỉ gồm xi-nhan, tốc độ từ 30 km/h trở lên, thời gian chờ sau lần hủy và mép đường. Không có kiểm tra
xe ngược chiều, và mô phỏng không báo điểm mù. Nên mong đợi va chạm đầu đối đầu nếu người lái không tự dừng lại. Dùng
ca này để chứng minh hệ thống cần một kiểm tra như vậy trước khi sửa, không phải một ca "phải đạt".

Nếu lần 2 va chạm, log in khối `#` kèm `COLLIDED WITH: VEHICLE.KAWASAKI.NINJA` và tốc độ hai xe.

Trên `Town04_Opt` (cao tốc) xe ngược chiều chạy ở đường 1401 bên kia dải phân cách, cách xe mình khoảng 20 m, nên không
đối đầu được; dùng `Town01_Opt` cho ca này.

### Chấm điểm tự động

Cho một vật cản đơn lẻ (không phải hàng nhiều xe), `bench.sh` chạy nhiều lần và tự chấm, ví dụ:

```bash
bash bench.sh run motorcycle 0 60 3     # xe máy đứng yên, xe mình 60 km/h, 3 lần: ô "dừng trước vật cản"
bash bench.sh run motorcycle 40 60 3    # xe máy 40 km/h: ô "bám theo"
```

Điều kiện đạt của từng loại ô: xem [`bench.md`](bench.md).
