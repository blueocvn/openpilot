# Benchmark vật cản (bench.sh)

`bench.sh` chạy cùng một bộ ô với xe đã engage, không cần bàn phím, và chấm bằng số thật của CARLA. Dùng để so sánh
trước và sau mỗi thay đổi của planner. Số thật của CARLA chỉ dùng để chấm điểm, không bao giờ đưa vào openpilot.

## Một ô là ba tham số

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| vật cản (`obstacle`) | `none`, `motorcycle`, `car`, `barrier`, `container` | `none` là đường trống; `car` là ô tô đỗ nhìn từ sau; `barrier` là rào chắn đường nằm ngang làn; `container` là thùng 2 x 1 x 1,7 m (trước nay không thành lead) |
| tốc độ vật cản (`obstacle_kph`) | `0` hoặc km/h | `0` là đứng yên. Chỉ `motorcycle` chạy được: chạy thẳng theo đường với tốc độ không đổi |
| tốc độ xe mình (`ego_kph`) | km/h | tốc độ cài đặt của cruise, ví dụ 30 hoặc 60 |

Ô có vật cản chạy phải chậm hơn xe mình, nếu không xe không bao giờ đuổi kịp: ô đó bị bỏ qua và script báo lý do.
Cờ `OVERTAKE=1` thêm "người lái" theo kịch bản: sau khi xe dừng sau vật cản 3 giây, người lái bấm xi-nhan trái, đánh lái
và đạp ga để vượt (cùng lệnh với bàn phím).

Tham số quyết định cả cách đo và điều kiện đạt (xem mục "Điều kiện đạt"), nên không cần tên kịch bản riêng:

| Tham số | Kiểu ô | Ví dụ |
|---|---|---|
| `none` | đường trống | `none 0 60` |
| vật cản đứng yên, hoặc chạy không quá 3 m/s (10,8 km/h) | dừng trước vật cản | `container 0 60`, `motorcycle 7.2 30` |
| vật cản chạy nhanh hơn 3 m/s, chậm hơn xe mình | bám theo | `motorcycle 40 60`, `motorcycle 20 30` |

## Lệnh

Chạy trong `tools/sim-linux-docker/`.

```bash
bash bench.sh run motorcycle 20 30 3                   # một ô: xe máy 20 km/h, xe mình 30 km/h, 3 lần
OVERTAKE=1 bash bench.sh run barrier 0 30 3            # một ô vượt vật cản
bash bench.sh full                                     # các ô có sẵn (xem `list`), 3 lần mỗi ô
BENCH_JOBS=2 bash bench.sh full                        # như trên, chạy 2 stack song song (nhanh gần gấp đôi)
bash bench.sh list                                     # các ô sẽ chạy và tổng số lần chạy
OBSTACLE=motorcycle OBSTACLE_KPH=0,10,20 EGO=30,60 bash bench.sh grid   # mọi tổ hợp, không chỉ các ô có sẵn
bash bench.sh short                                    # khoảng 5 phút: xe máy đứng yên + đường trống, 30 km/h, 3 lần
bash bench.sh summary <thư mục kết quả>                # in lại bảng, ghi results.csv và results.md
bash bench.sh stop                                     # tắt CARLA và container của benchmark
```

## Các ô có sẵn (`full`)

14 ô, 42 lần chạy với 3 lần lặp. Chúng là các dòng của bảng tiêu chí trong plan, viết bằng ba tham số:

| Ô | Ý nghĩa | `ego_kph` |
|---|---|---|
| `motorcycle 0` | xe máy đứng yên | 30 và 60 |
| `motorcycle 7.2` | xe máy chạy 2 m/s | 30 và 60 |
| `car 0` | ô tô đỗ | 30 và 60 |
| `barrier 0` | rào chắn | 30 và 60 |
| `container 0` | thùng container | 30 và 60 |
| `none 0` | đường trống | 30 và 60 |
| `motorcycle 40` | bám xe máy 40 km/h | 60 |
| `barrier 0` + `overtake` | dừng sau rào rồi vượt | 30 |

## Chọn ô bằng bộ lọc

`full` và `list` nhận ba bộ lọc, phân cách bằng dấu phẩy, đều tùy chọn. Một ô phải khớp mọi bộ lọc đã đặt. Không ô nào
khớp thì script báo lỗi và không chạy gì.

| Biến | Ví dụ |
|---|---|
| `OBSTACLE` | `motorcycle,container` |
| `OBSTACLE_KPH` | `0,40` |
| `EGO` | `30,60` |

```bash
OBSTACLE=container,barrier EGO=60 bash bench.sh full     # 2 ô = 6 lần
EGO=60 REPEATS=1 BENCH_JOBS=2 bash bench.sh full         # một lượt nhanh ở 60 km/h
```

`grid` bắt buộc đặt cả ba biến và chạy mọi tổ hợp (kể cả những tổ hợp không có sẵn); tổ hợp vô nghĩa bị bỏ qua kèm lý
do. Dùng `bash bench.sh list grid` để xem trước.

Để rút ngắn: giảm `REPEATS` (1 lần chỉ cho xu hướng, không biết độ lệch giữa các lần), chạy ít ô hơn bằng bộ lọc, hoặc
chạy 2 stack song song. Ở Town04 hai lần lặp của cùng một ô có thể lệch nhau nhiều mét (model thấy vật ở khoảng cách
khác nhau mỗi lần), nên khi so hai cấu hình cần nhiều lần lặp hơn.

## Kết thúc lần chạy

| Kiểu ô | Kết thúc khi |
|---|---|
| vật cản đứng yên | va chạm, hoặc xe dừng hẳn 5 giây |
| xe máy chạy không quá 3 m/s | xe đi đủ 230 m (xe máy bắt đầu cách 100 m) |
| xe máy chạy nhanh hơn 3 m/s | xe đi đủ 1300 m (xe máy bắt đầu cách 60 m) |
| `none` | xe đi đủ 1000 m (đổi bằng `BENCH_CLEAR_M`) |
| `overtake` | kịch bản chạy xong + 3 giây |

Mọi lần chạy còn có giới hạn thời gian thực (tính từ quãng đường và tốc độ xe, cộng khoảng 30 giây khởi động); lần nào
chưa kết thúc khi hết giờ coi như hỏng (xe không bao giờ gặp vật cản) và bị cắt, không kéo dài hàng phút.

## Bản đồ và vị trí

Chỉ dùng `Town04_Opt` (đường cao tốc). Town05 đã bỏ: đường thẳng ngắn (tối đa 360 m, ngã tư mỗi 90 m), và ở các ngã tư
không có vạch kẻ xe lệch khỏi đường (đo: ở 60 km/h xe tông hàng rào trước khi gặp vật cản; ở 30 km/h qua ngã tư 170 m
xe cũng lệch). Ngã tư là một case riêng, sẽ xử lý sau.

| Ô | Spawn | Ghi chú |
|---|---|---|
| mọi ô trừ xe máy chạy nhanh hơn 3 m/s | 170 | thẳng 450 m, có một chỗ nhập làn ở 245 đến 300 m; vật cản đứng yên cách spawn 400 m (đổi bằng `OBSTACLE_M`) |
| xe máy chạy nhanh hơn 3 m/s | 171 | thẳng 715 m rồi cong nhẹ |

- Đường lên dốc khoảng 11 m trên đoạn thẳng.
- Xe máy có tốc độ do bridge điều khiển: chạy thẳng theo đường với tốc độ không đổi, không rẽ ở chỗ nhập làn (Traffic
  Manager của CARLA 0.9.12 không có lệnh đặt lộ trình). Nó không dùng vật lý nên xe mình đâm vào là va chạm cứng.

## Tốc độ và tài nguyên

- Benchmark dùng server CARLA riêng (cổng 2100, 2110, ... không cửa sổ, container `carla_bench_N`) và container bridge riêng
  (`bukapilot_bench_N`), nên phiên mô phỏng đang chạy ở cổng 2000 không bị ảnh hưởng; hai bên chỉ dùng chung GPU.
- `scons` chạy một lần trong một container riêng trước khi các stack khởi động (các stack chạy với `SKIP_BUILD=1`).
  Hai stack chạy `scons` cùng lúc trên cùng cây thư mục làm hỏng cơ sở dữ liệu build của nó.
- Không tăng tốc đồng hồ mô phỏng được: các tiến trình openpilot (`modeld`, `controlsd`) chạy theo đồng hồ thật, nên
  mô phỏng chạy đúng thời gian thực. Tăng tốc sẽ đổi độ trễ mà openpilot cảm nhận, đúng thứ ta đang đo.
- Cách rút ngắn: chạy nhiều stack song song (`BENCH_JOBS=N`, mỗi stack cần khoảng 3 đến 4 GB GPU) và ít lần lặp hơn
  (`REPEATS`, mặc định 3).
- Đo trên máy này (GPU 16 GB, sim của người dùng chạy cùng lúc): 2 stack thường giữ được thời gian thực, nhưng lần chạy
  dài ở 60 km/h (bám xe máy) từng xuống 0,69; 3 stack xuống 0,8. Dùng `BENCH_JOBS=1` khi cần kết quả chắc chắn.
- Mỗi kết quả có `realtime_factor`. Dưới 0,95 là máy quá tải; `bench.sh summary` tự đánh trượt lần đó và yêu cầu chạy lại.
- Thời gian, quãng đường và gia tốc tính theo đồng hồ mô phỏng.

## Biến môi trường

| Biến | Mặc định | Ý nghĩa |
|---|---|---|
| `REPEATS` | `3` | số lần lặp mỗi ô (`full`, `grid`, `run`) |
| `BENCH_JOBS` | `1` | số stack chạy song song; tối đa nên là 2 |
| `BENCH_OUT` | `<bukapilot>/.carla/bench/<thời gian>` | thư mục kết quả |
| `BENCH_CLEAR_M` | `1000` (`short`: `600`) | độ dài ô `none` (m) |
| `OBSTACLE_M` | `400` | khoảng cách tới vật cản đứng yên (m) |
| `RUN_CAP_S` | tính từ quãng đường và tốc độ | giới hạn thời gian mỗi lần chạy (giây) |
| `BENCH_PORT` | `2100` | cổng CARLA của stack 0; stack n dùng `BENCH_PORT + 10 n` |
| `BENCH_QUALITY` | `High` | chất lượng đồ họa CARLA (`High` / `Low`) |
| `BRIDGE_ARGS` | không đặt | tham số thêm cho bridge |
| `OVERTAKE` | không đặt | `1`: thêm người lái theo kịch bản vượt (`run`, `grid`) |
| `PARAMS` | không đặt | `Khóa=1,Khóa=0`: đặt các Params boolean cho mọi lần chạy (cờ tính năng đang thử), ví dụ `PARAMS=VisionLeadHold=1` |
| `TRACE`, `TRACE_IMAGES` | không đặt | `TRACE=1`: mỗi lần chạy ghi thêm `<lần chạy>.trace/truth.jsonl` (đầu ra lead của model cạnh số thật, mỗi khung camera một dòng); `TRACE_IMAGES=1` lưu thêm ảnh |

Các biến sau do `bench.sh` tự đặt cho bridge; có thể đặt tay khi chạy bridge riêng:

| Biến | Mặc định | Ý nghĩa |
|---|---|---|
| `LEAD_RESULT_FILE` | không đặt | ghi kết quả chấm điểm của lần chạy ra một file JSON và tự kết thúc lần chạy (va chạm, dừng hẳn, hoặc đi đủ `LEAD_RUN_DISTANCE_M` mét) |
| `LEAD_SCENARIO` | rỗng | nhãn `vật cản/tốc độ vật cản/tốc độ xe[/overtake]` ghi vào kết quả |
| `LEAD_RUN_DISTANCE_M` | không đặt | quãng đường kết thúc lần chạy (m) |
| `LEAD_OVERTAKE` | `0` | `1`: người lái theo kịch bản vượt vật cản. Cần `LEAD_RESULT_FILE` |
| `LEAD_RECORD_DIR`, `LEAD_RECORD_EVERY`, `LEAD_RECORD_IMAGES` | không đặt, `1`, `1` | lưu mọi khung hình camera thứ N (JPEG) kèm `truth.jsonl` (khoảng cách thật, tốc độ, tư thế xe, đầu ra lead của model); `LEAD_RECORD_IMAGES=0` chỉ ghi `truth.jsonl` |
| `SIM_PARAMS` | không đặt | `Khóa=1,Khóa=0`: Params boolean đặt trước khi manager chạy (do `PARAMS` truyền vào) |

## Kết quả

Nằm trong `.carla/bench/<thời gian>/`: mỗi lần chạy một file JSON và hai file log (bridge và manager), cộng `results.csv`
(mọi lần chạy) và `results.md` (bảng theo ô). Tên file là `<vật cản>-<tốc độ vật cản>kph_ego<tốc độ xe>_<lần>`, ví dụ
`motorcycle-20kph_ego30_01.json`. Lần chạy không ghi được kết quả vẫn có một file JSON `end_reason: "no result"` và tính
là trượt.

| Trường | Ý nghĩa |
|---|---|
| `min_gap_m` | khoảng cách nhỏ nhất giữa hai đầu xe tới vật cản trong làn (âm hoặc 0 là đã chạm) |
| `collision`, `impact_speed_mps` | có va chạm với vật cản không, tốc độ xe lúc chạm |
| `other_contacts`, `other_contact_types` | số lần và loại vật khác đã chạm (lề, tường, cột): xe đã ra khỏi đường |
| `peak_decel_mps2`, `time_below_m1_5_s` | giảm tốc mạnh nhất, tổng thời gian giảm tốc mạnh hơn 1,5 m/s² (chỉ tính trước lúc chạm) |
| `false_brake_events` | số lần giảm tốc mạnh hơn 1,5 m/s² khi không có vật nào trong 60 m |
| `approach_speed_mps` | tốc độ xe lúc vật cản còn cách 60 m |
| `first_lead_true_gap_m` | khoảng cách thật lúc openpilot lần đầu có lead |
| `avg_speed_cruise_mps`, `set_speed_min_kph` | tốc độ trung bình sau khi đạt 90% tốc độ cài đặt; tốc độ cài đặt thấp nhất trong lần chạy |
| `steady_gap_m`, `intended_gap_m` | khoảng cách trung bình 10 giây cuối và khoảng cách planner muốn giữ (ô bám theo) |
| `overtake_max_brake`, `overtake_yrel_mean_abs_error_m` | phanh lớn nhất trong lúc người lái vượt; sai số lệch ngang `yRel` của openpilot so với thật |
| `realtime_factor` | 1,0 là mô phỏng kịp thời gian thực; dưới 0,95 là máy quá tải |
| `end_reason` | `collision`, `distance`, `stopped`, `overtake`, hoặc rỗng khi hết giờ |

## Điều kiện đạt

Định nghĩa trong `tools/sim/bridge/bench_summary.py` (`rule_for`), lấy từ bảng tiêu chí của plan
`plans/261112-1710-static-obstacle-braking-experiments/plan.md`. Luật được chọn theo ô, không theo tên.

| Ô | Đạt khi |
|---|---|
| vật cản đứng yên hoặc chạy không quá 3 m/s | không va chạm, `min_gap_m` từ 4 m trở lên, giảm tốc không mạnh hơn 3,5 m/s² |
| `none` (đường trống) | 0 lần phanh nhầm, tốc độ trung bình từ 90% tốc độ cài đặt |
| vật cản chạy nhanh hơn 3 m/s | không va chạm, khoảng cách ổn định lệch không quá 20% so với khoảng cách planner muốn giữ |
| có `overtake` | không chạm, không phanh ngược ý người lái (phanh tối đa không quá 5%) |

Mọi lần chạy còn bị loại nếu chưa engage, nếu xe ra khỏi đường (chạm vật khác ngoài vật cản), hoặc nếu máy quá tải.
