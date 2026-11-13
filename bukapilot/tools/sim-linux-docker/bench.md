# Benchmark vật cản (bench.sh)

`bench.sh` chạy cùng một bộ tình huống với xe đã engage, không cần bàn phím, và chấm bằng số thật của CARLA. Dùng để
so sánh trước và sau mỗi thay đổi của planner. Số thật của CARLA chỉ dùng để chấm điểm, không bao giờ đưa vào openpilot.

## Lệnh

Chạy trong `tools/sim-linux-docker/`.

```bash
bash bench.sh short                                    # khoảng 5 phút: xe máy đứng yên + đường trống, Town05, 30 km/h, 3 lần
bash bench.sh full                                     # toàn bộ ma trận, 3 lần mỗi ô
BENCH_JOBS=2 bash bench.sh full                        # như trên, chạy 2 stack song song (nhanh gần gấp đôi)
bash bench.sh run container Town05_Opt 30 3            # một tình huống, 3 lần
bash bench.sh list                                     # các ô sẽ chạy và tổng số lần chạy
bash bench.sh summary <thư mục kết quả>                # in lại bảng, ghi results.csv và results.md
bash bench.sh stop                                     # tắt CARLA và container của benchmark
```

## Chọn kịch bản bằng bộ lọc

`full` và `list` nhận ba bộ lọc, phân cách bằng dấu phẩy, đều tùy chọn. Một ô phải khớp mọi bộ lọc đã đặt. Không ô nào
khớp thì `full` báo lỗi và không chạy.

| Biến | Giá trị |
|---|---|
| `SCENARIOS` | `stationary-motorcycle`, `slow-motorcycle`, `parked-car`, `barrier`, `container`, `clear-road`, `follow-motorcycle`, `overtake` |
| `TOWNS` | `Town04_Opt`, `Town05_Opt` |
| `SPEEDS` | `30`, `60` (km/h, tốc độ cài đặt) |

```bash
SCENARIOS=container,barrier TOWNS=Town04_Opt bash bench.sh full    # 4 ô = 12 lần
SPEEDS=60 REPEATS=1 BENCH_JOBS=2 bash bench.sh full                # một lượt nhanh ở 60 km/h
```

Ma trận đầy đủ là 21 ô, 63 lần chạy với 3 lần lặp. Để rút ngắn: giảm `REPEATS` (1 lần chỉ cho xu hướng, không biết
độ lệch giữa các lần), bỏ `clear-road` và `follow-motorcycle` (hai ca dài nhất), hoặc chia theo mục đích (các ca phanh
trước vật cản riêng, phanh nhầm và bám xe riêng).

## Tình huống

| Tình huống | Vật cản | Kết thúc khi |
|---|---|---|
| `stationary-motorcycle` | xe máy đứng yên trong làn | va chạm, hoặc xe dừng hẳn 5 giây |
| `slow-motorcycle` | xe máy chạy thẳng 2 m/s (7,2 km/h), bắt đầu cách 100 m | xe đi đủ 160 m (Town05) hoặc 230 m (Town04) |
| `parked-car` | ô tô đỗ, nhìn từ sau | như xe máy đứng yên |
| `barrier` | rào chắn đường nằm ngang làn | như xe máy đứng yên |
| `container` | thùng container nhỏ (2 x 1 x 1,7 m), trước nay không thành lead | như xe máy đứng yên |
| `follow-motorcycle` | xe máy chạy thẳng 40 km/h, bắt đầu cách 60 m | xe đi đủ 1300 m |
| `clear-road` | không có | xe đi đủ 1000 m (đổi bằng `BENCH_CLEAR_M`) |
| `overtake` | rào chắn; sau khi xe dừng 3 giây, một "người lái" theo kịch bản bấm xi-nhan trái, đánh lái và đạp ga để vượt (cùng lệnh với bàn phím) | kịch bản chạy xong + 3 giây |

Mọi lần chạy còn có giới hạn thời gian thực (tính từ quãng đường và tốc độ, cộng khoảng 30 giây khởi động); lần nào
chưa kết thúc khi hết giờ coi như hỏng (xe không bao giờ gặp vật cản) và bị cắt, không kéo dài hàng phút.

## Bản đồ và vị trí

| Town | Spawn | Vật cản tĩnh cách spawn | Ghi chú |
|---|---|---|---|
| `Town04_Opt` | 170 (bám xe máy: 171) | 400 m | spawn 170 thẳng 450 m; spawn 171 thẳng 715 m rồi cong nhẹ |
| `Town05_Opt` | 126 (`clear-road`: 1) | 150 m | không có đường thẳng nào dài hơn 360 m; ngã tư mỗi 90 m |

- `clear-road` giữ spawn cũ (Town04 170, Town05 1), có khúc cua.
- Đổi khoảng cách vật cản bằng `OBSTACLE_M`.
- Vật cản nằm trước ngã tư ở 170 m của Town05: qua ngã tư đó xe rẽ lệch (đo: ở 250 m xe không bao giờ gặp vật cản), và
  xe phải dừng được trên đoạn có vạch kẻ chứ không phải trong ngã tư. Ngã tư là một case riêng, sẽ xử lý sau.
- Town05 chỉ chạy ở 30 km/h: ở 60 km/h xe luôn lệch khỏi đoạn thẳng trong các ngã tư và tông hàng rào trước khi gặp vật cản
  (đo 4 lần, lần nào cũng vậy). Các ô 60 km/h của `full` chỉ có ở Town04. `follow-motorcycle` cũng chỉ chạy ở Town04.
- Xe máy có tốc độ do bridge điều khiển: chạy thẳng theo đường với tốc độ không đổi, không rẽ ở ngã tư (Traffic Manager
  của CARLA 0.9.12 không có lệnh đặt lộ trình). Nó không dùng vật lý nên xe mình đâm vào là va chạm cứng.

## Tốc độ và tài nguyên

- Benchmark dùng server CARLA riêng (cổng 2100, 2110, ... không cửa sổ, container `carla_bench_N`) và container bridge riêng
  (`bukapilot_bench_N`), nên phiên mô phỏng đang chạy ở cổng 2000 không bị ảnh hưởng; hai bên chỉ dùng chung GPU.
- Không tăng tốc đồng hồ mô phỏng được: các tiến trình openpilot (`modeld`, `controlsd`) chạy theo đồng hồ thật, nên
  mô phỏng chạy đúng thời gian thực. Tăng tốc sẽ đổi độ trễ mà openpilot cảm nhận, đúng thứ ta đang đo.
- Cách rút ngắn: chạy nhiều stack song song (`BENCH_JOBS=N`, mỗi stack cần khoảng 3 đến 4 GB GPU) và ít lần lặp hơn
  (`REPEATS`, mặc định 3 vì kết quả lặp lại trong khoảng 1 m).
- Đo trên máy này (GPU 16 GB, sim của người dùng chạy cùng lúc): 2 stack giữ được thời gian thực, 3 stack xuống 0,8 nên
  không dùng được.
- Mỗi kết quả có `realtime_factor`. Dưới 0,95 là máy quá tải; `bench.sh summary` tự đánh trượt lần đó và yêu cầu chạy lại.
- Thời gian, quãng đường và gia tốc tính theo đồng hồ mô phỏng.

## Biến môi trường

| Biến | Mặc định | Ý nghĩa |
|---|---|---|
| `REPEATS` | `3` | số lần lặp mỗi ô (`full`, `run`) |
| `BENCH_JOBS` | `1` | số stack chạy song song; tối đa nên là 2 |
| `BENCH_OUT` | `<bukapilot>/.carla/bench/<thời gian>` | thư mục kết quả |
| `BENCH_CLEAR_M` | `1000` (`short`: `600`) | độ dài `clear-road` (m) |
| `OBSTACLE_M` | `400` (Town04), `150` (Town05) | khoảng cách tới vật cản tĩnh (m) |
| `RUN_CAP_S` | tính từ quãng đường và tốc độ | giới hạn thời gian mỗi lần chạy (giây) |
| `BENCH_PORT` | `2100` | cổng CARLA của stack 0; stack n dùng `BENCH_PORT + 10 n` |
| `BENCH_QUALITY` | `High` | chất lượng đồ họa CARLA (`High` / `Low`) |
| `BRIDGE_ARGS` | không đặt | tham số thêm cho bridge |

Các biến sau do `bench.sh` tự đặt cho bridge; có thể đặt tay khi chạy bridge riêng:

| Biến | Mặc định | Ý nghĩa |
|---|---|---|
| `LEAD_RESULT_FILE` | không đặt | ghi kết quả chấm điểm của lần chạy ra một file JSON và tự kết thúc lần chạy (va chạm, dừng hẳn, hoặc đi đủ `LEAD_RUN_DISTANCE_M` mét) |
| `LEAD_SCENARIO` | rỗng | nhãn `tên/town/tốc độ` ghi vào kết quả |
| `LEAD_RUN_DISTANCE_M` | không đặt | quãng đường kết thúc lần chạy (m) |
| `LEAD_OVERTAKE` | `0` | `1`: người lái theo kịch bản vượt vật cản. Cần `LEAD_RESULT_FILE` |
| `LEAD_RECORD_DIR`, `LEAD_RECORD_EVERY` | không đặt, `1` | lưu mọi khung hình camera thứ N (JPEG) kèm `truth.jsonl` (khoảng cách thật, tốc độ, tư thế xe) để thử nghiệm offline |

## Kết quả

Nằm trong `.carla/bench/<thời gian>/`: mỗi lần chạy một file JSON và hai file log (bridge và manager), cộng `results.csv`
(mọi lần chạy) và `results.md` (bảng theo tình huống). Lần chạy không ghi được kết quả vẫn có một file JSON
`end_reason: "no result"` và tính là trượt.

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
| `steady_gap_m`, `intended_gap_m` | khoảng cách trung bình 10 giây cuối và khoảng cách planner muốn giữ (tình huống bám xe) |
| `overtake_max_brake`, `overtake_yrel_mean_abs_error_m` | phanh lớn nhất trong lúc người lái vượt; sai số lệch ngang `yRel` của openpilot so với thật |
| `realtime_factor` | 1,0 là mô phỏng kịp thời gian thực; dưới 0,95 là máy quá tải |
| `end_reason` | `collision`, `distance`, `stopped`, `overtake`, hoặc rỗng khi hết giờ |

## Điều kiện đạt

Định nghĩa trong `tools/sim/bridge/bench_summary.py`, lấy từ bảng tiêu chí của plan
`plans/261112-1710-static-obstacle-braking-experiments/plan.md`.

| Tình huống | Đạt khi |
|---|---|
| Vật cản đứng yên hoặc chạy chậm | không va chạm, `min_gap_m` từ 4 m trở lên, giảm tốc không mạnh hơn 3,5 m/s² |
| `clear-road` | 0 lần phanh nhầm, tốc độ trung bình từ 90% tốc độ cài đặt |
| `follow-motorcycle` | khoảng cách ổn định lệch không quá 20% so với khoảng cách planner muốn giữ |
| `overtake` | không chạm, không phanh ngược ý người lái (phanh tối đa không quá 5%) |

Mọi lần chạy còn bị loại nếu chưa engage, nếu xe ra khỏi đường (chạm vật khác ngoài vật cản), hoặc nếu máy quá tải.
