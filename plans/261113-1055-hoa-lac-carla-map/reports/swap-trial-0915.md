# Thử swap sim sang CARLA 0.9.15

Ngày đo: 2026-11-16, trên máy văn phòng (RTX 5060 Ti 16 GB). Phase: [phase-01](../phase-01-trial-swap-to-carla-0915.md).

## Kết luận

Cổng đạt theo các tiêu chí của phase 1: kết quả đạt/không đạt của cả 5 ô `bench.sh standard` giống nhau giữa 0.9.12
và 0.9.15, smoke 120 giây và unit test đạt, server giữ thời gian thực ở chất lượng High. Có hai khác biệt về hành vi
cần biết trước khi chuyển hẳn (mục "Benchmark"). Người dùng xác nhận cổng đạt ngày 2026-11-16; sang Phase 2.

| Tiêu chí của phase 1 | Kết quả |
|---|---|
| Unit test `tools/sim/tests` | Đạt: 87 test, 1 bỏ qua (kernel OpenCL cũ, pocl không biên dịch được trong container; không liên quan phiên bản) |
| Smoke 120 giây, `Town04_Opt` spawn 16 | Đạt: engage suốt 118,6 s, 1252 m, không chạm cảnh vật, không va chạm, không phanh nhầm, không process nào chết |
| Đạt/không đạt từng ô giống 0.9.12 | Đạt: 5/5 ô |
| Khoảng cách tối thiểu lệch không quá 2 m | Đạt ở mọi ô trừ xe máy đứng yên (-0,81 so với -3,43 m). Ở ô đó cả hai bản đều đâm ở khoảng 16 m/s; số âm là độ lún sau va chạm, không phải khả năng phanh |
| 20 FPS ở High | Đạt: mọi lần chạy có `realtime_factor` từ 1,0 trở lên; camera 20 FPS |

## Thay đổi đã làm

| Phần | Thay đổi |
|---|---|
| Phiên bản CARLA là tham số | `CARLA_VERSION` (mặc định `0.9.12`) trong `Dockerfile.carla`, `Dockerfile.bukapilot`, `start_carla.sh`, `run.sh`, `start_docker.sh`, `bench.sh`. Image bukapilot theo phiên bản: `bukapilot-sim:local` (0.9.12), `bukapilot-sim:carla-<phiên bản>` (khác). Không đặt biến thì `run.sh setup` không dựng lại gì |
| Benchmark | `bench.sh standard` là bộ mặc định duy nhất: 5 ô ở `BENCH_EGO` (60 km/h), mỗi ô 1 lần, trượt thì chạy thêm đúng 1 lần. `short`: 2 ô xe máy. Bỏ `full`, bỏ `container` và `barrier` khỏi bộ có sẵn (vật tĩnh trên đường là việc của phanh tự động). Xem `bukapilot/tools/sim-linux-docker/bench.md` |
| Kịch bản vật cản chạy | Xe máy có tốc độ đứng chờ tới khi xe mình engage (`release_obstacles()` trong bridge). Trước đây nó chạy từ tick đầu nên ô bám theo không bao giờ bắt kịp. Kết quả ô xe máy chạy đo trước ngày này không so trực tiếp được |
| Build không phụ thuộc vị trí repo | RPATH của mọi file build trong repo là `$ORIGIN/...` (`SConstruct`); `libdbc`, `libkf` và hai solver MPC link theo tên. Đã kiểm: 53 file ELF không còn đường dẫn tuyệt đối vào repo; mount repo ở đường dẫn khác vẫn import được |

## Điểm spawn

Cả 372 điểm spawn của `Town04_Opt` trùng nhau giữa hai phiên bản (lệch 0,00 m, 0°), kể cả 16, 170 và 171 mà
benchmark dùng. Không cần đổi chỉ số.

## Ảnh camera

Cùng vị trí, cấu hình camera của bridge (1928 x 1208, FOV 40, cao 1,13 m, post-process bật, ClearNoon):

| Vị trí | Luma mặt đường 0.9.12 | 0.9.15 | Lệch |
|---|---|---|---|
| spawn 16 | 214,1 | 216,8 | +1,3 % |
| spawn 170 | 214,3 | 216,9 | +1,2 % |

Gamma mặc định của `sensor.camera.rgb` là 2,2 trên cả hai bản (plan ghi 0.9.12 là 2,4: sai). Đặt `gamma=2.4` trên
0.9.15 làm ảnh sáng thêm (219,8), lệch xa hơn, nên giữ mặc định. Nhìn bằng mắt hai ảnh gần như giống hệt; khác rõ nhất
là mẫu cột đèn đường.

## Benchmark

`bench.sh standard`, ego 60 km/h, cả hai bản không có asset patch (`CARLA_NO_ASSET_PATCH=1`) để so công bằng.
Kết quả: `bukapilot/.carla/bench/standard60-0.9.12/` và `standard60-0.9.15/`. Mỗi ô 2 lần (lần 1 trượt nên chạy lại).

| Ô | 0.9.12 | 0.9.15 |
|---|---|---|
| xe máy đứng yên | trượt: đâm ở 16,1 / 16,0 m/s; lead lần đầu ở 1,3 / 1,9 m | trượt: đâm ở 16,4 / 16,2 m/s; lead ở 4,3 / 8,0 m |
| xe máy 2 m/s | trượt: dừng sát còn 0,03 / 0,11 m, không chạm | trượt: chạm ở 2,0 m/s cả 2 lần |
| ô tô đỗ | trượt: chạm ở 2,8 / 3,6 m/s; lead ở khoảng 62 m | trượt: chạm ở 3,4 / 3,6 m/s; lead ở khoảng 58 m |
| đường trống 300 m | đạt: 15,96 m/s, không phanh nhầm | đạt: 16,15 m/s, không phanh nhầm |
| bám xe máy 40 km/h, 500 m | trượt: khoảng cách ổn định 5,2 / 5,7 m (planner muốn 17,9 m); lead ở 29,8 m | trượt: 4,6 / 4,5 m (muốn 18,4 / 18,6 m); lead ở 65 m |

Khác biệt về hành vi:

1. Xe máy 2 m/s: 0.9.12 dừng sát không chạm, 0.9.15 chạm nhẹ ở 2 m/s. Lệch khoảng 0,05 m, nhiều khả năng do vật lý xe.
2. Bám theo: trên 0.9.15 openpilot nhận xe máy là lead từ 65 m, trên 0.9.12 chỉ từ 30 m. Ảnh đầu vào của model khác
   nhau; số đo nhận diện trên 0.9.15 không so trực tiếp được với baseline 0.9.12.

Lần chạy 0.9.12 dùng bản build có RPATH tuyệt đối, lần 0.9.15 dùng bản build có RPATH tương đối (sửa giữa hai lần). Chỉ
khác cách tìm thư viện, không đổi code chạy.

## Asset logo người lái (`carla-assets/`)

| Texture | 0.9.12 so với 0.9.15 | Bản patch hiện có trên 0.9.15 |
|---|---|---|
| `T_Male1Cloth_v1_d` (áo phông trắng) | giống hệt từng byte | dùng được nguyên |
| `T_BomRed_d` (áo bomber đỏ) | cùng kích thước, khác byte (nén lại); hình giống nhau | không dùng được; phải dựng lại từ bản gốc 0.9.15 |

`build_rider_logo.py --src <bản gốc 0.9.15> --out <thư mục khác>` dựng được bản patch cho 0.9.15. Server 0.9.15 mount
bản patch đó nạp bình thường (không lỗi), logo BlueOC hiện trên ngực áo bomber của người lái Yamaha. Trong 7 mẫu xe hai
bánh của 0.9.15 không thấy người lái mặc áo phông trắng.

Việc của Phase 2: bản patch theo phiên bản (thư mục `carla-assets` cho 0.9.15, `start_carla.sh` mount theo
`CARLA_VERSION`), và `build_rider_logo.py` lấy bản gốc theo phiên bản (đang viết cứng `carla-sim:0.9.12`).

Chữ "CARLA" lớn trên lưng áo bomber (có ở cả hai bản) đã được che: `build_rider_logo.py` có thêm ô logo
(166, 1405, 454, 1495) trên lưng áo; bản patch 0.9.12 trong `carla-assets/Content` đã dựng lại và kiểm trên server
0.9.12 (lưng áo hiện "BlueOC"). Bản 0.9.15 dựng lại cùng script ở Phase 2.

## Câu hỏi còn mở

- Khi so hai phiên bản ở Phase 2: có bỏ tiêu chí "khoảng cách tối thiểu lệch không quá 2 m" cho ô mà cả hai bên đều
  va chạm (chỉ so tốc độ lúc chạm) không?
