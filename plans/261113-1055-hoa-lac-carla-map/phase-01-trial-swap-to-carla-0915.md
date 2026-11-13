---
title: "Phase 1: Thử swap sim sang CARLA 0.9.15"
status: todo
priority: P1
effort: "1d"
dependencies: []
---

# Phase 1: Thử swap sim sang CARLA 0.9.15

## Overview

Chạy sim bukapilot trên CARLA 0.9.15 song song với bản 0.9.12 đang dùng, đo sự khác biệt, và ra quyết định đạt hay không đạt. Mặc định vẫn là 0.9.12 trong suốt phase này.

## Key Insights

- `carla==0.9.15` có wheel cp38 trên PyPI; 0.9.16 thì không (đã kiểm ngày 2026-11-13).
- Bridge chỉ dùng API cơ bản (`load_world`, `try_spawn_actor`, `apply_control`, `get_wheel_steer_angle`, Traffic Manager, cảm biến RGB, GNSS, IMU, va chạm): xem `bukapilot/tools/sim/bridge/carla_world.py`.
- Thay đổi cần đo: gamma camera 2,2 thay vì 2,4; cảm biến va chạm một sự kiện mỗi frame; chỉ số điểm spawn có thể đổi (`bukapilot/tools/sim/demo.md:130` đã ghi nhận giữa 0.9.12 và 0.9.16); vật lý xe.
- `carla-assets/` (áo người lái xe máy có logo) là asset đã cook cho 0.9.12, chưa chắc nạp được trên 0.9.15.

## Requirements

- [ ] Phiên bản CARLA là tham số, mặc định `0.9.12`, không đổi hành vi hiện tại khi không đặt tham số.
- [ ] Container và cổng thử nghiệm tách khỏi sim đang chạy.
- [ ] Có bảng so sánh 0.9.12 và 0.9.15 đo trên cùng máy.

## Files to Create / Modify

- Modify: `bukapilot/tools/sim-linux-docker/Dockerfile.bukapilot` (thêm `ARG CARLA_VERSION=0.9.12`, dùng trong `pip install carla==`).
- Modify: `bukapilot/tools/sim-linux-docker/Dockerfile.carla` (thêm `ARG CARLA_VERSION=0.9.12` trước `FROM`).
- Modify: `bukapilot/tools/sim-linux-docker/start_carla.sh`, `run.sh` (đọc `CARLA_VERSION`, suy ra tên image).
- Create: `plans/261113-1055-hoa-lac-carla-map/reports/swap-trial-0915.md` (bảng so sánh và quyết định).

## Implementation Steps

1. Tham số hóa phiên bản trong hai Dockerfile và hai script, giữ mặc định 0.9.12. Kiểm tra `bash run.sh setup` không dựng lại image nào khi không đặt biến.
2. Dựng `bukapilot-sim:carla-0.9.15` và `carla-sim:0.9.15` với `CARLA_VERSION=0.9.15`. Kiểm tra dòng sửa apt trong `Dockerfile.carla` còn đúng với image gốc 0.9.15.
3. Khởi động server 0.9.15 off-screen, tên `carla_trial_0915`, cổng 2300, `CARLA_NO_ASSET_PATCH=1`. Ghi lại: khởi động được trên RTX 5060 Ti, phiên bản client và server khớp.
4. Chạy unit test `tools/sim/tests` trong container tạm bằng image 0.9.15 (không `--net=host`, không `--ipc=host`).
5. Xuất tọa độ điểm spawn 16, 170, 171 của `Town04_Opt` trên cả hai phiên bản; nếu lệch quá 1 m, tìm chỉ số tương đương trên 0.9.15 và ghi vào báo cáo.
6. Chạy smoke `--duration 120 --carla-town Town04_Opt --carla-spawn-point 16` trên 0.9.15 với container bridge tên riêng. Ghi: engage được, không rời làn, không lỗi process.
7. So ảnh camera: chụp cùng vị trí trên hai phiên bản, ghi độ sáng trung bình của vùng mặt đường. Nếu lệch rõ, thử thuộc tính `gamma` của `sensor.camera.rgb` để khớp lại.
8. Chạy `bash bench.sh short` trên cả hai phiên bản (đặt `CARLA_VERSION`, `BENCH_OUT` riêng), rồi các ô chính của plan vật cản. So kết quả từng ô.
9. Thử asset logo: chạy `carla-assets/build_rider_logo.py` với image 0.9.15; nếu file gốc khác cấu trúc, ghi lại là việc của Phase 2.
10. Viết `reports/swap-trial-0915.md` với bảng so sánh và kết luận đạt hay không đạt. Dừng mọi container thử nghiệm.

## Todo

- [ ] Tham số hóa `CARLA_VERSION`, mặc định không đổi
- [ ] Dựng hai image 0.9.15
- [ ] Server 0.9.15 chạy trên máy văn phòng
- [ ] Unit test đạt trên 0.9.15
- [ ] Đối chiếu điểm spawn Town04_Opt
- [ ] Smoke 120 giây engage
- [ ] So ảnh camera và gamma
- [ ] `bench.sh short` và các ô chính trên hai phiên bản
- [ ] Thử asset logo người lái
- [ ] Báo cáo và quyết định, dọn container

## Success Criteria

Cổng đạt khi tất cả đúng:

- Unit test đạt.
- Smoke 120 giây: engage, không rời làn, không process nào chết.
- Mỗi ô của `bench.sh short` có kết quả đạt/không đạt giống 0.9.12; khoảng cách tối thiểu lệch không quá 2 m.
- Server giữ 20 FPS ở chất lượng High.

Không đạt: dừng plan tại đây, báo số đo, lập lại plan theo hướng giữ 0.9.12.

## Risk Assessment

- Ảnh camera khác (gamma) làm model cho kết quả khác. Dấu hiệu: ô benchmark đổi kết quả. Phản ứng: khớp gamma ở cảm biến, đo lại một lần; vẫn lệch thì tính là không đạt.
- Image 0.9.15 chiếm thêm khoảng 30 GB. Kiểm `df -h /` trước khi pull.
- Hai server CARLA cùng lúc dùng khoảng 8,5 GB VRAM trên 16 GB (đã đo với hai server 0.9.12). Chạy benchmark so sánh lần lượt, không song song.

## Rollback

Không đặt `CARLA_VERSION` là quay về 0.9.12. Xóa image thử nghiệm bằng `docker rmi bukapilot-sim:carla-0.9.15 carla-sim:0.9.15`.
