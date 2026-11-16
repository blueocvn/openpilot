---
title: Thử swap sim bukapilot sang CARLA 0.9.15
date: 2026-11-16
summary: "Phase 1 đo xong: cổng đạt theo tiêu chí, chờ xác nhận; benchmark chuẩn hóa thành bench.sh standard"
---

# Thử swap sim bukapilot sang CARLA 0.9.15

## What happened
- Tham số hóa `CARLA_VERSION` (mặc định 0.9.12) trong Dockerfile và script của `bukapilot/tools/sim-linux-docker`; dựng `bukapilot-sim:carla-0.9.15`.
- Unit test lỗi `libdbc.so: cannot open shared object file`: các file `.so` ghi cứng đường dẫn tuyệt đối của chỗ repo cũ (`/home/ubuntu/openpilot`). Sửa bằng RPATH `$ORIGIN/...` trong `SConstruct` và link theo tên cho libdbc, libkf, solver MPC. 53 file ELF không còn đường dẫn tuyệt đối vào repo.
- Ô bám xe máy không đo được gì: xe máy chạy từ tick đầu nên xe mình không bao giờ bắt kịp trong 500 m. Sửa: vật cản chạy đứng chờ tới khi xe engage (`release_obstacles()`).
- Benchmark chuẩn hóa theo yêu cầu người dùng: chỉ còn `standard` (5 ô ở `BENCH_EGO`=60, 1 lần, trượt thì chạy thêm 1 lần) và `short` (2 ô xe máy); bỏ `full`, `container`, `barrier` vì vật tĩnh trên đường là việc của phanh tự động.

## Result
- 372 điểm spawn Town04_Opt trùng nhau; ảnh camera lệch khoảng 1,3 % độ sáng; gamma mặc định là 2,2 trên cả hai bản (plan ghi sai 2,4).
- 5/5 ô có cùng kết quả đạt/không đạt; khác biệt: xe máy 2 m/s chuyển từ dừng sát sang chạm nhẹ, và 0.9.15 nhận lead xe máy từ 65 m thay vì 30 m.
- Asset logo: áo phông dùng nguyên, áo bomber phải dựng lại từ bản gốc 0.9.15 (đã thử, nạp được).

## Next steps
- Người dùng xác nhận cổng rồi sang Phase 2 (mặc định 0.9.15, asset theo phiên bản).
- Báo cáo: plans/261113-1055-hoa-lac-carla-map/reports/swap-trial-0915.md

> Historical work record — not durable authority. Prefer docs/specs/ADRs for current decisions.
