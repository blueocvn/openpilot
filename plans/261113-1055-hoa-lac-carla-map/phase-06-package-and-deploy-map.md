---
title: "Phase 6: Đóng gói và đưa map vào sim"
status: todo
priority: P1
effort: "1d"
dependencies: [2, 5]
---

# Phase 6: Đóng gói và đưa map vào sim

## Overview

Đưa map `HoaLac` từ dự án Unreal sang server CARLA 0.9.15 dạng đóng gói mà sim đang dùng, để chạy bằng `--carla-town HoaLac` như mọi map khác.

## Key Insights

- Bridge nạp map bằng `client.load_world(town)` và so tên map hiện tại (`bukapilot/tools/sim/bridge/carla_world.py`, hàm `_load_world`). Một map đã cook có tên `HoaLac` không cần sửa bridge.
- Image CARLA đóng gói có sẵn thư mục `Import` và `ImportAssets.sh` (đã thấy trên image 0.9.12; cần kiểm lại trên 0.9.15).
- [UNVERIFIED] Cách đóng gói một map tạo trong editor bằng Digital Twin Tool: tài liệu Digital Twin không mô tả. `make package ARGS="--packages=<tên>"` được viết cho gói asset nhập qua thư mục `Import`. Bước đầu tiên của phase là xác định cách đúng từ mã nguồn 0.9.15.
- `--low_quality` gọi `unload_map_layer`; map mới không có layer nên cần kiểm lời gọi này không lỗi.

## Requirements

- [ ] Server 0.9.15 đóng gói (không có editor) nạp được `HoaLac`.
- [ ] `bash run.sh all --carla-town HoaLac --carla-spawn-point <n>` chạy được.
- [ ] Có danh sách điểm spawn nên dùng cho từng trục N.
- [ ] Cách tái tạo map được ghi lại từ đầu tới cuối.

## Files to Create / Modify

- Modify: `bukapilot/tools/sim-linux-docker/Dockerfile.carla` hoặc `start_carla.sh` (nạp gói map vào server; chọn cách ít thay đổi nhất sau bước 1).
- Modify: `bukapilot/tools/sim-linux-docker/demo.md` (map `HoaLac`, điểm spawn theo trục N).
- Modify: `bukapilot/tools/sim-linux-docker/map-tools/README.md` (đóng gói và nạp).
- Modify: `.gitignore` (bỏ qua file gói map nếu đặt trong cây repo).

## Implementation Steps

1. Đọc `Util/BuildTools/Package.sh` và tài liệu đóng gói của 0.9.15 để xác định cách đưa map lưu trong editor vào một gói; ghi kết luận vào `map-tools/README.md` trước khi chạy.
2. Đóng gói. Ưu tiên gói chỉ chứa map để nạp vào image `carlasim/carla:0.9.15` qua `Import` và `ImportAssets.sh`. Nếu không được, dùng bản CARLA đóng gói đầy đủ có kèm map làm image server.
3. Dựng image server có map (ví dụ `carla-sim:0.9.15-hoalac`), khởi động ở container và cổng riêng, gọi `load_world("HoaLac")` từ client 0.9.15.
4. Liệt kê điểm spawn, gán từng điểm vào trục N và chiều chạy; ghi bảng vào `demo.md`.
5. Chạy `bash run.sh all --carla-town HoaLac --carla-spawn-point <n>` và smoke `--duration 120`; thử thêm `--low_quality`.
6. Quyết định nơi lưu file gói map (dự kiến vài GB): ngoài repo kèm lệnh tái tạo, không đưa vào git. Hỏi người dùng nếu muốn lưu ở nơi khác.

## Todo

- [ ] Xác định cách đóng gói map từ mã nguồn 0.9.15
- [ ] Đóng gói map `HoaLac`
- [ ] Image server có map, nạp được qua API
- [ ] Bảng điểm spawn theo trục N
- [ ] `run.sh all` với `HoaLac`, có và không có `--low_quality`
- [ ] Chốt nơi lưu gói map
- [ ] Cập nhật `demo.md` và `map-tools/README.md`

## Success Criteria

- Trên server đóng gói: `client.load_world("HoaLac")` thành công, `get_spawn_points()` có điểm trên mọi trục N.
- `run.sh all --carla-town HoaLac` engage được và chạy 120 giây không process nào chết.
- Một người khác làm theo `map-tools/README.md` dựng lại được image có map.

## Extension Trace

Biến thể tiếp theo có thật là một khu thử khác (ví dụ tuyến trong khuôn viên ĐHQGHN). Hợp đồng giữ nguyên: tên map truyền qua `--carla-town` và chỉ số điểm spawn. Phần phải đổi: file `.osm`, bảng kiểm kê, tên map. Không thêm lớp đăng ký hay cấu hình map; thư mục `tools/sim/maps/<tên>/` là đủ.

## Risk Assessment

- Map lưu trong editor không cook được thành gói rời. Dấu hiệu: bước 2. Phản ứng: dùng bản đóng gói đầy đủ của bản dựng source làm server (dung lượng lớn hơn, khoảng 20 GB).
- Gói cook từ bản dựng source không tương thích image `carlasim/carla:0.9.15` (lệch asset). Phản ứng như trên.
- Tên map sau khi cook có đường dẫn khác (ví dụ nằm dưới thư mục con). `_load_world` so phần sau dấu `/` cuối nên vẫn đúng; kiểm ở bước 3.

## Rollback

Dùng lại image `carla-sim:0.9.15` không có map; bridge và script không đổi hành vi với các map cũ.
