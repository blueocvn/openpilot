---
title: "Phase 3: Dựng CARLA 0.9.15 từ source"
status: todo
priority: P1
effort: "1-2d"
dependencies: [1]
---

# Phase 3: Dựng CARLA 0.9.15 từ source

## Overview

Digital Twin Tool chạy trong Unreal Editor, nên cần một bản CARLA 0.9.15 biên dịch từ source cùng Unreal Engine 4.26 bản của CARLA. Phase này dựng môi trường đó một lần trên máy văn phòng. Phần lớn thời gian là chờ máy biên dịch.

## Key Insights

- Tài liệu CARLA 0.9.15: khoảng 130 GB đĩa (Unreal khoảng 91 GB, CARLA khoảng 31 GB), GPU tối thiểu 6 GB, tổng thời gian 4 giờ trở lên.
- Tài liệu nhắm Ubuntu 18.04 (clang-8) và 20.04 (clang-10). Máy đang chạy Ubuntu 26.04, nên biên dịch trong container Ubuntu 20.04, mã nguồn đặt trên đĩa máy chủ và mount vào.
- Unreal Engine lấy từ `github.com/CarlaUnreal/UnrealEngine`, nhánh `carla`. Chỉ tài khoản GitHub đã liên kết với tài khoản Epic Games mới tải được.
- Container `carla_server` hiện tại đã chứng minh GPU và X11 dùng được từ trong container trên máy này.
- Luồng "nhập map bằng Docker" chính thức của CARLA cần 600 đến 700 GB nên không dùng.

## Requirements

- [ ] Người dùng liên kết tài khoản Epic Games với GitHub và đặt thông tin đăng nhập git trên máy (không ghi token vào repo).
- [ ] Mã nguồn và sản phẩm biên dịch nằm ngoài repo, ví dụ `/home/ubuntu/Project/carla-src/`.
- [ ] Unreal Editor mở được dự án CARLA trên màn hình máy văn phòng.

## Files to Create / Modify

- Create: `bukapilot/tools/sim-linux-docker/map-tools/Dockerfile.carla-source` (Ubuntu 20.04 và các gói phụ thuộc theo tài liệu 0.9.15, người dùng trùng uid máy chủ).
- Create: `bukapilot/tools/sim-linux-docker/map-tools/editor.sh` (mở shell hoặc editor trong container với GPU, X11 và thư mục mã nguồn được mount).
- Create: `bukapilot/tools/sim-linux-docker/map-tools/README.md` (các bước, dung lượng, thời gian đo được).

## Implementation Steps

1. Kiểm `df -h /`: cần còn ít nhất 180 GB sau khi đã có image 0.9.15. Nếu thiếu, hỏi người dùng trước khi xóa bất kỳ image nào (ví dụ các image 0.9.16 và bản trùng `carlasim/*`).
2. Viết `Dockerfile.carla-source` theo danh sách gói của tài liệu 0.9.15 cho Ubuntu 20.04; dựng image.
3. Người dùng xác nhận đã có quyền vào kho Unreal. Clone nhánh `carla` vào `/home/ubuntu/Project/carla-src/UnrealEngine_4.26`; chạy `Setup.sh`, `GenerateProjectFiles.sh`, `make` trong container.
4. Clone CARLA tag `0.9.15` vào `/home/ubuntu/Project/carla-src/carla`; chạy `Update.sh` để tải asset; đặt `UE4_ROOT`.
5. Chạy `make launch` trong container với X11 của máy chủ; chờ biên dịch shader; mở map mặc định và bấm Play.
6. Chạy `make osmrenderer` (trình xem OSM ngoại tuyến của Digital Twin Tool).
7. Nối client `carla==0.9.15` (image `bukapilot-sim:local`) vào editor đang Play để chắc phiên bản khớp.
8. Ghi dung lượng và thời gian thực tế vào `map-tools/README.md`.

## Todo

- [ ] Kiểm dung lượng đĩa
- [ ] Image môi trường biên dịch Ubuntu 20.04
- [ ] Quyền truy cập kho Unreal của người dùng
- [ ] Biên dịch Unreal Engine 4.26
- [ ] Clone CARLA 0.9.15 và tải asset
- [ ] `make launch` mở được editor
- [ ] `make osmrenderer`
- [ ] Client 0.9.15 nối được vào editor
- [ ] Ghi tài liệu `map-tools/README.md`

## Success Criteria

- Editor mở, bấm Play trên map mặc định, client 0.9.15 gọi `get_server_version()` trả về 0.9.15.
- Widget `UW_OnRoadMainWidget` có trong `CarlaToolsContent/OnroadMapGenerator`.
- Đĩa còn ít nhất 40 GB trống sau khi dựng xong (để lưu và đóng gói map).

## Risk Assessment

- Biên dịch lỗi do bộ công cụ cũ trên nhân máy mới. Dấu hiệu: `make` của Unreal hoặc CARLA dừng. Phản ứng: thử image Ubuntu 18.04 với clang-8; vẫn lỗi thì dừng và báo.
- Editor không khởi động với RTX 5060 Ti (driver mới, Vulkan của UE 4.26). Dấu hiệu: cửa sổ không mở hoặc crash khi biên dịch shader. Phản ứng: thử `-opengl` hoặc `-vulkan` tường minh; vẫn lỗi thì dừng và báo.
- 29 GB RAM có thể thiếu khi liên kết Unreal với 16 luồng. Phản ứng: giảm số luồng biên dịch.
- Không có quyền vào kho Unreal. Đây là việc chỉ người dùng làm được; phase chờ tới khi có.

## Security Considerations

Token GitHub chỉ nằm trong cấu hình git của người dùng trên máy; không đưa vào Dockerfile, script hay commit.

## Rollback

Xóa `/home/ubuntu/Project/carla-src/` và image môi trường biên dịch. Repo chỉ thêm thư mục `map-tools/`.
