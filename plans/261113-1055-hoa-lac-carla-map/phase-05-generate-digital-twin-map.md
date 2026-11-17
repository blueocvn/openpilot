---
title: "Phase 5: Sinh map bằng Digital Twin Tool"
status: todo
priority: P1
effort: "1-2d"
dependencies: [3, 4]
---

# Phase 5: Sinh map bằng Digital Twin Tool

## Overview

Dùng Digital Twin Tool trong Unreal Editor của CARLA 0.9.15 để sinh map `HoaLac` từ `hoalac-core.osm`, kiểm vạch kẻ làn trong góc nhìn camera lái, rồi lưu map.

## Key Insights

- Theo tài liệu CARLA 0.9.15: công cụ sinh mặt đường có vạch kẻ và vân bề mặt, nhà theo footprint và chiều cao trong OSM, cây trên vỉa hè. Sinh khoảng 10 phút cho vùng 2 x 2 km; lưu map mất từ một đến vài giờ.
- CARLA ghi rõ đây là tính năng thử nghiệm, có thể còn vùng chưa trang trí hoặc chưa có vân.
- Trình xem OSM ngoại tuyến chỉ có trên Linux và cần `make osmrenderer` (đã làm ở Phase 3).
- Tài liệu không nói về địa hình: map sẽ phẳng.
- Nhà do công cụ dựng theo footprint và chiều cao trong OSM; nếu OSM thiếu footprint ở Hòa Lạc, công cụ lấp vào các khoảng trống bằng nhà giả lập. Chỉ mạng đường là dữ liệu bám thực tế, nên không gọi phần nhà là digital twin. Ghi điều này vào báo cáo sinh map.
- Camera lái của sim: FOV 40, 1928 x 1208, đặt cao 1,13 m, cách gốc tọa độ của xe 0,8 m về phía trước (`bukapilot/tools/sim/bridge/carla_world.py`).

## Requirements

- [ ] Map tên `HoaLac` lưu trong dự án CARLA 0.9.15.
- [ ] Vạch kẻ làn hiện trên mọi trục N khi nhìn từ camera lái.
- [ ] Map có điểm spawn và OpenDRIVE đi kèm đọc được qua API.

## Files to Create / Modify

- Create: `plans/261113-1055-hoa-lac-carla-map/reports/digital-twin-generation.md` (thông số, thời gian, ảnh, vấn đề gặp phải).
- Modify: `bukapilot/tools/sim-linux-docker/map-tools/README.md` (các bước sinh map đã chạy thật).

## Implementation Steps

1. Chép `hoalac-core.osm` vào thư mục map của trình xem OSM trong cây mã nguồn CARLA và chạy script tiền xử lý của nó theo tài liệu Digital Twin (`./build.sh <đường_dẫn_file.osm>` trong `Build/libosmcout-source/maps/`). Nếu script báo không tìm thấy `Import`, nhánh `elif` ở Phase 3 bước 6 chưa được thêm.
2. Mở editor, chạy widget `UW_OnRoadMainWidget` (trong `CarlaToolsContent/OnroadMapGenerator`), chọn cơ sở dữ liệu OSM vừa xử lý, chọn vùng lưới trục N, đặt tên `HoaLac`, bấm generate. Ghi thời gian.
3. Xem map trong editor: từng trục N có vạch tim và vạch chia làn không, nút giao có liền mặt không, vùng nào chưa trang trí.
4. Bấm Play, nối client 0.9.15: đếm điểm spawn, lấy OpenDRIVE qua `get_map().to_opendrive()`, so số làn và bề rộng làn với bảng kiểm kê của Phase 4.
5. Chụp ảnh từ camera lái (thông số như trên) tại ít nhất một điểm trên mỗi trục N và tại hai nút giao; lưu vào báo cáo.
6. Sửa trong editor những chỗ thiếu vạch hoặc mặt đường hở trên trục N. Không trang trí thêm ngoài việc đó.
7. Bấm Save Map và để máy chạy tới khi xong. Ghi thời gian và dung lượng.
8. Viết `reports/digital-twin-generation.md`.

## Todo

- [ ] Tiền xử lý OSM cho trình xem ngoại tuyến
- [ ] Sinh map `HoaLac`
- [ ] Kiểm vạch và nút giao trong editor
- [ ] Kiểm điểm spawn và OpenDRIVE qua API
- [ ] Ảnh camera lái trên từng trục N
- [ ] Sửa chỗ thiếu vạch trên trục N
- [ ] Lưu map
- [ ] Báo cáo sinh map

## Success Criteria

- Client 0.9.15 nạp `HoaLac` trong editor đang Play và nhận được điểm spawn trên mọi trục N.
- Ảnh camera lái trên mỗi trục N cho thấy vạch hai bên làn xe chạy.
- Số làn trong OpenDRIVE của map khớp bảng kiểm kê trên mọi trục N.

## Risk Assessment

- Vạch kẻ thiếu hoặc sai trên nhiều trục. Dấu hiệu: bước 3 hoặc 5. Phản ứng: sửa bằng công cụ vẽ đường của editor cho các trục N; nếu hơn một nửa chiều dài trục N phải sửa tay, dừng và báo để cân nhắc hướng sinh mesh bằng script.
- Công cụ bỏ qua thẻ bề rộng làn. Dấu hiệu: bước 4 cho bề rộng 4,0 m ở mọi nơi. Phản ứng: ghi độ lệch, đưa vào giới hạn của map.
- Lưu map quá lâu hoặc hết đĩa. Kiểm `df -h /` trước khi lưu; cần ít nhất 40 GB trống.
- Nút giao của đường hai chiều tách rời bị lỗi mặt đường. Không chặn mục tiêu vì lái qua nút giao nằm ngoài phạm vi, nhưng phải ghi lại.

## Rollback

Xóa thư mục map `HoaLac` trong dự án CARLA; file `.osm` và bảng kiểm kê không đổi.
