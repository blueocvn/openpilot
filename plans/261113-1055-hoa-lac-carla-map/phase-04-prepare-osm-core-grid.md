---
title: "Phase 4: Chuẩn bị dữ liệu OSM lưới trục N"
status: todo
priority: P2
effort: "1-2d"
dependencies: []
---

# Phase 4: Chuẩn bị dữ liệu OSM lưới trục N

## Overview

> Cập nhật 2026-11-17: người dùng chốt file OSM là `bukapilot/tools/sim/maps/hoalac/hoalac-route.osm`, tách từ bản tải JOSM theo khung vẽ tay và route mask bằng `extract-route.py` (thay cho `fetch-osm.sh` và `hoalac-core.osm`). Bảng kiểm kê làn và việc bổ sung thẻ `lanes`/`width` chưa làm.


Tạo một file `.osm` đã chốt và đã sửa cho lưới trục N ở lõi Khu CNC Hòa Lạc, kèm bảng kiểm kê từng đường. Đây là đầu vào duy nhất của Digital Twin Tool. Phase này không phụ thuộc phase nào và làm song song được trong lúc chờ Phase 3 biên dịch.

## Key Insights

- Khung đã chốt: 105.5270 đến 105.5450 E, 21.0030 đến 21.0190 N (khoảng 1,9 x 1,8 km): N07, N09, N11, N12, N13, N14, N16.
- Đo trên bản tải ngày 2026-11-13: 39,6 km đường chạy xe được; thẻ `lanes` phủ 59,6 % chiều dài, `oneway` 50,2 %, không có thẻ `width`, không có đèn tín hiệu, không có cao độ. Đường `service` (14,4 km) gần như không có thẻ.
- Bộ chuyển đổi của CARLA mặc định làn rộng 4,0 m khi OSM không ghi bề rộng. Bề rộng làn thật tại hiện trường chưa biết.
- Các trục N được vẽ hai chiều tách rời; ở luồng standalone, nút giao của chúng sinh mesh chồng chéo (ảnh trong báo cáo nghiên cứu).
- Overpass API có lúc trả 502 khi quá tải; script tải phải thử lại.
- Chưa có dữ liệu hiện trường: đối chiếu chỉ bằng ảnh vệ tinh và ảnh đường phố công khai.

## Requirements

- [ ] File `.osm` nằm trong repo, tải lại được bằng một lệnh, có ghi nguồn ODbL.
- [ ] Mỗi trục N trong phạm vi có: số làn mỗi chiều, chiều đường, có dải phân cách hay không, bề rộng làn ước lượng, loại và màu vạch.
- [ ] Hình học được kiểm bằng luồng standalone trước khi đưa vào Digital Twin Tool.

## Files to Create / Modify

- Create: `bukapilot/tools/sim/maps/hoalac/fetch-osm.sh` (tải khung đã chốt từ Overpass, có thử lại).
- Create: `bukapilot/tools/sim/maps/hoalac/hoalac-core.osm` (bản đã chốt và đã sửa).
- Create: `bukapilot/tools/sim/maps/hoalac/osm-to-xodr.py` (chuyển sang OpenDRIVE bằng `carla.Osm2Odr`, phát triển từ `research/probe-osm-to-xodr.py`).
- Create: `bukapilot/tools/sim/maps/hoalac/preview-standalone.py` (nạp OpenDRIVE vào server tạm và chụp ảnh, phát triển từ `research/probe-standalone-render.py`).
- Create: `bukapilot/tools/sim/maps/hoalac/README.md` (nguồn dữ liệu, bảng kiểm kê đường, cách tái tạo).

## Implementation Steps

1. Viết `fetch-osm.sh`, tải khung đã chốt gồm cả đường và nhà (Digital Twin Tool dùng footprint nhà). Lưu bản gốc kèm ngày tải.
2. Vẽ sơ đồ các đường trong khung có ghi tên (một ảnh PNG) và gửi người dùng xác nhận phạm vi và các trục thực sự chạy thử.
3. Lập bảng kiểm kê trong `README.md`: với từng trục N, đọc ảnh vệ tinh và ảnh đường phố công khai để ghi số làn, chiều, dải phân cách, bề rộng làn (đo trên ảnh vệ tinh), vạch tim và vạch chia làn. Ghi rõ nguồn và ngày chụp của từng ảnh.
4. Sửa file `.osm` cục bộ bằng JOSM: bổ sung `lanes`, `oneway`, `width` hoặc `lanes:width` còn thiếu trên trục N; bỏ đường `track`, `path`, `footway` và đường `service` không thuộc tuyến chạy thử.
5. Chạy `osm-to-xodr.py` và `preview-standalone.py` trên một server tạm (container và cổng riêng); xem ảnh từ trên xuống của từng nút giao giữa hai trục N.
6. So chiều dài từng trục N trong OpenDRIVE với chiều dài tính từ OSM.
7. Ghi các giới hạn đã biết vào `README.md`: map phẳng, bề rộng làn là ước lượng từ ảnh.

## Todo

- [ ] `fetch-osm.sh` và bản OSM gốc
- [ ] Sơ đồ đường có tên, người dùng xác nhận phạm vi
- [ ] Bảng kiểm kê từng trục N
- [ ] Sửa thẻ trong `hoalac-core.osm`
- [ ] Xem trước bằng standalone, kiểm nút giao
- [ ] So chiều dài trục N với OSM
- [ ] `README.md` có nguồn và giới hạn

## Success Criteria

- `bash fetch-osm.sh` tái tạo được bản gốc; `hoalac-core.osm` chuyển sang OpenDRIVE không lỗi.
- Mọi trục N trong phạm vi có đủ `lanes` và `oneway`; bảng kiểm kê không còn ô trống.
- Chiều dài từng trục N trong OpenDRIVE lệch không quá 2 % so với OSM.
- Người dùng đã xác nhận sơ đồ phạm vi.

## Risk Assessment

- Ảnh đường phố công khai không phủ khu này. Dấu hiệu: không tìm được ảnh cho một trục. Phản ứng: chỉ dùng ảnh vệ tinh cho trục đó và đánh dấu "ước lượng" trong bảng kiểm kê.
- Bề rộng làn sai làm xe trong sim bám làn khác ngoài thật. Không kiểm chứng được khi chưa có dữ liệu hiện trường; ghi là giới hạn và sửa khi có log thật.
- Digital Twin Tool có thể không đọc thẻ bề rộng. Kiểm ở Phase 5; nếu không đọc, ghi độ lệch vào báo cáo kiểm chứng.

## Security Considerations

Dữ liệu OSM theo giấy phép ODbL: giữ dòng ghi nguồn "© OpenStreetMap contributors" trong `README.md` và trong tài liệu của map. Ảnh vệ tinh chỉ dùng để đo, không đưa vào repo.

## Rollback

Xóa thư mục `bukapilot/tools/sim/maps/hoalac/`.
