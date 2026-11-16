---
title: "Clone đường Khu CNC Hòa Lạc vào CARLA từ OpenStreetMap"
description: "Thử chuyển sim bukapilot sang CARLA 0.9.15, nếu đạt thì chuyển hẳn và dựng map lưới trục N của Khu CNC Hòa Lạc bằng Digital Twin Tool từ dữ liệu OSM."
status: pending
priority: P1
effort: 8-10d
branch: hola-world
tags: [feature, infra, sim, carla, map]
blockedBy: []
blocks: []
created: 2026-11-13
---

# Clone đường Khu CNC Hòa Lạc vào CARLA từ OpenStreetMap

## Overview

Thử chuyển sim bukapilot sang CARLA 0.9.15, nếu đạt thì chuyển hẳn và dựng map lưới trục N của Khu CNC Hòa Lạc bằng Digital Twin Tool từ dữ liệu OSM. Lý do: không khảo sát được hiện trường, chỉ làm tại văn phòng, nên nguồn dữ liệu là OpenStreetMap và ảnh công khai.

Bằng chứng khả thi và nguồn: [báo cáo nghiên cứu](./research/researcher-01-osm-to-carla-feasibility.md).

## Kết quả mong muốn

Chạy được `bash run.sh all --carla-town HoaLac --carla-spawn-point <n>` trong `bukapilot/tools/sim-linux-docker/`: xe bukapilot engage và bám làn trên các trục N07 đến N16 của bản clone, với vạch kẻ làn nhìn thấy trong camera lái.

## Ràng buộc

- Chỉ dùng dữ liệu lấy được từ văn phòng: OSM (ODbL, phải ghi nguồn), ảnh vệ tinh và ảnh đường phố công khai. Chưa có log hay video hiện trường.
- Sim phải giống xe thật: không thêm nhánh code chỉ dành cho sim, không thêm cảm biến xe thật không có.
- Không chạy thử trong container `bukapilot_sim_carla` và `carla_server` đang dùng; mọi thử nghiệm chạy ở container và cổng riêng (tránh 2000 đến 2002 của sim và các cổng 2100, 2110, ... của `bench.sh`; plan dùng 2300).
- Đĩa còn 283 GB: đủ cho bản dựng từ source (khoảng 130 GB) và image 0.9.15 (khoảng 30 GB), không đủ cho luồng nhập map bằng Docker của CARLA (600 đến 700 GB).
- Bukapilot chạy Python 3.8: client CARLA mới nhất dùng được là 0.9.15.

## Ngoài phạm vi

- Cao độ và địa hình thật (OSM không có; map sẽ phẳng).
- Đèn tín hiệu, biển báo đúng vị trí thật.
- Tuyến 2,8 km trong khuôn viên ĐHQGHN.
- Lái qua nút giao (đã tách thành bài toán riêng).
- Kịch bản giao thông, xe khác, người đi bộ trên map mới.
- Huấn luyện lại model.

## Tiêu chí nghiệm thu

| # | Tiêu chí | Cách kiểm |
|---|---|---|
| 1 | Cổng swap: sim bukapilot chạy trên 0.9.15 với kết quả đạt/không đạt của từng ô benchmark giống 0.9.12 | Phase 1, bảng so sánh |
| 2 | `tools/sim-linux-docker` và nhánh WSL `tools/sim` chạy CARLA 0.9.15; bỏ hẳn 0.9.12 (người dùng chọn ngày 2026-11-16) | Phase 2 |
| 3 | Map `HoaLac` nạp được trên server 0.9.15 đóng gói, có điểm spawn trên mọi trục N trong phạm vi | Phase 6 |
| 4 | Chiều dài từng trục N trong sim lệch không quá 2 % so với OSM; số làn khớp bảng kiểm kê | Phase 7 |
| 5 | Vạch kẻ làn hiện trong ảnh camera lái (FOV 40, cao 1,13 m) trên mọi trục N | Phase 5 và 7 |
| 6 | Bukapilot engage, chạy 30 và 40 km/h hết từng đoạn thẳng giữa hai nút giao mà không rời làn | Phase 7 |
| 7 | Server giữ 20 FPS ở chất lượng High trên máy văn phòng | Phase 7 |

## Phases

| # | Phase | Status |
|---|-------|--------|
| 1 | [Thử swap sim sang CARLA 0.9.15](./phase-01-trial-swap-to-carla-0915.md) | Done: cổng đạt, đã xác nhận ([báo cáo](./reports/swap-trial-0915.md)) |
| 2 | [Chuyển mặc định sang 0.9.15](./phase-02-switch-default-to-0915.md) | Done, chưa có baseline benchmark |
| 3 | [Dựng CARLA 0.9.15 từ source](./phase-03-build-carla-0915-source.md) | Pending |
| 4 | [Chuẩn bị dữ liệu OSM lưới trục N](./phase-04-prepare-osm-core-grid.md) | Pending |
| 5 | [Sinh map bằng Digital Twin Tool](./phase-05-generate-digital-twin-map.md) | Pending |
| 6 | [Đóng gói và đưa map vào sim](./phase-06-package-and-deploy-map.md) | Pending |
| 7 | [Kiểm chứng map so với hiện trường](./phase-07-validate-map-against-real-site.md) | Pending |

## Phụ thuộc

- Phase 1 là cổng quyết định. Không đạt thì dừng và lập lại plan theo hướng "giữ 0.9.12, sinh mesh bằng script" (xem báo cáo, mục 4).
- Phase 2 và 3 cần Phase 1 đạt. Phase 4 không phụ thuộc phase nào, làm song song được.
- Phase 3 cần người dùng có tài khoản Epic Games liên kết GitHub (để tải Unreal Engine 4.26 bản của CARLA).
- Phase 5 cần Phase 3 và 4. Phase 6 cần 2 và 5. Phase 7 cần 6.
- Ảnh hưởng chéo: plan `bukapilot/plans/261112-1710-static-obstacle-braking-experiments` đo baseline trên 0.9.12; Phase 2 đo lại baseline trên 0.9.15.

## Giả định mang tải

| Giả định | Dấu hiệu sai | Phản ứng đã định |
|---|---|---|
| Bridge 0.9.12 chạy được với client và server 0.9.15 mà không sửa lớn | Phase 1 không qua cổng | Dừng, lập lại plan theo hướng giữ 0.9.12 |
| Digital Twin Tool sinh vạch kẻ làn dùng được cho model camera | Phase 5: vạch thiếu trên trục N | Sửa trong editor; nếu vẫn thiếu, dừng và báo |
| Map lưu từ Digital Twin đóng gói được để chạy trên server không có editor | Phase 6, bước đầu tiên | Chạy server từ bản dựng source thay cho image đóng gói |
| Unreal 4.26 biên dịch và mở editor được trong container Ubuntu 20.04 trên máy Ubuntu 26.04 với RTX 5060 Ti | Phase 3: biên dịch hoặc editor lỗi | Thử Ubuntu 18.04; nếu vẫn lỗi, dừng và báo |

## Validation Log

### Session 1 — 2026-11-13
**Trigger:** chốt hướng và phạm vi trước khi viết plan.
**Questions asked:** 4

#### Questions & Answers

1. **[Architecture]** Bạn muốn plan đi theo hướng dựng map nào?
   - Options: Hai mốc: hình học rồi Digital Twin (Recommended) | Chỉ hình học, không build Unreal | Giữ CARLA 0.9.12, dựng mesh riêng
   - **Answer:** Other
   - **Custom input:** "tại sao không gợi ý carla 0.9.12 mà lại follow 0.9.15"
   - **Rationale:** quyết định phiên bản CARLA và toàn bộ công cụ dựng map.
2. **[Architecture]** (sau khi giải thích) Bạn chốt hướng nào?
   - Options: Giữ 0.9.12, sinh mesh bằng script (Recommended) | Chuyển sang 0.9.15 và Digital Twin Tool | Chỉ hình học trên 0.9.12, không build Unreal
   - **Answer:** Chuyển sang 0.9.15 và Digital Twin Tool
   - **Custom input:** "bướ đầu sẽ là swap thử qua 9.15 nếu khả thi ta sẽ chuyển qua hẳn rồi build map"
   - **Rationale:** đặt việc thử swap làm cổng đầu tiên, trước mọi việc dựng map.
3. **[Scope]** 'Khu đường chạy thử xe' muốn clone là phần nào của Khu CNC Hòa Lạc?
   - Options: Lưới trục N ở lõi khu (Recommended) | Một vòng kín ngắn 2 đến 4 km | Toàn bộ khu (khoảng 4,5 x 4 km)
   - **Answer:** Lưới trục N ở lõi khu
   - **Rationale:** quyết định khung OSM và khối lượng sửa dữ liệu.
4. **[Assumptions]** Đang có sẵn dữ liệu nào ghi tại khu Hòa Lạc?
   - Options: Log lái xe thật | Video dashcam hoặc điện thoại | Chưa có gì
   - **Answer:** Chưa có gì
   - **Rationale:** việc đối chiếu chỉ dựa vào ảnh công khai; độ giống thật chưa kiểm chứng được bằng log.

#### Confirmed Decisions
- Phiên bản: thử 0.9.15 trước, đạt thì chuyển hẳn. Người dùng chọn, dù khuyến nghị lúc đó là giữ 0.9.12.
- Công cụ dựng map: Digital Twin Tool của CARLA 0.9.15.
- Phạm vi: lưới trục N ở lõi khu, khoảng 1,9 x 1,8 km.
- Dữ liệu đối chiếu: chỉ nguồn công khai.
- 2026-11-16: ở lại CARLA 0.9.15, không lên 0.9.16. `carla==0.9.16` trên PyPI chỉ có wheel cp310 đến cp312, bukapilot chạy
  Python 3.8; build client 0.9.16 cho 3.8 hoặc nâng Python không đáng công khi Digital Twin Tool đã có trong 0.9.15.
- 2026-11-16: bỏ hẳn 0.9.12 (cả `tools/sim-linux-docker` lẫn nhánh WSL `tools/sim`), xóa image 0.9.12.

#### Impact on Phases
- Phase 1 là cổng swap; Phase 2 chỉ chạy khi cổng đạt.
- Phase 4 và 7 dùng ảnh vệ tinh và ảnh đường phố công khai thay cho log hiện trường.

## Red Team Review

Chưa chạy. Chạy `/ak-plan red-team /home/ubuntu/openpilot/plans/261113-1055-hoa-lac-carla-map` nếu muốn phản biện trước khi làm.

<!-- slug: hoa-lac-carla-map -->
