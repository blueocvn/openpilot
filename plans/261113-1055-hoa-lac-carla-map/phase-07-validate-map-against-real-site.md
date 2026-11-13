---
title: "Phase 7: Kiểm chứng map so với hiện trường"
status: todo
priority: P2
effort: "1d"
dependencies: [6]
---

# Phase 7: Kiểm chứng map so với hiện trường

## Overview

Đo mức độ map `HoaLac` khớp với dữ liệu hiện có về khu thật, và kiểm bukapilot lái được trên đó. Vì chưa có log hay video hiện trường, phần "giống thật" chỉ kiểm được về hình học và bằng mắt; báo cáo phải nói rõ phần nào chưa kiểm chứng.

## Key Insights

- Tốc độ dự kiến của tuyến thử nghiệm thật không quá 40 km/h, nên kiểm lái ở 30 và 40 km/h.
- Lái qua nút giao nằm ngoài phạm vi: chỉ chấm các đoạn thẳng giữa hai nút giao.
- Số thật của CARLA chỉ dùng để chấm điểm, không đưa vào bukapilot.
- Camera lái của sim đã được kiểm khớp hình học (FOV 40, cao 1,13 m).

## Requirements

- [ ] Bảng so sánh hình học map với OSM và bảng kiểm kê.
- [ ] Ảnh camera lái đặt cạnh ảnh công khai của cùng vị trí.
- [ ] Kết quả lái engage trên từng trục N.
- [ ] Danh sách giới hạn và phần chưa kiểm chứng.

## Files to Create / Modify

- Create: `plans/261113-1055-hoa-lac-carla-map/reports/hoalac-map-validation.md`.
- Modify: `bukapilot/tools/sim/maps/hoalac/README.md` (mục giới hạn đã biết).

## Implementation Steps

1. Hình học: từ OpenDRIVE của map, tính chiều dài từng trục N, số làn, bề rộng làn; so với OSM và bảng kiểm kê Phase 4.
2. Hình ảnh: tại năm vị trí cố định (ít nhất một trên mỗi trục chính), chụp ảnh camera lái và đặt cạnh ảnh đường phố hoặc ảnh vệ tinh công khai của cùng chỗ. Ghi khác biệt nhìn thấy: màu vạch, dải phân cách, vỉa hè, cây, nhà.
3. Lái: với mỗi trục N và mỗi chiều, chạy bukapilot engage ở 30 và 40 km/h từ điểm spawn tới nút giao kế tiếp, ba lần. Ghi độ lệch ngang so với tim làn (số thật của CARLA, chỉ để chấm), xác suất vạch làn của model, và có rời làn hay không.
4. Hiệu năng: ghi FPS của server ở chất lượng High trong lúc chạy bước 3.
5. Viết báo cáo: bảng kết quả theo từng tiêu chí nghiệm thu của plan, giới hạn (phẳng, bề rộng làn ước lượng, không có đèn và biển báo), và việc cần làm khi có log hiện trường.

Mọi lần chạy ở bước 3 và 4 dùng container và cổng riêng, không dùng container sim đang chạy của người dùng.

## Todo

- [ ] So hình học với OSM và bảng kiểm kê
- [ ] Ảnh đối chiếu tại năm vị trí
- [ ] Chạy engage 30 và 40 km/h trên từng trục N
- [ ] Đo FPS
- [ ] Báo cáo kiểm chứng và giới hạn
- [ ] Cập nhật `README.md` của map

## Success Criteria

- Chiều dài từng trục N lệch không quá 2 % so với OSM; số làn khớp bảng kiểm kê.
- Vạch kẻ làn nhìn thấy trong ảnh camera lái ở cả năm vị trí.
- Trên mọi đoạn thẳng giữa hai nút giao, ở 30 và 40 km/h, ba lần chạy đều không rời làn.
- Server giữ 20 FPS ở chất lượng High.
- Báo cáo liệt kê rõ phần chưa kiểm chứng được vì thiếu dữ liệu hiện trường.

## Risk Assessment

- Xe rời làn trên một trục. Phản ứng: phân biệt lỗi map (thiếu vạch, bề rộng sai) với hành vi của model; lỗi map quay lại Phase 5, hành vi model ghi vào báo cáo và không sửa trong plan này.
- Map trông khác thật nhiều (nhà, cây, màu vạch). Không chặn nghiệm thu; ghi lại để người dùng quyết định có trang trí thêm hay không.
- FPS dưới 20. Phản ứng: thử chất lượng Low và ghi lại; giảm vùng map là quyết định của người dùng.

## Next Steps

Khi có log hoặc video hiện trường: so bề rộng làn và vạch kẻ, chạy lại bước 3, cập nhật bảng kiểm kê. Việc đó là một plan riêng.
