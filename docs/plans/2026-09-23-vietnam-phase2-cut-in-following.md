# Phase 2: phản ứng bám xe khi xe máy tạt đầu

## Mục tiêu

Phase 2 dùng lead mà Openpilot đã nhận qua `radarState` để tăng nhẹ khoảng
thời gian bám xe ở tốc độ đô thị. Thay đổi chỉ bật trong CARLA bằng
`VN_TRAFFIC_MODE=1`. Artifact model hiện tại được giữ nguyên.

## Thiết kế đã triển khai

- `TrafficFollowPolicy` đọc `leadOne` và `leadTwo` hợp lệ, mới không quá
  250 ms. Policy chỉ kích hoạt khi tốc độ đóng lớn hơn 0,5 m/s và TTC dưới
  5 giây.
- Mức tăng khoảng thời gian được nội suy từ 0 tại TTC 5 giây đến tối đa
  0,30 giây tại TTC 3 giây. Hiệu lực đầy đủ tới 40 km/h, giảm dần và bằng
  không ở 50 km/h. Tổng thời gian không vượt 2,05 giây.
- Giá trị tăng với tốc độ 0,30 giây/giây và giảm với tốc độ 0,15 giây/giây
  khi nguy cơ hết. Input lỗi, cũ hoặc mode không active trả về stock ngay.
- `LongitudinalMpc.update(..., t_follow_override=None)` giữ nguyên đường stock.
  Override được từ chối nếu không hữu hạn hoặc ngắn hơn khoảng stock.
- Policy không tạo lệnh phanh trực tiếp. MPC tự tính trajectory và gia tốc từ
  khoảng bám mới. Positive acceleration ramp Phase 2 hiện có vẫn chạy sau bước
  chọn gia tốc.

## Quan sát và đánh giá

Planner trace ghi riêng input model, `leadOne`, `leadTwo`, nguồn MPC và quyết
định của TrafficFollowPolicy theo từng chu kỳ. Analyzer báo từng tầng:
model candidate, radar lead, policy active và lead slot được dùng. Phân tích
shadow chỉ đọc report sau khi chạy; dữ liệu actor CARLA không đi vào planner.

Phép A/B yêu cầu receipt runtime đầy đủ của từng run và map hash chunk model
được nạp phải giống nhau. Chứng minh nguồn ONNX và build recipe vẫn được báo
riêng bằng `compiled_source_link_verified`; nó không chặn phép so sánh tương
đối khi cả hai run đã xác nhận dùng cùng artifact runtime.

## Vì sao chưa sửa detection

Run chẩn đoán cho thấy model candidate và radar lead tiến gần quỹ đạo xe máy,
nhưng sai lệch hệ tọa độ và nhiều actor khiến việc gán đúng xe còn mơ hồ.
Thay threshold detection lúc này có thể làm tăng false lead. Bước hiện tại
tăng phản ứng với lead đã hợp lệ; bước detection cần dataset có nhãn và phép
đo precision/recall riêng trước khi thay đổi model hoặc radar association.

## Nghiệm thu còn lại

Chạy có giao diện trong WSL, cùng model runtime, scene và seed cho Mode 0/1.
Xác nhận replay Mode 0 khớp Phase 1 stock, planner output diagnostic đủ mọi
tick, sau đó đối chiếu TTC, gia tốc yêu cầu, gia tốc thực, jerk, clearance và
contact. Trạng thái `brake > 0` không phải điều kiện riêng vì giảm tốc có thể
được thực hiện bằng coasting hoặc gia tốc âm trước khi bridge cần phanh.
