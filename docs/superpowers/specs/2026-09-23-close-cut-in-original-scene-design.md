# Close Cut-In Original Scene Design

## Mục tiêu

Tạo một biến thể scene CARLA trong đó một xe máy đang chạy ở làn kế bên tạt
vào rất sát phía trước ego ở 30 km/h. Khi chạy với `VN_TRAFFIC_MODE=0`, contact
với xe máy là kết quả baseline được mong đợi. Scene này sẽ được giữ nguyên để
sau đó đánh giá `VN_TRAFFIC_MODE=1` hoặc một enhancement mới có tránh được
contact hay không.

Tên branch không còn mang nghĩa Phase 1/Phase 2. Trong mọi tài liệu và report:

- **Original** nghĩa là `VN_TRAFFIC_MODE=0`.
- **Enhancement** nghĩa là `VN_TRAFFIC_MODE=1`.
- Cả hai mode phải dùng cùng scene, seed, map, spawn point và model artifact.

## Phạm vi hiện tại

Chỉ triển khai scene, cấu hình, telemetry, analyzer và tài liệu chạy. Không sửa
model weights, `modeld`, `radard`, planner, MPC, `vn_traffic_policy.py`, bridge
longitudinal controller hoặc actuator command để tạo ra contact hay phanh.

## Giao diện scene

Giữ `--carla-scene motorcycle_weave` và `--carla-scene-case` để chọn hướng.
Thêm:

```text
--carla-scene-variant standard|close_cut_in
```

`standard` giữ nguyên hành vi hiện tại. `close_cut_in` tạo đúng một cut-in trong
một episode. Để kiểm tra đủ hai hướng, chạy hai episode riêng với
`right_to_left` và `left_to_right`. `alternating` vẫn hợp lệ, nhưng với variant
một sự kiện nó bắt đầu bằng `right_to_left` như scheduler hiện tại.

## Hình học và động lực học

- Ego cruise: 30 km/h; chỉ bắt đầu sau khi Openpilot longitudinal active và
  `vEgo >= 7.5 m/s`.
- Xe máy được lấy từ background traffic đang chuyển động trong làn kế bên.
  Không spawn chồng lấn, không teleport ra trước ego, không ép velocity mỗi tick.
- Tốc độ mục tiêu xe máy: `4.0–4.8 m/s`.
- Khoảng hở footprint thực đo khi xe máy vào làn ego: mục tiêu `2.5–3.5 m`.
- Thời gian đi từ làn bên cạnh tới giữa làn ego: `1.0–1.3 s`.
- Thời gian giữ trong làn ego: `1.5–2.0 s`.
- Tốc độ đóng và TTC lúc vào làn phải được ghi từ trạng thái thực tế, không suy
  ra từ lịch dự kiến.
- Scheduler commit seed/direction chỉ sau khi actor hợp lệ được promote. Nếu
  đường, actor hoặc vị trí không hợp lệ, event được hoãn mà không tiêu thụ RNG.
- Sau khi event đầu tiên được commit, không tạo thêm cut-in trong episode. Scene
  vẫn ghi telemetry tới hết `--carla-scene-duration` để giữ phần hậu sự kiện.

Khoảng cách này phải đủ gần để Original thường contact nếu không phản ứng kịp,
nhưng contact phải đến từ chuyển động tương đối sau khi actor đã được promote,
không đến từ spawn overlap hay teleport.

## Telemetry và kết quả

`scenario_started`, `cut_in_started` và `manifest.json` ghi `scene_variant`.
Collision callback ghi vai trò actor: `active_cut_in`, `background` hoặc `other`.
Analyzer phải đếm contact với active cut-in ngay cả khi va chạm làm event không
bao giờ phát `cut_in_finished`.

Summary của mỗi run phải có tối thiểu:

- variant, case và seed;
- số cut-in đã start/enter/finish;
- actual entry gap, ego speed, motorcycle speed, closing speed và entry TTC;
- contact với active cut-in, contact với actor khác, timestamp và impulse;
- lead observation, requested acceleration, actual acceleration và CARLA brake
  hiện có để dùng cho phân tích sau.

## Nghiệm thu scene Original

Chạy có giao diện trong WSL, `VN_TRAFFIC_MODE=0`, duration 20 giây, seed 42–44
cho từng hướng. Giữ tất cả run, kể cả run không contact.

Scene đạt khi:

- mỗi run có đúng một cut-in start và không có cut-in thứ hai;
- actor được promote từ background, không spawn overlap, không front teleport;
- actual entry gap nằm trong `2.5–3.5 m` và actor tracking vẫn trong gate hiện có
  trước contact;
- contact đúng active cut-in xảy ra ở ít nhất 2/3 run của mỗi hướng;
- contact với background/other actor được báo riêng;
- telemetry tiếp tục đến hết episode hoặc ghi rõ lỗi mô phỏng khiến episode kết
  thúc sớm.

Sau khi đạt, đóng băng variant và seed protocol. Công việc enhancement tránh
contact sẽ có design/plan riêng và dùng lại chính scene đã đóng băng này.
