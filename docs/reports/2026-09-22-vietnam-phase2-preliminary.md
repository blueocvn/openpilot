# Phase 2 v1 — kết quả chẩn đoán đầu tiên, chưa nghiệm thu

Nhánh `codex/vietnam-traffic-phase2`, commit code `f8efb82`. Hai ca chạy tuần tự
trong WSL, có giao diện, Town04_Opt/spawn 40, seed 42, `motorcycle_weave`
alternating, 60 giây sau ready, Openpilot longitudinal active và experimental
mode tắt. Mode 0 dùng cùng planner/scene/model/bridge, chỉ khác biến môi trường
`VN_TRAFFIC_MODE`; candidate dùng profile `balanced`. Manifest hai ca có cùng
commit, source hash, ONNX hash, compiled-artifact hash, Params, CARLA version và
map. Artifact Tinygrad vẫn chưa có bằng chứng nguồn biên dịch trực tiếp từ ONNX.

| Phép đo | Stock (mode 0) | Balanced (mode 1) |
| --- | ---: | ---: |
| Control samples / long active | 601 / 601 | 601 / 601 |
| P95 độ lớn jerk thực tế (m/s³) | 12,614 | 9,914 |
| Số lần đổi tăng/giảm tốc | 16 | 12 |
| Quãng đường (m) | 425,592 | 417,550 |
| Contact: actor / callback | 0 / 0 | 1 / 6 |
| TTC lead quan sát nhỏ nhất (s) | 2,076 | 1,804 |
| Clearance actor nhỏ nhất (m) | 1,066 | 0,013 |
| Sai số ngang P95 / tối đa (m) | 0,658 / 0,933 | 0,624 / 0,934 |
| Lead ghép đúng actor tạt đầu | 0 | 0 |

Report stock: `.carla/reports/motorcycle-weave-20260922-044329-396168628/`.
Report candidate: `.carla/reports/motorcycle-weave-20260922-044523-832856067/`.

Jerk P95 giảm khoảng 21,4% và quãng đường đạt 98,1% stock trong *cặp này*,
nhưng candidate có contact ở giây 59,8 khi stock không có. TTC giảm 0,273 giây
và clearance giảm 1,054 m, đều vượt ngưỡng cho phép. Không thể kết luận mode
gây contact chỉ từ một cặp chạy; cũng không thể gọi đây là cải thiện an toàn.
Scorer trả `pass=false`.

Quan trọng hơn, cả hai ca vẫn trượt gate quỹ đạo Phase 1 (P95 ≤0,4 m và tối đa
≤0,8 m), và chưa có replay đủ input planner, provenance biên dịch model hoặc
clip đồng bộ. 10 Hz control telemetry của hai ca hợp lệ; điều đó **không** đồng
nghĩa baseline tổng thể hợp lệ. Không dùng cặp này để chọn profile hay nghiệm
thu Phase 2. Mode giữ mặc định tắt, giới hạn ở `SIMULATION=1`; không triển khai
lên xe thật.

Việc còn lại theo kế hoạch: đóng các gate Phase 1, ghi/replay input planner,
chạy ca đường trống và lead đều, chạy đủ tập tuning/validation nhiều seed, lưu
clip và chỉ chọn profile nếu tất cả ràng buộc comfort/safety cùng đạt.
