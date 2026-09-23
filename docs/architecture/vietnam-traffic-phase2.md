# Vietnam traffic mode — kiến trúc Phase 2

Trạng thái: **đã triển khai dưới cờ thử nghiệm, chưa nghiệm thu** trên nhánh
`codex/vietnam-traffic-phase2` (từ commit `8ad4325` của Phase 1; code Phase 2
ở `f8efb82` và `1228c9c`). Tài liệu này mô tả *mã hiện có*, không phải kiến
trúc mong muốn trong [kế hoạch Phase 2](../plans/2026-09-22-vietnam-traffic-phase2.md).
Phạm vi chỉ là CARLA trong WSL; không bật trên xe thật.

## Luồng điều khiển và ranh giới

```text
CARLA camera + trạng thái ego
  → bridge mô phỏng → modeld / radard → plannerd (MPC, cruise, model)
  → chọn yêu cầu gia tốc stock → [VN traffic policy, opt-in]
  → longitudinalPlan.aTarget → controlsd / LongControl
  → carControl.actuators.accel → bridge P/I → throttle hoặc brake CARLA
  → CARLA ego → cảm biến ở tick tiếp theo

CARLA actor/ground truth → recorder → report/analyzer (chỉ quan sát)
                                ↛ planner/modelV2/radarState
```

| Thành phần | Vai trò và thay đổi |
| --- | --- |
| [`motorcycle_weave.py`](../../openpilot/tools/sim/bridge/carla/scenes/motorcycle_weave.py) | Scene Phase 1 vẫn tạo xe máy; Phase 2 chỉ bổ sung mẫu gia tốc dọc CARLA ở 10 Hz và mode/profile vào manifest. Không dùng actor để điều khiển planner. |
| [`longitudinal_planner.py`](../../openpilot/selfdrive/controls/lib/longitudinal_planner.py) | Tính time-gap Phase 2 từ radar lead trước `mpc.update`, rồi áp dụng ramp gia tốc dương sau khi chọn gia tốc. Mode 0 truyền `None` vào MPC và giữ đường stock. |
| [`vn_traffic_policy.py`](../../openpilot/selfdrive/controls/lib/vn_traffic_policy.py) | Hai policy thuần: time-gap cho lead đóng nhanh đã quan sát và ramp gia tốc dương. Không import simulator/CARLA, không nhận actor ID hay ground truth. |
| [`long_mpc.py`](../../openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py) | Nhận `t_follow_override` tùy chọn. `None` dùng personality stock; override không được ngắn hơn stock. MPC vẫn tự sinh trajectory/gia tốc. |
| [`controlsd.py`](../../openpilot/selfdrive/controls/controlsd.py), [`longcontrol.py`](../../openpilot/selfdrive/controls/lib/longcontrol.py) | Đường điều khiển stock: `aTarget` qua LongControl để thành `carControl.actuators.accel`. Phase 2 không sửa hai file này. |
| [`common.py`](../../openpilot/tools/sim/bridge/common.py) | Bridge đổi gia tốc yêu cầu thành ga/phanh CARLA bằng vòng P/I và saturation hiện có; Phase 2 không đổi bridge. |
| [`analyze_vn_traffic.py`](../../openpilot/tools/sim/analyze_vn_traffic.py) | Đọc report JSONL xoay theo kiểu streaming, tính comfort/safety và so sánh một cặp mode 0/1. Không gửi tín hiệu ngược vào Openpilot. |

Model weights, `modeld`, `radard`, lựa chọn lead, MPC obstacle/danger cost và
giới hạn phanh giữ nguyên. Phase 2 tăng có điều kiện time-gap của personality
trước khi MPC giải trajectory, rồi áp dụng ramp dương lên `aTarget` đã chọn.
Mode 0 tiếp tục dùng time-gap personality stock.

## Điều kiện bật và thuật toán

Planner đọc `VN_TRAFFIC_MODE=0|1` và
`VN_TRAFFIC_PROFILE=balanced|gentle|responsive` **một lần khi khởi tạo**;
giá trị không hợp lệ gây lỗi khởi động. Mặc định là `0` và `balanced`.
[`launch_openpilot.sh`](../../openpilot/tools/sim/launch_openpilot.sh) đặt
`SIMULATION=1`. Khi mode bật, policy chỉ có hiệu lực nếu Openpilot longitudinal
đang active, experimental mode tắt, không có `shouldStop`/standstill và các
message đầu vào qua kiểm tra hợp lệ. Bất kỳ điều kiện nào không đạt đều trả
nguyên yêu cầu stock và reset ramp. Trên môi trường không phải mô phỏng,
mode luôn vô hiệu dù biến môi trường được đặt.

Với yêu cầu gia tốc `a > 0`, profile cung cấp trần gia tốc và tốc độ tăng
gia tốc dương:

| Profile | Trần gia tốc (m/s²) | Ramp (m/s³) |
| --- | ---: | ---: |
| `balanced` | 1,0 | 0,8 |
| `gentle` | 0,8 | 0,6 |
| `responsive` | 1,2 | 1,0 |

Ở mỗi tick `dt`, policy tính
`limited = min(a, accel_cap, max(previous_output, 0) + ramp * dt)`.
Dưới 40 km/h, output là `limited`. Từ 40 đến 50 km/h, output nội suy tuyến tính
về `a`; từ 50 km/h trở lên hoàn toàn stock. Output không lớn hơn yêu cầu gốc.
Yêu cầu `a <= 0` được truyền nguyên, đồng thời reset ramp: policy **không tạo
lệnh phanh mới và không trì hoãn yêu cầu giảm tốc**. Nó cũng không thay đổi
`shouldStop`. Đây là hạn chế có chủ đích: rung giật do lead/perception hoặc
pha giảm tốc có thể không được cải thiện.

Trước MPC, follow policy xét cả `leadOne` và `leadTwo` mới không quá 250 ms.
Với tốc độ đóng trên 0,5 m/s, TTC từ 5 xuống 3 giây làm tăng time-gap từ 0 tới
0,30 giây. Mức tăng đầy đủ tới 40 km/h, giảm về 0 ở 50 km/h, không vượt tổng
2,05 giây; rise/fall lần lượt là 0,30/0,15 giây mỗi giây. MPC quyết định gia
tốc từ time-gap này. Policy không ép brake và không dùng actor CARLA.

## Quan sát và đối chiếu A/B

Scene ghi event/contact ngay khi xảy ra, ground truth ở 2 Hz và
`control_sample` ở 10 Hz. Mẫu 10 Hz chứa gia tốc dọc đo trực tiếp từ CARLA
(chiếu vector gia tốc lên hướng tiến của ego), tốc độ, phanh thực tế và
snapshot Openpilot/bridge. File JSONL xoay ở 10 MiB. `manifest.json` ghi
mode/profile, cấu hình scene, Params và hash source/ONNX/compiled artifact;
`compiled_source_link_verified=false` được báo riêng như giới hạn về nguồn
build. A/B tương đối yêu cầu receipt của mỗi run khớp manifest và map hash các
chunk artifact thực sự được modeld nạp phải giống hệt giữa Mode 0 và Mode 1.

Analyzer tính jerk thực tế từ gia tốc CARLA và timestamp thực; report cũ không
có gia tốc trực tiếp chỉ được phân tích bằng chênh lệch tốc độ, và scorer không
ghép hai nguồn đo khác nhau. Comfort sau contact đầu tiên bị loại khỏi phép
chấm nhưng vẫn báo thời lượng bị loại. Contact được đếm theo actor khác nhau,
đồng thời lưu số callback. TTC dùng **lead quan sát qua radar**; clearance và
sai số quỹ đạo dùng ground truth **chỉ trong analyzer**.

Scorer cặp A/B kiểm tra mode 0→1, cùng commit/seed/map/Params/hash, hoàn thành
thời lượng hữu hạn, telemetry gần 10 Hz, longitudinal active ở ít nhất 90%
mẫu và gate tracking xe máy. Sau đó mới kiểm tra giảm ≥15% P95 jerk thực tế,
không tăng số lần đổi tăng/giảm tốc, quãng đường ≥95% stock, contact và
disengagement không tăng, TTC không giảm quá 0,2 s và clearance không giảm
quá 0,25 m. `data_valid=true` trong output chỉ nói **control telemetry** hợp lệ;
nó không chứng nhận toàn bộ baseline Phase 1.

Lệnh chạy có giao diện và lệnh chấm nằm trong
[`README.md`](../../openpilot/tools/sim/carla/README.md). Cặp chẩn đoán seed 42
được ghi ở [báo cáo Phase 2](../reports/2026-09-22-vietnam-phase2-preliminary.md):
jerk giảm nhưng candidate có contact, nên `pass=false`. Không suy ra quan hệ
nhân quả từ một cặp run.

## Chưa có / điều kiện trước khi coi là cải thiện

- Gate Phase 1 chưa đóng: tracking actor vượt ngưỡng; provenance biên dịch
  Tinygrad chưa chứng minh; chưa có clip đồng bộ và ghép lead đúng actor.
- Chưa ghi/replay đầy đủ message đầu vào planner, chưa chứng minh parity trên
  một trace runtime; unit test chỉ kiểm tra policy và một ca tích hợp planner.
- Chưa có scene đường trống/xe chỉ đi cạnh và lead đều, chưa chạy đủ các seed
  tuning/validation hoặc chọn profile theo tổng hợp. Một cặp seed 42 không đủ
  để nghiệm thu hay tuyên bố an toàn.
- Mode phải giữ mặc định tắt. Không dùng bản này trên xe thực tế; nếu tiếp tục,
  đóng gate Phase 1 rồi chạy lại A/B có giao diện theo kế hoạch đã chốt.
