# Vietnam traffic mode — kiến trúc Phase 2

Trạng thái: **đã triển khai dưới cờ thử nghiệm, chưa nghiệm thu**. Phần radar
cut-in bổ sung ngày 2026-09-24 trên nhánh `codex/motorcycle-radar-cutin`;
phần model-only phía dưới ghi lại thiết kế và kết quả ngày 2026-09-23.
Phạm vi chỉ là CARLA trong WSL; không bật trên xe thật.

## Luồng điều khiển và ranh giới

```text
CARLA camera + trạng thái ego
  → bridge mô phỏng → modeld / radard → plannerd
  → [dự đoán cut-in từ modelV2, opt-in] → MPC (+ radar lead 0/1)
  → chọn yêu cầu gia tốc → [VN traffic acceleration ramp, opt-in]
  → longitudinalPlan.aTarget → controlsd / LongControl
  → carControl.actuators.accel → bridge P/I → throttle hoặc brake CARLA
  → CARLA ego → cảm biến ở tick tiếp theo

CARLA actor/ground truth → recorder → report/analyzer (chỉ quan sát)
                                ↛ planner/modelV2/radarState
```

| Thành phần | Vai trò và thay đổi |
| --- | --- |
| [`motorcycle_weave.py`](../../openpilot/tools/sim/bridge/carla/scenes/motorcycle_weave.py) | Scene Phase 1 vẫn tạo xe máy; Phase 2 chỉ bổ sung mẫu gia tốc dọc CARLA ở 10 Hz và mode/profile vào manifest. Không dùng actor để điều khiển planner. |
| [`longitudinal_planner.py`](../../openpilot/selfdrive/controls/lib/longitudinal_planner.py) | Tính time-gap Phase 2 và dự đoán cut-in trước `mpc.update`, rồi áp dụng ramp gia tốc dương sau khi chọn gia tốc. Mode 0 gọi MPC như trước thay đổi. |
| [`vn_traffic_policy.py`](../../openpilot/selfdrive/controls/lib/vn_traffic_policy.py) | Hai policy thuần: time-gap cho lead đóng nhanh đã quan sát và ramp gia tốc dương. Ramp áp dụng ngay cả khi xe đang đứng yên để tránh request đầu tiên nhảy bậc. Không import simulator/CARLA, không nhận actor ID hay ground truth. |
| [`vn_cutin_policy.py`](../../openpilot/selfdrive/controls/lib/vn_cutin_policy.py) | Xét ba giả thuyết lead như các giả thuyết, không phải ba actor. So độ lệch với đường đi ego, liên kết observation qua frame theo `probTime` và hình học, rồi xác nhận trajectory hoặc chuyển động ngang qua ba frame mới. Chỉ trả chướng ngại tương lai, không dùng CARLA. |
| [`long_mpc.py`](../../openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py) | Nhận `t_follow_override` và `cut_in_obstacle` tùy chọn. Giữ hai radar lead; thêm cut-in làm chướng ngại thứ ba qua tham số solver hiện có, không rebuild solver. |
| [`controlsd.py`](../../openpilot/selfdrive/controls/controlsd.py), [`longcontrol.py`](../../openpilot/selfdrive/controls/lib/longcontrol.py) | Đường điều khiển stock: `aTarget` qua LongControl để thành `carControl.actuators.accel`. Phase 2 không sửa hai file này. |
| [`common.py`](../../openpilot/tools/sim/bridge/common.py) | Bridge đổi gia tốc yêu cầu thành ga/phanh CARLA bằng vòng P/I và saturation hiện có; Phase 2 không đổi bridge. |
| [`analyze_vn_traffic.py`](../../openpilot/tools/sim/analyze_vn_traffic.py) | Đọc report JSONL xoay theo kiểu streaming, tính comfort/safety và so sánh một cặp mode 0/1. Không gửi tín hiệu ngược vào Openpilot. |

Model weights, artifact, `modeld`, `radard`, giới hạn phanh và bridge giữ nguyên.
Phase 2 bổ sung chướng ngại cut-in và tăng có điều kiện time-gap trước khi MPC
giải trajectory, rồi áp dụng ramp dương lên `aTarget` đã chọn. Mode 0 tiếp tục
dùng time-gap personality stock. Replay 774 chu kỳ mode 0 trên cùng input,
với cả planner **và MPC** lấy từ commit trước thay đổi `6a12538`, cho
trajectory, trạng thái solver và output điều khiển bằng nhau.

## Nhánh radar cut-in mô phỏng (2026-09-24)

Khi `VN_RADAR_CUTIN=1`, bridge CARLA gắn một radar trước xe (FOV ngang 100°,
dọc 10°, tầm 40 m, 3000 điểm/giây, tick 0,05 giây; mount x=0,4 m,
z=1,22 m). [`carla_radar.py`](../../openpilot/tools/sim/lib/carla_radar.py)
đổi các return sang hệ xe `+x` tiến, `+y` phải, `+z` lên và đóng gói có giới
hạn 200 điểm / 32 KiB. [`simulated_sensors.py`](../../openpilot/tools/sim/lib/simulated_sensors.py)
gửi qua `customReservedRawData0` chỉ trong simulator. Message chứa frame,
thời gian CARLA, thời gian lấy mẫu host và return; không chứa actor ID hay
tọa độ ground truth.

`plannerd` chỉ đăng ký service tùy chọn này khi `SIMULATION=1` và cờ radar
bật. [`vn_radar_cutin_policy.py`](../../openpilot/selfdrive/controls/lib/vn_radar_cutin_policy.py)
gom điểm, nối track qua frame và so chuyển động ngang với đường ego từ
`modelV2.position`. Bộ lọc yêu cầu chuyển động hướng vào hành lang, thời gian
đóng nguy hiểm và quan sát liên tục; trường hợp phản xạ nhiễu có cửa sổ xu
hướng dài hơn. Track mất, dữ liệu quá 250 ms, frame lặp, target nhập nhằng
hoặc mode không hợp lệ thì không tạo ràng buộc mới. Khoảng cách sử dụng trừ
3,2 m cho đầu xe và nửa chiều dài vật thể giả định; đây là giả định hình học,
không phải kết quả phân loại xe máy từ radar.

Chỉ khi `VN_TRAFFIC_MODE=1`, longitudinal active và experimental mode tắt,
planner hợp nhất chướng ngại radar với chướng ngại model bằng minimum theo
từng mốc MPC; hai stock lead vẫn còn. `VN_TRAFFIC_MODE=0` không dùng return
radar để quyết định điều khiển. Không đổi solver, model weights, artifact,
bridge hoặc lệnh phanh trực tiếp. Trace planner ghi bytes radar bằng base64,
trạng thái track và nguồn cut-in; report radar xoay file 10 MiB. Replay mode 0
trên 507 chu kỳ input có service radar xác nhận output planner/MPC bằng bản
stock trước thay đổi.

Hai lượt alternating 30 giây seed 42 trong CARLA có giao diện đã hoàn tất
không contact; gia tốc yêu cầu tối thiểu lần lượt −3,30/−2,97 m/s² và gia tốc
đo được −3,58/−3,48 m/s². Đây **chưa phải nghiệm thu**: sai số tracking ngang
xe máy P95 0,757 m > ngưỡng 0,4 m, và chưa có clip đồng bộ. Các report trước
khi sửa trường `bumper_gap_m` còn thiếu chứng cứ pose 10 Hz; không đổi nhãn
chúng thành ca đạt. Xem [báo cáo sơ bộ](../reports/2026-09-24-vn-radar-cutin-preliminary.md).

## Dự đoán cut-in và giới hạn đã thấy

Predictor xét 40 m phía trước, 2 giây tương lai, hành lang xung đột 1,08 m
nửa bề rộng Tesla + 0,5 m nửa bề rộng giả định của xe máy + 0,2 m biên.
Xác suất tối thiểu 0,55; phải tiến gần đường ego và được xác nhận trên ba
frame mới trong ít nhất 0,1 giây. Predictor dùng trajectory model khi nó dự
đoán vào hành lang; khi trajectory thưa hoặc không ổn định, nó dùng chuyển
động ngang nhất quán của cùng nguồn `probTime` qua các frame. Một hypothesis
hỏng không làm mất hypothesis hợp lệ khác. Các hypothesis gần nhau cùng phía
được xem là những khả năng của model, không phải nhiều actor độc lập.

Dữ liệu quá 250 ms, timestamp không tiến, thiếu message hoặc NaN hủy nguy cơ.
Observation có `xStd > 75% x` được phép giữ continuity nhưng chưa được tác
động MPC. Frame cuối xác nhận phải đạt gate dọc này và `yStd <= 1,8 m`; đường
dự đoán từ một frame vẫn dùng gate ngang 1,5 m. Sau ba frame, `xStd` tạo biên
bảo thủ tối đa 50% khoảng cách. Một track đã xác nhận được rút trong 0,3 giây
khi observation tạm thời không chắc chắn và có thể reacquire; stale hoặc mất
message vẫn bỏ ngay. Trước MPC, khoảng cách model được đổi sang
cùng gốc với radar bằng cách trừ 1,52 m như `radard`; dự đoán đã ở sau ego
trở thành chướng ngại vô hạn, không truyền khoảng cách âm cho solver.
`lead2` biểu thị nguồn chướng ngại cut-in khi nó có biên an toàn MPC thấp
nhất dọc trajectory, không phải chứng minh nhân quả của lần phanh; trace ghi
slot giả thuyết, xác suất, `probTime`, bên xe, độ bất định, thời gian
nhập làn và khoảng cách dùng trong MPC mỗi chu kỳ. `hasLead` vẫn chỉ phản ánh
radar leadOne, không được dùng làm chứng cứ xác định actor cut-in.

Ca CARLA có giao diện seed 42 sau sửa tracking vẫn contact. Predictor live
không xác nhận cut-in; yêu cầu giảm tốc bắt đầu 0,171 giây trước entry. Replay
một trace cũ bằng predictor mới xác nhận chuyển động ngang 0,316 giây trước
lane entry, nhưng MPC không chọn `lead2`; yêu cầu giảm tốc vẫn bắt đầu khoảng
0,153 giây trước entry và chỉ mạnh hơn ở frame đó. Model ước lượng khoảng cách
khoảng 16–18 m trong khi ground truth gần hơn nhiều. Kết quả vẫn là **chưa
đạt** cho tránh va chạm. Predictor không dùng ground truth để dựng lead giả.
Chi tiết ở
[báo cáo cut-in](../reports/2026-09-23-vn-cutin-preliminary.md).

## Điều kiện bật và thuật toán

Planner đọc `VN_TRAFFIC_MODE=0|1` và
`VN_TRAFFIC_PROFILE=balanced|gentle|responsive` **một lần khi khởi tạo**;
giá trị không hợp lệ gây lỗi khởi động. Mặc định là `0` và `balanced`.
[`launch_openpilot.sh`](../../openpilot/tools/sim/launch_openpilot.sh) đặt
`SIMULATION=1`. Khi mode bật, policy chỉ có hiệu lực nếu Openpilot longitudinal
đang active và experimental mode tắt. Ramp gia tốc dương hoạt động từ
standstill và reset khi `shouldStop`, disengage hoặc đầu vào không hợp lệ;
predictor cắt nguy cơ ngay
khi model không còn mới hoặc hợp lệ. Trên môi trường không phải mô phỏng,
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
- Đã ghi/replay đầy đủ message đầu vào planner và chứng minh parity mode 0
  trên một trace 774 chu kỳ; vẫn cần nhiều seed và ảnh/video đồng bộ.
- Đã có ca đối chứng đường trống/xe chỉ đi cạnh và lead đều, nhưng chưa chạy đủ các seed
  tuning/validation hoặc chọn profile theo tổng hợp. Một cặp seed 42 không đủ
  để nghiệm thu hay tuyên bố an toàn.
- Mode phải giữ mặc định tắt. Không dùng bản này trên xe thực tế; nếu tiếp tục,
  đóng gate Phase 1 rồi chạy lại A/B có giao diện theo kế hoạch đã chốt.
