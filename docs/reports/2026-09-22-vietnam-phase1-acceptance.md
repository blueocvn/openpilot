# Nghiệm thu Phase 1 — Vietnam motorcycle scene

Ngày kiểm tra: 2026-09-22. Repo: `blueocvn/openpilot`.
Nhánh: `codex/vietnam-motorcycle-phase1`.
Mốc đối chiếu: `61363a54bad2bcb95162cbeb6b9bcf2131b3c331`.

## Kết luận

**Đạt kiểm tra tính nguyên vẹn của source và trọng số driving model so với mốc fork trên. Chưa nghiệm thu đầy đủ baseline thực nghiệm để chấm cải thiện Phase 2.**

Phase 1 quan sát hành vi hiện tại của Openpilot trong mô phỏng. Một kết quả có va chạm hoặc không tác động brake vẫn có thể là baseline hợp lệ; điều kiện là tình huống, chuỗi điều khiển và phép đo phải được xác minh. Không sửa planner hoặc ép brake để đạt nghiệm thu.

## Bằng chứng tính nguyên vẹn

- `git diff HEAD` không có thay đổi trong `openpilot/selfdrive/modeld`, `openpilot/selfdrive/controls` và `openpilot/selfdrive/car`.
- SHA-256 của hai file ONNX thực tế khớp chính xác với OID Git LFS tại HEAD:

| File | SHA-256 |
| --- | --- |
| `driving_supercombo.onnx` | `659727c4d4839adc4992a254409a54259a8756a743f2d567bf5fdc6579f8009b` |
| `big_driving_supercombo.onnx` | `a501760a9d1d5fef0eab2b8c5d122d06124fc26dc8e0782e0aa94b82a208f0ff` |

- Thay đổi source trong `selfdrive` đang có là cách hiển thị camera UI. Các thay đổi phục vụ tình huống nằm trong simulator, scene, bridge và telemetry.
- Bridge có vòng điều khiển chuyển gia tốc Openpilot yêu cầu thành ga/phanh CARLA. Vòng này đã có trong commit nền, không được thêm hoặc chỉnh để ép phanh trong scene mới. Tuy vậy đây vẫn là mô phỏng actuator, không phải đường CAN/actuator của xe thật.
- Đây là đối chiếu với **fork blueocvn**, không phải chứng nhận toàn bộ fork giống hệt một bản upstream comma.ai.
- `modeld` chạy artifact Tinygrad đã biên dịch. Report cũ không lưu hash/provenance artifact đang chạy, nên chưa thể xác nhận hồi tố toàn bộ runtime chỉ từ hash ONNX. Manifest artifact và nguồn biên dịch là hạng mục còn thiếu.
- Workspace có thay đổi khác, bao gồm file AGNOS bị xóa. Không đưa các thay đổi ngoài phạm vi vào baseline hoặc khôi phục chúng khi chưa xác định chủ sở hữu.

## Kết quả chạy hiện có

Report: `.carla/reports/motorcycle-weave-20260922-022007-533860003/`.
Lệnh dùng WSL, có giao diện, Town04_Opt/spawn 40, seed 42, cruise 30 km/h, experimental mode tắt.

| Chỉ số | Kết quả |
| --- | --- |
| Thời gian | 82 giây |
| Ground truth / control samples | 165 / 822 |
| `longActive` | 822/822 control samples |
| Tốc độ ego lớn nhất | 8,362 m/s, khoảng 30,1 km/h |
| Cut-in hoàn tất | 12 |
| Mẫu có `hasLead` | 243/822 |
| Gia tốc yêu cầu thấp nhất | −1,662 m/s² |
| Brake CARLA lớn nhất | 0 |
| Actor khác nhau có contact với ego | 7 |
| Sai số ngang P95 / lớn nhất | 2,023 m / 2,774 m |
| `actor_tracking_valid` | false |

Con số 12 lượt có lead nghĩa là có ít nhất một mẫu `hasLead` trong từng cửa sổ sự kiện. Analyzer hiện không ghép lead với actor, nên chưa chứng minh nhận đúng xe máy tạt đầu. `collision_count=7` hiện đếm actor ID khác nhau, không phải số va chạm độc lập có timestamp đầy đủ.

Đoạn trên mô tả **report cũ**. Analyzer mới đã có phép ghép radar lead–actor bằng hình học ở ground truth 2 Hz, trả về không biết khi mơ hồ; nó không sửa dữ liệu đưa vào planner.

Gia tốc âm kèm brake=0 phù hợp với khả năng giảm tốc bằng nhả ga, nhưng log chưa ghi đủ các hạng P/I của bridge để chứng minh đây là nguyên nhân duy nhất. Contact cũng có thể làm ego giảm tốc. Không kết luận lỗi thuộc model hoặc planner từ các chỉ số tổng hợp này.

## Run chẩn đoán sau sửa simulator — chưa khóa baseline

Report: `.carla/reports/motorcycle-weave-20260922-040434-954638640/`; WSL có giao diện, Town04_Opt/spawn 40, seed 42, 30 giây. Đây là run chẩn đoán khi sửa scene chưa commit; hash file thực tế nằm trong manifest, còn `git_commit` chỉ là mốc HEAD lúc chạy. Source planner/model/weights vẫn không đổi. Xe tạt đầu lấy từ dòng xe máy đã chạy ổn định, không teleport ra trước ego và không gọi `set_target_velocity` mỗi tick. Manifest ghi hash source, ONNX và artifact Tinygrad, nhưng `compiled_source_link_verified=false`; chưa chứng minh artifact được biên dịch từ đúng ONNX đó.

| Phép đo | Kết quả |
| --- | --- |
| Cut-in hoàn tất / contact | 4 / 0 |
| Khoảng hở khi thực sự vào làn | 6,03–7,93 m |
| Thời gian thực ở làn ego | 1,45–1,75 giây |
| Sai số tốc độ xe máy P95 | 0,429 m/s, đạt |
| Sai số ngang P95 / lớn nhất | 0,673 / 0,934 m, **không đạt** 0,4 / 0,8 m |
| Cut-in có `hasLead` bất kỳ / ghép đúng actor | 4 / 0 |
| Brake CARLA lớn nhất | 0,048; không đạt ngưỡng ghi nhận >0,05 |

Run này cho thấy Openpilot giảm tốc ở một số lượt nhưng không chứng minh radar/planner nhận đúng chiếc xe máy tạt đầu, và không có bằng chứng đủ để kết luận nó phanh vì xe đó. `hasLead` có thể là xe máy khác trong dòng. Không dùng run này để chấm Phase 2. Những run trung gian khác gồm một run 60 giây với 6 lượt không contact nhưng sai số ngang P95 1,227 m, và hai run chẩn đoán có contact; không chọn riêng run đẹp để tuyên bố nghiệm thu.

Đã đóng một phần các hạng mục đo: sự kiện contact có frame/timestamp/impulse, radar/model lead cùng trạng thái message, các hạng P/I/saturation của bridge, actual gap/hold, và manifest. Còn thiếu: tracking đạt gate trên nhiều seed, provenance biên dịch artifact, ghi/đọc đủ input planner để replay, clip đồng bộ do người dùng capture, và bộ A/B lặp lại. Bộ kiểm tra tập trung hiện 64 pass, 1 skip (sau thay đổi scene); kiểm tra code style đạt. Phase 2 theo kế hoạch vẫn bị gate Phase 1 chặn.

## Các điểm phải đóng trước khi chốt baseline

1. **Quỹ đạo:** đạt tiêu chí đang có: P95 sai số ngang ≤0,4 m, lớn nhất ≤0,8 m trước contact; P95 sai số tốc độ ≤1 m/s. Đo lúc nhập làn, gap và thời gian ở làn thực tế, không lấy lịch dự kiến làm ground truth. Không nới ngưỡng chỉ để đánh dấu pass.
2. **Động lực học:** kiểm tra việc gọi `set_target_velocity` mỗi tick; nó cưỡng chế vận tốc xe máy, đặc biệt sau contact. Dùng điều khiển actor có phản hồi, khởi tạo vận tốc một lần lúc spawn; kết thúc chấm comfort ở contact đầu tiên của episode. Mọi sửa mô phỏng phải dùng chung cho baseline và Phase 2.
3. **Quan sát:** ghi `radarState.leadOne/leadTwo`, raw model leads, timestamp/frame, trạng thái hợp lệ, các hạng điều khiển bridge và ga/phanh thực tế. Ghép lead–actor chỉ trong bộ phân tích; trường hợp không chắc phải ghi `unknown`.
4. **Contact:** ghi sự kiện trực tiếp với simulation timestamp, frame, actor ID và impulse; tách contact đầu tiên, contact lặp và trường hợp actor nền.
5. **Tái lập:** manifest chứa commit, diff/hash scene/bridge, ONNX, compiled artifact, checkpoint model, Params, personality, camera/calibration, CARLA version, map, seed và command. Xác minh nguồn artifact; nếu thiếu provenance, biên dịch lại từ ONNX đã xác minh rồi tạo report baseline mới.
6. **Kiểm tra:** chạy lại scene có giao diện, lưu clip đồng bộ ít nhất một lượt mỗi hướng và đối chiếu log; chạy đủ test simulator đúng runner. Bộ kiểm tra tập trung sau các sửa scene đạt 64 test, 1 skip; Ruff đạt. Bộ test rộng hơn còn lỗi collection ở `test_carla_bridge.py` vì thiếu pytest và cần dùng pytest cho các test dạng function.

Baseline chỉ được khóa sau khi các điểm trên có bằng chứng. Kết quả collision/brake được giữ nguyên trung thực; không coi phanh mạnh hoặc zero collision là điều kiện để chứng nhận rằng Phase 1 đã quan sát đúng hệ thống.
