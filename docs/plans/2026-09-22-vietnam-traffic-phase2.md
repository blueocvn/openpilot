# Phase 2 — Làm mượt hành vi Openpilot trong dòng xe máy

## Mục tiêu và các quyết định đã chốt

Ưu tiên giảm dao động tăng/giảm tốc và stop-and-go không cần thiết. Giữ nguyên trọng số neural model và pipeline nhận diện. Openpilot tự quyết định điều khiển từ tín hiệu nó quan sát được tại thời điểm chạy.

User đã chốt giữ model, ưu tiên smooth, và không được cho ego biết trước tình huống. Câu hỏi về rút ngắn khoảng bám bị ngắt khi cúp điện: mặc định **giữ nguyên khoảng bám của personality hiện tại** ở bản đầu. Không đổi ngưỡng nhận lead, khoảng cách dừng, giới hạn phanh hoặc safety code.

Phạm vi triển khai đầu tiên là planner trong mô phỏng WSL có giao diện. Giữ nguyên map/scene đã nghiệm thu, cruise 30 km/h, experimental mode tắt. Không có triển khai lên xe thật trong kế hoạch này.

## 1. Hoàn tất và khóa nghiệm thu Phase 1

Thực hiện các hạng mục còn thiếu trong [biên bản nghiệm thu](../reports/2026-09-22-vietnam-phase1-acceptance.md). Hiện source/ONNX giữ nguyên so với fork HEAD nhưng baseline còn sai số quỹ đạo lớn, thiếu provenance runtime và thiếu ghép lead–actor.

- Sửa controller xe máy trong simulator bằng phản hồi vị trí/tốc độ, bỏ cưỡng chế vận tốc mỗi tick và bỏ phase lookahead theo thời gian gây nhập/ra làn sớm. Tính tiến độ trên quỹ đạo từ vị trí actor; bắt đầu bộ đếm hold khi actor thật sự ở hành lang làn ego. Giữ vị trí/tốc độ/gap đã chốt và tiêu chí tracking hiện có.
- Bổ sung observer và manifest mà không sửa model, radard, planner hoặc controller ego. Ghi ground truth 2 Hz, trạng thái điều khiển 10 Hz, sự kiện chuyển làn/contact theo tick. Thêm ghi các message đầu vào planner bằng định dạng log hiện có, xoay 10 MiB; buffer giới hạn và đánh dấu run không hợp lệ nếu mất dữ liệu.
- Kiểm tra bridge từ `aTarget` → `carControl.actuators.accel` → P/I và saturation → throttle/brake → gia tốc thực tế. Nếu tìm thấy lỗi bridge, sửa trong Phase 1 và tạo lại cả baseline; không dùng sửa bridge làm bằng chứng Phase 2 tốt hơn.
- Run mới phải xác nhận actual lane hold/gap, tracking và đồng bộ thời gian; clip có giao diện giúp kiểm chứng hình học. Va chạm vẫn là kết quả được ghi nhận, không phải lý do ép phanh hoặc xóa ca xấu.
- Khóa baseline bằng snapshot riêng của đúng các file Phase 1 và manifest. Giữ nhánh Phase 1; tạo `codex/vietnam-traffic-phase2` từ snapshot đã chốt, làm trong checkout WSL hiện tại. Không gom thay đổi ngoài phạm vi vào snapshot.

Đây là gate bắt buộc: chưa có baseline hợp lệ thì không bắt đầu chỉnh hành vi hoặc tuyên bố cải thiện.

## 2. Tham khảo FrogPilot và giới hạn áp dụng

Đã kiểm tra release branch `FrogPilot`, commit `1e23dec6352cef5a36a87be0af7d7a082b7c48a4`.
[FrogPilotFollowing](https://github.com/FrogAi/FrogPilot/blob/1e23dec6352cef5a36a87be0af7d7a082b7c48a4/frogpilot/controls/lib/frogpilot_following.py) dùng profile Traffic Mode với các hệ số acceleration/danger/speed jerk 0,5 và thời gian bám nội suy 0,5–1,0 giây, đạt 1 giây ở 30 km/h. Các hệ số này là trọng số tối ưu, không phải giới hạn jerk vật lý; giảm trọng số không tự đảm bảo mượt hơn.

Áp dụng ý tưởng một profile low-speed có thể bật/tắt và đánh giá độc lập. Không cherry-pick toàn bộ FrogPilot: planner/API hai fork khác nhau. Bản đầu giữ `t_follow`, MPC braking và perception như baseline, tập trung làm mượt quá trình tăng tốc trở lại sau khi lead đổi hoặc rời làn.

## 3. Hành vi và interface Phase 2 v1

Thêm `VN_TRAFFIC_MODE=0|1`, đọc một lần khi planner khởi động. Mặc định 0; mode chỉ có hiệu lực khi `SIMULATION=1`, experimental mode tắt và longitudinal active. Chọn profile thử bằng `VN_TRAFFIC_PROFILE=balanced|gentle|responsive`; mặc định `balanced`. Giá trị không hợp lệ phải báo lỗi lúc khởi động. Ghi mode/profile vào manifest.

Thêm một policy nhỏ trong tầng longitudinal planner, đầu vào chỉ gồm output hiện tại của planner, ego speed, active/stop state và trạng thái ramp từ các tick trước. Áp dụng ngay trước khi lưu output và tích phân trạng thái mong muốn, để output và trạng thái nội bộ nhất quán.

| Profile | Trần gia tốc dương | Tốc độ tăng gia tốc dương |
| --- | --- | --- |
| balanced | 1,0 m/s² | 0,8 m/s³ |
| gentle | 0,8 m/s² | 0,6 m/s³ |
| responsive | 1,2 m/s² | 1,0 m/s³ |

- Với yêu cầu gia tốc dương, giới hạn bằng giá trị nhỏ nhất giữa yêu cầu gốc, trần profile và `max(previous_output, 0) + jerk_limit * dt`. Policy chỉ giảm một yêu cầu tăng tốc đã có.
- Với yêu cầu gia tốc bằng hoặc nhỏ hơn 0, truyền nguyên giá trị gốc và reset ramp dương. Không trì hoãn yêu cầu giảm tốc, tạo yêu cầu phanh mới hoặc thay `shouldStop`.
- Áp dụng đầy đủ dưới 40 km/h; nội suy output về output gốc từ 40 đến 50 km/h, và hoàn toàn stock từ 50 km/h. Reset trạng thái khi disengage hoặc mode không active. Mode tắt phải cho cùng output như baseline với cùng input trace.
- Giữ nguyên lựa chọn lead, t_follow, MPC obstacle/danger cost và đường chuyển gia tốc sang actuator. Đây là phạm vi có chủ đích của v1; nếu rung giật chủ yếu do lead/perception hoặc deceleration, kết quả có thể không đạt và phải được báo rõ.

Planner không được import simulator/CARLA hoặc đọc seed, scene time, event ID, actor ID, gap ground truth hay lịch spawn. Dự đoán chuyển động từ các lead đang quan sát trong MPC vẫn là hành vi bình thường của Openpilot. Ground truth chỉ đi vào recorder/analyzer, không publish ngược vào `modelV2` hoặc `radarState`.

## 4. Kiểm thử và phép so sánh

**Unit/replay:** mode off parity; tăng tốc có ramp đúng với dt; deceleration truyền nguyên; reset ở disengage/standstill; chuyển miền tốc độ; input stale/invalid giữ quy tắc baseline; không tăng positive acceleration so với output gốc. Dùng cùng chuỗi message đã ghi cho baseline và candidate để kiểm tra thuật toán.

**Kiểm tra không biết trước:** hai trace có cùng tiền tố quan sát nhưng khác phần tương lai phải cho output bằng nhau trong tiền tố. Thay seed/actor IDs/ground truth trong evaluator không được làm thay đổi output planner khi các message Openpilot đầu vào vẫn giống nhau. Thêm kiểm tra dependency để module policy không phụ thuộc `tools.sim`/CARLA.

**CARLA có giao diện:** dùng ba nhóm ca: đường trống/xe máy đi cạnh mà không nhập làn; bám một lead chạy đều rồi chậm–nhanh; scene đông xe tạt đầu đã khóa. Ego và actor tiếp tục đi trong mỗi episode, không reset giữa các lượt; mỗi episode độc lập khởi động cùng map/spawn. Giữ nguyên command, camera, model, Params và bridge giữa A/B, chỉ đổi mode/profile.

- Tuning: seed 42, 43, 44; mỗi seed chạy baseline và cả ba profile, 60 giây sau khi ready. Chọn profile đạt các ràng buộc dưới đây với P95 jerk thấp nhất; nếu bằng nhau, ưu tiên tiến độ hành trình cao hơn.
- Xác nhận độc lập: seed 45–49, baseline và profile đã chọn, hai lần mỗi seed, 60 giây. So sánh theo seed và loại tình huống; seed giống nhau không đảm bảo timestamp hoặc quỹ đạo traffic giống nhau khi ego đã chạy khác.
- Chế độ manual `duration=0` vẫn có sẵn. Các episode finite phục vụ so sánh, chạy tuần tự có giao diện trong WSL. Lưu clip kiểm chứng ít nhất một ca mỗi nhóm, không cần quay liên tục toàn bộ phiên dài.

Ghi requested/actual acceleration, jerk ở 10 Hz, số lần đổi giữa tăng tốc và giảm tốc, thời gian dừng/khởi hành, tốc độ trung bình và quãng đường; đồng thời ghi min gap, TTC theo chuyển động đang quan sát, contact và disengagement. Quy ước đổi trạng thái dùng ngưỡng ±0,2 m/s² và duy trì 0,3 giây để bỏ nhiễu. Tính jerk từ hiệu gia tốc và timestamp thực tế; tách requested/actual. Các đoạn sau contact không được chấm comfort; luôn báo số lượng và thời lượng bị loại.

**Nghiệm thu Phase 2:** trên bộ xác nhận, P95 absolute actual jerk giảm ít nhất 15%, số lần đổi tăng/giảm tốc không tăng, quãng đường trung bình đạt ít nhất 95% baseline. Contact/disengagement không tăng theo cặp seed; ở ca không contact, min TTC không giảm quá 0,2 giây và min gap không giảm quá 0,25 m. Báo cả từng run và tổng hợp, không chỉ một run đẹp. Nếu baseline vốn có contact, giữ nguyên số contact không được gọi là đã giải quyết an toàn.

Nếu không profile nào đạt, giữ mặc định off và kết luận v1 chưa cải thiện đủ. Không đổi model, cài lịch tình huống vào planner hoặc tự mở rộng sang lead filtering để ép đạt chỉ tiêu. Báo cáo phải chỉ rõ nguyên nhân giới hạn và đề xuất một thí nghiệm tiếp theo có phạm vi riêng.

## 5. Sản phẩm bàn giao

- Biên bản Phase 1 được cập nhật thành pass sau khi đóng các gate, kèm manifest và report baseline mới.
- Policy Phase 2 có opt-in, unit/replay tests và công cụ chạy/chấm A/B streaming; model hashes vẫn khớp baseline.
- Bảng số liệu từng seed, profile được chọn hoặc kết luận no-go, clip đối chiếu và lệnh chạy tái lập. Đánh giá phản ứng tự nhiên của Openpilot; không dùng tỷ lệ brake > 0 làm mục tiêu tối ưu.

Chưa có code Phase 2 được triển khai trong lượt lập kế hoạch này.
