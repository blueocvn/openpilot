# Báo cáo: xe không giữ khoảng cách và không dừng trước vật cản (sim CARLA, chỉ camera)

Ngày đo: 2026-11-12. Nhánh `bukapilot-motorcycle`. Kịch bản: Town04_Opt, spawn point 170, xe máy trong cùng làn.

## Kết luận ngắn

- Xe không phanh vì planner nhận tốc độ xe phía trước (`vLead`) sai: model gần như luôn báo xe trước đang chạy xấp xỉ tốc độ xe mình, kể cả khi nó đứng yên.
- Tầm model thấy rõ xe máy chỉ 10 đến 20 m và co lại khi áp sát nhanh, nên ở tốc độ cao không đủ quãng đường để dừng.
- Khoảng cách model báo thường xa hơn thật, làm xe dừng sát hoặc va chạm.
- Vị trí camera và độ trễ không phải nguyên nhân.
- Chưa chứng minh được vì sao model cho số sai (xem mục "Chưa lý giải được").

## Số liệu quét tốc độ áp sát

Xe mình chạy 8 m/s, xe máy trong làn, khoảng cách thật giảm dần.

| Tốc độ áp sát | Tốc độ thật của xe máy (m/s) | Model báo tốc độ xe máy (m/s) | Còn thấy rõ (prob > 0,5) tới | Sai khoảng cách ở 10 m thật |
|---|---|---|---|---|
| 1 m/s | 7,0 | 8,1 | 16 m | +2 m |
| 3 m/s | 5,0 | 7,1 | 14 m | +4 m |
| 5 m/s | 3,0 | 7,5 | 10 m | +3 m |
| 6,5 m/s | 1,5 | 6,8 | 10 m | +22 m |
| 8 m/s (đứng yên) | 0 | 7,1 | 10 m | +36 m |

Hai lần đối chứng:

| Tình huống | Kết quả |
|---|---|
| Xe mình 3 m/s vào xe máy đứng yên | Thấy rõ tới 26 m, sai +2 đến +4 m, model báo tốc độ xe máy 1,2 m/s |
| Xe mình 15 m/s, áp sát 3 m/s | Thấy rõ tới 20 m, sai +6 đến +9 m |

Ghi chú về độ tin cậy của `x`: `leadsV3.xStd` là 1 đến 4 m khi khoảng cách đúng và 25 đến 60 m khi khoảng cách sai, nên dùng được làm thước đo tin cậy.

## Số liệu khi xe đỗ (cả hai đứng yên)

| Khoảng cách thật (m) | 5 | 9 | 12 | 15 | 20 | 25 | 30 |
|---|---|---|---|---|---|---|---|
| Xe máy: model báo (m) | 8,4 | 9,1 | 8,2 | 8,8 | 13,2 | 29 (prob 0,45) | 70 (prob 0,04) |
| Ô tô: model báo (m) | 5,0 | 8,0 | 8,2 | 10,5 | 14,2 | 17,4 | 40 (prob 0,42) |

Khi xe đỗ model báo gần hơn thật, khi xe chạy model báo xa hơn thật. Vì hai chiều sai ngược nhau, một hệ số hiệu chỉnh cố định không sửa được.

## Luồng tính khoảng cách

| Bước | Nơi xử lý | Nội dung |
|---|---|---|
| 1 | `tools/sim/bridge/carla_world.py`, `tools/sim/lib/camerad.py` | CARLA render 20 Hz, đổi BGRA sang I420, gửi qua VisionIPC |
| 2 | `selfdrive/modeld/modeld.cc` | Model xuất `modelV2.leadsV3` gồm `x`, `v`, `prob` |
| 3 | `selfdrive/controls/radard.py` (`get_lead`) | `dRel = x − 1.52`, `vLead = v`, chỉ nhận khi `prob > 0.5` |
| 4 | `selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py` | MPC tính gia tốc, muốn dừng cách 6,4 m |
| 5 | `controlsd`, bridge | Gia tốc đổi thành ga/phanh gửi CARLA |

## Đã lý giải được

- Code này được thiết kế để lấy khoảng cách và tốc độ xe trước từ radar. Xe trong sim là Honda Civic có radar, nhưng sim không có điểm radar nào, nên `radard` rơi về nhánh dự phòng dùng thẳng số của model.
- Nhánh dự phòng đưa `vLead` của model vào planner mà không kiểm tra. Planner phanh chủ yếu theo chênh lệch tốc độ, nên khi `vLead` xấp xỉ tốc độ xe mình thì không phanh.
- Quỹ đạo model dự đoán (`position`), `hardBrakePredicted` và các xác suất phanh không đổi khi có vật chắn, nên không có tầng nào khác nhận ra vật cản.

## Đã loại trừ

| Giả thuyết | Kết quả đo |
|---|---|
| Camera đặt sai hoặc zoom | Camera cách tâm xe 0,80 m, cao 1,13 m, FOV 40°, tiêu cự 2648,6 px (modeld dùng 2648). Hàng pixel chạm đất lệch không quá 2 px ở 6 đến 10 m. |
| Độ trễ | Khung hình tới `modelV2` mất 17 ms. Phản ứng bậc thang 0,20 đến 0,35 s. |
| Nhịp khung hình | 20 khung/giây, thời gian sim so với thực là 1,00. |
| Thời tiết | Đổi thời tiết không cải thiện, ClearNoon tốt nhất. |

Phần lệch nhỏ còn lại: `calibrationd` giả định camera cao 1,22 m, sim là 1,13 m (khoảng 8%).

## Chưa lý giải được: vì sao model cho số sai

| Nguyên nhân khả dĩ | Cách kiểm tra | Nếu đúng thì sửa ở đâu |
|---|---|---|
| Ảnh CARLA khác ảnh thật mà model được huấn luyện | Phát lại log xe thật có xe dừng phía trước qua cùng `modeld` | Sim |
| Phép đổi BGRA sang YUV sai nhẹ về số (mới xem bằng mắt) | So một khung với phép đổi chuẩn | Sim |
| Model vốn yếu về tốc độ xe trước (suy đoán, chưa có nguồn) | Hai phép trên đều không tìm ra lỗi | `radard` hoặc planner |

Điểm chưa khớp: xe máy đặt cố định một lần cho `prob = 0` ở 10 m, còn đặt lại mỗi tick cho `prob = 0,60`. Chưa rõ lý do.

## Kết luận đã rút lại

- "Chỉ cần có chuyển động tương đối là model bỏ qua vật": sai. Áp sát chậm vẫn được thấy.
- Bảng đo đầu tiên của thí nghiệm xe chạy vào xe máy đứng yên: vị trí xe máy được đọc trước khi CARLA cập nhật, nên không dùng. Lần đo lại với vị trí đã kiểm tra cho cùng kết luận ở 8 m/s.

## Các hướng thử

| Hướng | Nội dung | Giới hạn |
|---|---|---|
| 1. Không tin `vLead` của model | Tự tính tốc độ áp sát từ đạo hàm `dRel` khi `xStd` nhỏ, đưa planner giá trị bi quan hơn | Không giúp khi model không ra lead. Đạo hàm nhiễu, có thể phanh ảo. |
| 2. Trần tốc độ theo tầm thấy | Khi không có lead tin cậy, giới hạn tốc độ để dừng kịp trong khoảng 15 m (khoảng 30 km/h) | Xe chậm hẳn trên đường trống. |
| 3. Nguồn nhận biết độc lập | Thời gian đến va chạm từ độ phình ảnh, hoặc detector riêng cộng hình học camera | Module mới, rủi ro phanh ảo hoặc giới hạn phần cứng. |
| 4. Kiểm tra trên log xe thật | Xác định lỗi do sim hay do model | Cần log thật. |

Ràng buộc: chỉ dùng camera, sim và xe thật chạy cùng code. Hướng 1 và 2 đổi code dùng chung nên cần duyệt trước.

## Việc tiếp theo đề xuất

1. So số phép đổi BGRA sang YUV (khoảng 10 phút).
2. Phát lại log xe thật.
3. Thử offline đạo hàm `dRel` trên số liệu từng khung để xem hướng 1 có đủ sạch không.

## Câu hỏi còn mở

- Có log lái thật (rlog và video) với xe dừng hoặc xe máy chậm phía trước không?
- Xe thật chạy trên phần cứng nào (quyết định hướng 3)?
- Có chấp nhận trần tốc độ khoảng 30 km/h khi không có lead tin cậy không?
