# Báo cáo: xe không giữ khoảng cách và không dừng trước vật cản (sim CARLA, chỉ camera)

Ngày đo: 2026-11-12. Nhánh `bukapilot-motorcycle`. Kịch bản: Town04_Opt, spawn point 170, xe máy trong cùng làn.

## Kết luận ngắn

- Kết quả phụ thuộc mạnh vào cảnh. Ở Town05 (đường phố) model xử lý vật đứng yên tốt hơn nhiều so với Town04 (cao tốc). Các kết luận đo ở Town04 không được coi là đúng chung.
- Lỗi chung cho cả hai cảnh: khoảng cách model báo xa hơn thật 5 đến 7 m ở cự ly gần, trong khi planner muốn dừng ở 6,4 m. Đây là nguyên nhân bám sát và va chạm nhẹ.
- Tầm thấy xe máy và rào chắn chỉ 25 đến 45 m, không đủ để dừng ở 60 km/h.
- Vật nhỏ không giống xe (thùng container 2 x 1 x 1,7 m) không bao giờ thành lead, nên xe đâm thẳng vào.
- Ở Town04, model còn báo xe máy đứng yên đang chạy khoảng 7 m/s và chỉ thấy từ 10 m. Ở Town05 cùng tình huống cho 2,4 m/s và thấy từ 30 m. Chưa rõ vì sao Town04 tệ hơn.
- Vị trí camera, độ trễ và cách đặt vật khi đo không phải nguyên nhân.

## Số liệu Town05: vật cố định trong làn (spawn point 1, đường thẳng, phẳng)

Xe mình chạy thẳng tới vật đứng yên, openpilot không engage.

| Vật cản | Xe mình 8 m/s: thấy rõ (prob > 0,5) từ | Sai khoảng cách khi còn 10 m | Model báo tốc độ vật (thật = 0) | Xe mình 3 m/s: thấy rõ từ |
|---|---|---|---|---|
| Ô tô đỗ, nhìn từ sau | hơn 100 m | +5 m | 0,5 đến 2,4 m/s | khoảng 70 m |
| Ô tô nằm ngang làn | hơn 100 m | +5 m | khoảng 2 m/s | 30 m, chập chờn |
| Xe máy | 30 đến 45 m | +7 m | 2,4 m/s | khoảng 25 m |
| Rào chắn đường | khoảng 25 m | +7 m | 1,3 m/s | khoảng 25 m |
| Thùng container nhỏ | không thấy (prob tối đa 0,28) | không có lead | không có lead | không thấy |

Chi tiết xe máy đứng yên, xe mình 8 m/s:

| Khoảng cách thật (m) | 10 | 14 | 19 | 26 | 37 | 57 |
|---|---|---|---|---|---|---|
| Model báo (m) | 17,9 | 20,6 | 31,8 | 39,9 | 64,4 | 97,5 |
| prob | 0,97 | 0,94 | 0,90 | 0,81 | 0,50 | 0,17 |
| xStd (m) | 3,4 | 4,1 | 12,3 | 16,8 | 26,5 | 25,4 |

Kiểm tra phương pháp đo: bốn cách đặt xe máy (vật lý bật, vật lý tắt giữ nguyên tư thế, vật lý tắt đặt theo độ cao xe mình, dịch chuyển lại mỗi tick) cho số gần như giống nhau ở Town05. Vậy khác biệt giữa Town04 và Town05 không do cách đặt vật.

## Số liệu Town04: quét tốc độ áp sát (spawn point 170, cao tốc)

Xe mình chạy 8 m/s, xe máy trong làn, khoảng cách thật giảm dần. Số liệu này chỉ đúng cho Town04.

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

## Số liệu Town04 khi xe đỗ (cả hai đứng yên)

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
- Nhánh dự phòng đưa `dRel` và `vLead` của model vào planner mà không kiểm tra. Khoảng cách dư 5 đến 7 m làm xe dừng sát vật. Ở Town04 `vLead` còn xấp xỉ tốc độ xe mình nên planner không phanh.
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

Chưa rõ vì sao Town04 tệ hơn Town05 (cảnh cao tốc, độ dốc mặt đường, hoặc ánh sáng). Chưa đo Town04 với vật spawn bình thường trên mặt đường.

Điểm chưa khớp trong Town04: xe máy đặt cố định một lần cho `prob = 0` ở 10 m, còn đặt lại mỗi tick cho `prob = 0,60`. Chưa rõ lý do.

## Kết luận đã rút lại

- "Chỉ cần có chuyển động tương đối là model bỏ qua vật": sai. Áp sát chậm vẫn được thấy.
- Bảng đo đầu tiên của thí nghiệm xe chạy vào xe máy đứng yên: vị trí xe máy được đọc trước khi CARLA cập nhật, nên không dùng. Lần đo lại với vị trí đã kiểm tra cho cùng kết luận ở 8 m/s.

## Các hướng thử

| Hướng | Nội dung | Giới hạn |
|---|---|---|
| 1. Lead thận trọng | Trừ biên sai số khỏi `dRel` (theo `xStd`), và tự tính tốc độ áp sát từ đạo hàm `dRel` khi `xStd` nhỏ | Không giúp khi model không ra lead (thùng container). Phần `vLead` chỉ cần thiết nếu lỗi Town04 là thật. |
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
