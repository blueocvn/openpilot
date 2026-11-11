# Cơ chế xử lý dữ liệu bukapilot: từ cảm biến, model đến điều khiển xe

Tài liệu mô tả luồng dữ liệu end-to-end của bukapilot (nhánh openpilot 0.8.13) khi chạy trong mô phỏng
CARLA: dữ liệu nào đi qua file nào, giao tiếp bằng message nào và logic xử lý nằm ở đâu.
Đường dẫn tính từ thư mục `bukapilot/` trừ khi ghi khác.

Tài liệu cho biết mỗi file làm gì, không thay cho việc đọc code. Các phần chưa đọc kỹ ghi rõ là "chưa kiểm tra".

## 1. Tổng quan

```
CARLA server (UE4)
   │  camera RGB, IMU, GNSS, vận tốc, góc bánh xe
   ▼
Bridge (tools/sim/bridge + tools/sim/lib)         ← "xe giả" + "cảm biến giả"
   │  ① ảnh road camera  → VisionIPC "camerad" + msg roadCameraState
   │  ② CAN giả (Honda Civic 2016) → msg can, pandaStates
   │  ③ IMU/GPS/peripheral/driverMonitoring giả → msg sensorEvents, gpsLocationExternal, ...
   ▼
bukapilot (manager.py khởi động các process)
   modeld ──modelV2──► radard ──radarState──┐
      │                                      ▼
      └──modelV2──► plannerd ──lateralPlan / longitudinalPlan──► controlsd
                                                                    │ carControl, controlsState, carState
   ◄────────────────────────────────────────────────────────────────┘
Bridge đọc carControl + controlsState  → đổi thành ga / phanh / góc lái → CARLA VehicleControl
```

Hệ thống chạy theo vòng kín: CARLA → bridge → bukapilot → bridge → CARLA. Giao tiếp giữa các process dùng
**cereal/msgq** (pub/sub qua shared memory, schema trong `cereal/log.capnp` và `cereal/car.capnp`), riêng ảnh
camera dùng **VisionIPC**.

## 2. Tầng giao tiếp (message bus)

| Thành phần | Vị trí | Vai trò |
|---|---|---|
| `cereal.messaging` (`PubMaster`, `SubMaster`) | `cereal/messaging/` | pub/sub theo tên service. `SubMaster` giữ bản message mới nhất, theo dõi `alive`/`valid`/`updated`. |
| Danh sách service và tần số | `cereal/services.py` | tên message, tần số danh định (Hz), có ghi log không. `SubMaster` dựa vào tần số này để kiểm tra `alive`. |
| VisionIPC | `cereal/visionipc/` | truyền khung ảnh YUV/RGB (stream `ROAD`, `RGB_BACK`) từ camerad sang modeld và UI. |
| Params | `common/params.py` | lưu khoá/giá trị (vd `CarParams`, `ControlsReady`). Sim ghi sẵn qua `tools/sim/lib/params.py`. |

Quy ước cần nhớ:
- `sm.updated[x]`: x có message mới trong lần `update()` này.
- `sm.alive[x]`: tần số trung bình của x không tụt quá thấp. Process nào cũng có thể bị `alive=False` nếu nguồn chậm
  (xem lỗi `ignore_avg_freq` ở `tools/sim/lib/ui_relay.py`).
- `sm.valid[x]`: cờ `valid` do chính nguồn phát ra.

## 3. Tầng sim: bridge (xe giả và cảm biến giả)

Thư mục `tools/sim/`. Entry point `run_bridge.py`, khởi động bukapilot bằng `launch_openpilot.sh`.

| File | Chức năng |
|---|---|
| `run_bridge.py` | Phân tích tham số (`--carla-town`, `--carla-spawn-point`, `--motorcycle`, ...), tạo `CarlaBridge`, chạy bukapilot (`--launch-openpilot`). |
| `launch_openpilot.sh` | Đặt env `SIMULATION=1`, `FINGERPRINT="HONDA CIVIC 2016"`, `BLOCK=camerad,loggerd(,ui)`, rồi chạy `selfdrive/manager/manager.py`. |
| `bridge/common.py` (`SimulatorBridge`) | Vòng lặp chính `_run()`: nhận phím điều khiển, cập nhật `SimulatorState`, gọi sensor/car, đổi `carControl` thành lệnh ga/phanh/lái gửi CARLA. Chứa `_openpilot_longitudinal()` (accel → ga/phanh) và `rate_limit` cho góc lái. |
| `bridge/carla_bridge.py` (`CarlaBridge`) | Tạo `CarlaWorld` với tham số town, spawn point, xe máy. |
| `bridge/carla_world.py` (`CarlaWorld`) | Kết nối CARLA: nạp map, spawn xe và camera/IMU/GNSS, spawn xe máy (Traffic Manager), `apply_controls()`, `read_sensors()` (vận tốc, góc bánh xe thực), `read_cameras()`. |
| `lib/common.py` | `SimulatorState` (speed, steering_angle, user_gas/brake/torque, cruise_button, blinker, imu, gps) và lớp trừu tượng `World`. |
| `lib/simulated_car.py` | Đóng gói trạng thái xe thành **CAN giả** của Honda Civic 2016 (tốc độ bánh, nút cruise, pedal gas, góc lái, blinker, cửa, dây an toàn...) và phát `pandaStates`. Radar bus gửi "không có mục tiêu". |
| `lib/simulated_sensors.py` | Phát IMU (`sensorEvents`), GPS (`gpsLocationExternal`), `peripheralState`, `driverMonitoringState` giả (luôn tỉnh táo) và đẩy ảnh camera 20 Hz. |
| `lib/camerad.py` | Đổi BGRA của CARLA sang I420, gửi qua VisionIPC server `camerad` và phát `roadCameraState`. Thay thế process `camerad` thật. |
| `lib/params.py` | Ghi `Params` cần có cho sim (đã chấp nhận điều khoản, calibration...). |
| `lib/keyboard_ctrl.py`, `lib/manual_ctrl.py` | Nhận phím hoặc tay cầm; gửi lệnh vào hàng đợi cho bridge. |
| `lib/ui_relay.py`, `op_ui/host_ui.py` | Chỉ khi `OP_UI=1`: chuyển message bukapilot sang UI openpilot trên host. Không ảnh hưởng điều khiển. |

Nhịp: bridge chính chạy theo nhịp tick CARLA; CAN giả 100 Hz; ảnh 20 Hz; panda 2 Hz; GPS 10 Hz.

Hệ quả quan trọng của thiết kế này:
- **Không có radar thật.** Sim gửi mọi track radar = 255,5 m (không có mục tiêu). Xe phía trước chỉ đến từ **model thị giác**
  (`modelV2.leadsV3`), `radard` không ghép được với cụm radar.
- **Người lái giả** (`driverMonitoringState`, `peripheralState`) luôn hợp lệ nên không có cảnh báo giám sát lái.
- Bridge chỉ **đọc** `carControl` và `controlsState`; nó không đọc kế hoạch của planner.

## 4. Tầng nhận thức: model (modeld)

`selfdrive/modeld/` (C++).

| File | Chức năng |
|---|---|
| `modeld.cc` | Process `modeld`. Nhận khung ảnh từ VisionIPC (`VisionIpcClient`), lấy `liveCalibration` và `lateralPlan` (desire), tính ma trận warp ảnh sang khung model, chạy mạng và phát `modelV2`, `cameraOdometry`. |
| `models/driving.cc/.h` | Chuẩn bị đầu vào (ảnh 2 khung, desire dạng xung, traffic convention), chạy model, đổi đầu ra thô thành `modelV2` (`model_publish`) và `cameraOdometry` (`posenet_publish`). |
| `models/commonmodel.cc` | Biến đổi/lọc ảnh dùng OpenCL (warp, chuyển YUV). |
| `runners/`, `thneed/` | Bộ chạy mạng (SNPE/ONNX/thneed) trên các phần cứng khác nhau. |
| `constants.py` | Hằng số kích thước (số điểm quỹ đạo...). |
| `dmonitoringmodeld*` | Model giám sát tài xế. **Tắt trong sim** (`enabled=(not PC or WEBCAM)`). |

**`modelV2`** (đầu ra chính, khoảng 20 Hz) gồm:
- `position`, `orientation`, `velocity`, `acceleration`: quỹ đạo dự đoán của xe mình theo thời gian (33 điểm, `t`/`x`/`y`/`z`, kèm độ lệch chuẩn).
- `laneLines` (4 vạch) + `laneLineProbs` + `laneLineStds`; `roadEdges` (2 lề đường).
- `leadsV3`: xe phía trước (khoảng cách, vận tốc, gia tốc, xác suất), 2 xe.
- `meta`: `desireState`, `desirePrediction` (xác suất ý định đổi làn), `engagedProb`, `hardBrakePredicted`...

**Vòng phản hồi desire**: `modeld` đọc `lateralPlan.desire` do `plannerd` phát, đưa vào model như đầu vào dạng xung. Nhờ vậy model biết
"đang đổi làn trái/phải" và vẽ quỹ đạo tương ứng.

## 5. Tầng xử lý: radard, plannerd, controlsd

Tất cả tên process ở `selfdrive/manager/process_config.py`, do `manager.py` khởi động.

### 5.1 `radard` (`selfdrive/controls/radard.py`)
Hợp nhất radar và thị giác thành `radarState` (`leadOne`, `leadTwo`).
- Đọc track radar từ `RadarInterface` của xe (Honda: `selfdrive/car/honda/radar_interface.py`), mỗi track lọc Kalman (`radar_helpers.Track`), gom cụm (`cluster_points_centroid`).
- `match_vision_to_cluster()` ghép `modelV2.leadsV3` với cụm radar gần nhất theo xác suất (khoảng cách, lệch ngang, vận tốc).
- `get_lead()` trả lead cuối: nếu có cụm khớp thì lấy radar, **nếu không có thì dùng dữ liệu thị giác** (đường này luôn xảy ra trong sim).
- Phát `radarState` với `leadOne.{status, dRel, yRel, vRel, vLead, aLeadK, ...}`.

### 5.2 `plannerd` (`selfdrive/controls/plannerd.py`)
Một vòng lặp, chạy mỗi khi có `modelV2` mới. Gọi hai planner:

**a) `LateralPlanner`** (`selfdrive/controls/lib/lateral_planner.py`), phát `lateralPlan`
1. `LanePlanner.parse_model()` (`lane_planner.py`) lấy vạch trái/phải (index 1 và 2), xác suất, độ lệch chuẩn, xác suất đổi làn.
2. `DesireHelper.update()` (`desire_helper.py`) là máy trạng thái đổi làn: `off → preLaneChange → laneChangeStarting → laneChangeFinishing`. Điều kiện: bật xi-nhan, tốc độ tối thiểu `LANE_CHANGE_SPEED_MIN` (30 mph), không có xe trong điểm mù, và người lái có tác động lực lái (`steeringPressed`). Kết quả là `desire` (none / laneChangeLeft / laneChangeRight / ...).
3. `LanePlanner.get_d_path()` trộn **quỹ đạo của model** với **đường giữa hai vạch** (theo xác suất và độ tin cậy vạch). Đây là điểm "lane-lines + e2e" của bukapilot (`use_lanelines = True`, log `e2e mode`).
4. `LateralMpc` (`lateral_mpc_lib/`, acados) giải bài toán tối ưu bám đường `y_pts` và hướng `heading_pts`, tối thiểu hoá lệch và tốc độ đổi độ cong; trả `curvatures`, `psis`, `curvatureRates` (17 điểm).
5. Kiểm tra nghiệm MPC; nếu nan hay cost > 20000 thì reset và đánh dấu `mpcSolutionValid = False`.

**b) `Planner` dọc** (`selfdrive/controls/lib/longitudinal_planner.py`), phát `longitudinalPlan`
1. Đầu vào: `vEgo`, `aEgo` (carState), `vCruise` và `forceDecel` (controlsState), `radarState`.
2. Giới hạn gia tốc: `get_max_accel(v_ego)`, `A_CRUISE_MIN`, và `limit_accel_in_turns()` (giảm gia tốc khi góc lái lớn).
3. `LongitudinalMpc` (`longitudinal_mpc_lib/`) giải quỹ đạo vận tốc/gia tốc/jerk, ràng buộc bởi `vCruise` và lead (giữ khoảng cách theo thời gian).
4. Phát `speeds`, `accels`, `jerks` (17 điểm), `hasLead`, `longitudinalPlanSource` (cruise hay lead), cảnh báo va chạm `fcw`.

### 5.3 `controlsd` (`selfdrive/controls/controlsd.py`, 796 dòng)
Vòng điều khiển 100 Hz. Mỗi `step()`:
1. `data_sample()`: đọc CAN (`drain_sock_raw`), `CI.update()` biến CAN thành `carState`; `sm.update()` đọc các message khác.
2. `update_events()`: gom sự kiện/cảnh báo (`lib/events.py`, `alertmanager.py`).
3. `state_transition()`: máy trạng thái `disabled → preEnabled → enabled → softDisabling → overriding`, xử lý nút cruise, engage/disengage.
4. `state_control()`: tính **actuators**:
   - `LongControl.update()` (`lib/longcontrol.py`): trạng thái `off/pid/stopping/starting`, PID (`lib/pid.py`) bám `longitudinalPlan` → `actuators.accel`.
   - `LatControl*.update()` chọn theo loại xe (`latcontrol_pid.py`, `_indi.py`, `_lqr.py`, `_angle.py`) và dùng `VehicleModel` (`lib/vehicle_model.py`) để đổi `desired_curvature` (qua `get_lag_adjusted_curvature` ở `drive_helpers.py`) thành `steer` và `steeringAngleDeg`.
   - `lat_active` bị tắt khi: chưa `active`, đứng yên (`CS.standstill`), `vEgo <= minSteerSpeed`, đang bật xi-nhan mà không đổi làn, `steerWarning/Error`.
   - Sau khi bật lại lái, `reduce_steer()` giảm lực lái trong 2 s đầu.
5. `publish_logs()`: phát `controlsState` (có `vCruise`, `active`, `longControlState`, `curvature`), `carControl` (có `actuators`), `carState`, `carEvents`, ...

## 6. Tầng xe (car interface)

`selfdrive/car/` (mỗi hãng một thư mục; sim dùng `honda`).

| File | Chức năng |
|---|---|
| `car_helpers.py` (`get_car`) | Nhận dạng xe qua fingerprint CAN (sim ép bằng `FINGERPRINT`), nạp `CarInterface`, `CarState`, `CarController`; ghi `CarParams` vào Params. |
| `interfaces.py` | Lớp cơ sở `CarInterfaceBase` (`update` → `carState`, `apply` → lệnh CAN), `CarStateBase`, `RadarInterfaceBase`. |
| `honda/interface.py` | `CarParams` (khối lượng, tỉ số lái, `minSteerSpeed`, `openpilotLongitudinalControl`, `pcmCruise`...). Ghép với pedal comma nên bật điều khiển dọc bởi openpilot. |
| `honda/carstate.py` | Giải mã CAN (tốc độ bánh, góc lái, nút cruise, blinker, pedal) thành `carState`. |
| `honda/carcontroller.py`, `hondacan.py` | Đổi `actuators` thành khung CAN gửi xe (`sendcan`). Trong sim, đường `sendcan` không dùng; bridge đọc trực tiếp `carControl`. |
| `honda/radar_interface.py` | Giải mã track radar từ CAN (sim gửi rỗng). |

## 7. Các process phụ trợ ảnh hưởng đến dữ liệu

| Process | File | Vai trò |
|---|---|---|
| `calibrationd` | `selfdrive/locationd/calibrationd.py` | Ước lượng góc lắp camera, phát `liveCalibration` (modeld cần để warp ảnh). |
| `locationd` | `selfdrive/locationd/locationd.cc` | Lọc Kalman IMU/GPS/odometry, phát `liveLocationKalman` (controlsd kiểm tra `sensorsOK`). |
| `paramsd` | `selfdrive/locationd/paramsd.py` | Ước lượng `steerRatio`, `stiffnessFactor`, `angleOffset`, phát `liveParameters` (cho `VehicleModel`). |
| `dmonitoringd` | `selfdrive/monitoring/dmonitoringd.py` | Giám sát tài xế. Trong sim bị thay bằng message giả. |
| `ui` (Qt) hoặc UI openpilot | `selfdrive/ui/`, `tools/sim/op_ui/` | Chỉ hiển thị, không tham gia điều khiển. |

## 8. Đường đi của một chu kỳ (end-to-end)

Ví dụ một chu kỳ tại thời điểm xe đang engage:

1. CARLA tick; camera callback trả khung RGB.
2. `bridge/common.py` `_run()`: `world.read_sensors()` cập nhật `SimulatorState` (vận tốc, góc bánh thực, IMU, GPS).
3. Thread riêng: `simulated_car.update()` (100 Hz) phát `can`; `simulated_sensors.send_camera_images()` (20 Hz) gọi `camerad.send_bgra()` đẩy ảnh qua VisionIPC.
4. `modeld` nhận ảnh, chạy mạng, phát `modelV2` (quỹ đạo, vạch, lead).
5. `radard` đọc `modelV2` + `carState`, phát `radarState`.
6. `plannerd` đọc `modelV2`, `radarState`, `carState`, `controlsState`, phát `lateralPlan` (độ cong) và `longitudinalPlan` (vận tốc/gia tốc).
7. `controlsd` (100 Hz) đọc CAN → `carState`, đọc 2 plan, tính `actuators.accel` và `actuators.steeringAngleDeg`, phát `carControl` + `controlsState`.
8. Bridge đọc `controlsState.active`, `carControl.actuators`:
   - dọc: `_openpilot_longitudinal()` đóng vòng PI quanh `aEgo`, cắt ga khi vượt `vCruise + 0,5 m/s`;
   - ngang: `steeringAngleDeg` qua `rate_limit(...)`, rồi `world.apply_controls()` đổi sang `steer = -angle / (max_wheel_angle * STEER_RATIO)`.
9. CARLA nhận `VehicleControl(steer, throttle, brake)`, vòng lặp tiếp tục.

Phím điều khiển (`w a s d`, `1 2 3`, `z x`, `h`) ghi đè ở bước 8: phím lái/ga/phanh thắng openpilot, phím `1 2 3` giả lập nút cruise trên CAN, `z x` giả lập xi-nhan.

## 9. Điểm cần chú ý khi gỡ lỗi

| Hiện tượng | Nơi cần xem |
|---|---|
| Xe không engage | `controlsState.engageable`, `carState.events`; bridge tự bấm SET khi `engageable` (`bridge/common.py`). |
| Xe chạy sai tốc độ | `controlsState.vCruise` → `longitudinalPlan.speeds` → `longcontrol` → `_openpilot_longitudinal`. |
| Xe không né/vượt | Không có logic né vật cản; đổi làn chỉ qua `desire_helper.py` (xi-nhan + lực lái). |
| Góc lái giữ nguyên khi chậm/dừng | `controlsd.state_control` (`lat_active`) và `bridge/common.py` dùng `actuators.steeringAngleDeg` (xem mục 5.3). |
| Không thấy lead | Sim không có radar; kiểm tra `modelV2.leadsV3` và `radarState.leadOne.status`. |
| UI không vẽ | `alive` của message (xem `ui_relay.py`), font LFS. |

Log: `bukapilot/.carla/logs/` (`bridge.log`, `bukapilot.log`, `CarlaUE4.log`).

## 10. Phần chưa kiểm tra

- Nội dung chi tiết của `cereal/services.py` (tần số từng service) và các hằng số trong `longitudinal_planner.py` (`A_CRUISE_MIN`, `get_max_accel`).
- Logic nội bộ `latcontrol_*.py` ngoài `latcontrol_pid.py`; loại bộ điều khiển lái thực sự dùng cho Honda Civic 2016 trong sim.
- Chi tiết kích thước tensor và tiền xử lý trong `driving.cc`, `commonmodel.cc`.
- Phần `carcontroller.py` của Honda và đường `sendcan` (giả định không dùng trong sim).
