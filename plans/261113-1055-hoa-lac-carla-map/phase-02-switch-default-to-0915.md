---
title: "Phase 2: Chuyển mặc định sang 0.9.15"
status: todo
priority: P1
effort: "1d"
dependencies: [1]
---

# Phase 2: Chuyển mặc định sang 0.9.15

## Overview

Chỉ làm khi cổng Phase 1 đạt. Đổi mặc định của `tools/sim-linux-docker` sang CARLA 0.9.15, dựng lại asset, đo lại baseline, cập nhật tài liệu.

## Requirements

- [ ] `bash run.sh all` không đặt biến nào chạy trên 0.9.15.
- [ ] `CARLA_VERSION=0.9.12` vẫn chạy được như trước.
- [ ] Baseline benchmark trên 0.9.15 được ghi lại để plan vật cản dùng tiếp.
- [ ] Tài liệu không còn nói mặc định là 0.9.12.

## Files to Create / Modify

- Modify: `bukapilot/tools/sim-linux-docker/Dockerfile.bukapilot`, `Dockerfile.carla`, `start_carla.sh`, `run.sh` (đổi giá trị mặc định).
- Modify: `bukapilot/tools/sim-linux-docker/carla-assets/build_rider_logo.py` (hằng `IMAGE`) và dựng lại các file trong `carla-assets/Content/`.
- Modify: `bukapilot/tools/sim/bridge/carla_world.py` (chú thích đầu file; chỉ sửa code nếu Phase 1 chỉ ra chỗ cần, ví dụ thuộc tính `gamma`).
- Modify: `bukapilot/tools/sim-linux-docker/README.md`, `demo.md`, `bench.md`; `bukapilot/tools/sim/COMPARISON.md`, `demo.md` (chỉ số điểm spawn).
- Modify: `bukapilot/plans/261112-1710-static-obstacle-braking-experiments/plan.md` (dòng `platform`, chỉ số spawn nếu đổi).

## Implementation Steps

1. Đổi mặc định `CARLA_VERSION` thành `0.9.15` ở bốn file; tên image mặc định thành `carla-sim:0.9.15`.
2. Dựng lại asset logo người lái cho 0.9.15 bằng `build_rider_logo.py`; kiểm bằng mắt trên cửa sổ CARLA.
3. Áp dụng các chỉnh sửa mà báo cáo Phase 1 nêu (điểm spawn tương đương, gamma).
4. Chạy `bash bench.sh standard` trên 0.9.15, lưu `results.md` làm baseline mới.
5. Cập nhật tài liệu và plan vật cản; ghi rõ baseline cũ đo trên 0.9.12.
6. Hỏi người dùng trước khi động tới nhánh Windows/WSL (`tools/sim/run.sh:103`, `tools/sim/run.ps1:22`): không kiểm chứng được trên máy này.

## Todo

- [ ] Đổi mặc định ở Dockerfile và script
- [ ] Dựng lại asset logo cho 0.9.15
- [ ] Áp dụng chỉnh sửa từ báo cáo Phase 1
- [ ] `bench.sh standard` làm baseline mới
- [ ] Cập nhật tài liệu sim
- [ ] Cập nhật plan vật cản
- [ ] Chốt với người dùng về nhánh Windows/WSL

## Success Criteria

- `bash run.sh all --carla-town Town04_Opt --carla-spawn-point <chỉ số tương đương 16>` chạy trên 0.9.15 không đặt biến.
- `CARLA_VERSION=0.9.12 bash run.sh all ...` vẫn chạy.
- Có `results.md` của `bench.sh standard` trên 0.9.15.
- `grep -rn "0\.9\.12" bukapilot/tools/sim-linux-docker` chỉ còn các chỗ nói về lựa chọn quay lại.

## Risk Assessment

- Baseline đổi làm kết luận của plan vật cản lệch. Phản ứng: giữ cả hai bộ số, ghi rõ phiên bản ở từng bảng.
- Người dùng đang chạy sim khi đổi mặc định. Phản ứng: đổi file xong mới báo, không tự dừng container đang chạy.

## Rollback

`git revert` commit đổi mặc định; image 0.9.12 vẫn còn trên máy.
