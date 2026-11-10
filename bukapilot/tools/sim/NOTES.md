# Ghi chú tình trạng: bukapilot + CARLA + UI openpilot (WSL)

Ngày ghi: 2026-11-09. Nhánh `bukapilot-carla-sim`. Chưa commit các thay đổi bên dưới.

## 1. Đã làm xong / đã xác nhận

| Việc | Kết quả |
|---|---|
| Kết nối WSL → CARLA (cổng 2000) | **OK**. Nguyên nhân cũ: rule Windows Firewall `CARLA UE4` kiểu **Block** (profile Public) thắng rule Allow. Đã xóa các rule Block của `D:\openpilot\bukapilot\.carla\...`. |
| Xe spawn trong CARLA | OK. |
| UI openpilot (raylib) mở được, relay nối được | OK (`ui_relay` ↔ `host_ui`). |
| Venv openpilot Python 3.12 ở `D:\openpilot\.venv` | Dựng từ **WSL** (`uv sync --frozen --extra tools` + `libyuv` + `scons`). Không chạy `uv` từ Windows PowerShell trên venv này. |

## 2. Thay đổi code đã làm (chưa commit)

- `op_ui/host_ui.py`: đặt `OPENPILOT_PREFIX=opui` và tạo `/dev/shm/msgq_opui`, để UI mới không đè topic msgq của bukapilot (hai bên giờ chung `/dev/shm` vì không còn Docker).
- `run.sh`: `OP_UI=1` là mặc định; tự chạy `host_ui.py` bằng `D:\openpilot\.venv` (fallback sang UI Qt nếu thiếu venv); `stop` tắt thêm `host_ui.py`, `ui_relay.py`, `ui.py` để tránh relay cũ tranh cổng 8765.

## 3. Vấn đề còn tồn tại

### 3.1 Xe không engage: `modeld` crash (chặn chính)
- Triệu chứng: `engageable=False`, alert `commIssue/noEntry`, `not_alive` có `modelV2`, `lateralPlan`, ... ; trong log manager `modeld` màu đỏ.
- Lỗi: `error: unknown target CPU 'generic'` → `clutil.cc:84 Assertion failed`.
- Nguyên nhân: `modeld` của bukapilot 0.8.13 cần **OpenCL**. WSL chỉ có **pocl 1.8 (LLVM 11) trên CPU**, không build được kernel trên Ryzen 9 9950X3D (Zen 5). WSL chỉ có `libcuda`, **không có OpenCL của NVIDIA**.
- Vì sao commit `8cc75f698` (Docker) chạy được: chạy bukapilot trong image Ubuntu 20.04 `bukapilot-sim:local` với `--gpus all` + ICD NVIDIA. Chưa xác nhận OpenCL trong container lấy từ đâu.
- Vì sao `openpilot/tools/sim` không dính: `modeld` mới dùng **tinygrad** (không dùng OpenCL/`clutil`).

**Hướng đề xuất (theo thứ tự thử):**
1. Kiểm tra Docker còn dùng được không:
   `docker run --rm --gpus all bukapilot-sim:local clinfo | grep -i -E "platform name|device name|error"`
   - Không có `unknown target CPU` → quay lại Docker cho bukapilot (khôi phục `start_docker.sh`, giữ CARLA native).
2. Nếu lỗi y hệt: cài **pocl mới** (LLVM mới nhận Zen 5), ví dụ gói `pocl` của conda-forge, rồi trỏ `OCL_ICD_VENDORS`. Chưa thử.
3. Nếu chỉ cần xem UI mới + xe tự lái: chạy bản openpilot mới `openpilot/tools/sim/carla/run.sh` (CARLA 0.9.16, không dùng OpenCL). Đây là chạy openpilot, **không phải bukapilot**.

### 3.2 Không bấm được phím trong terminal bridge
- Chưa rõ nguyên nhân. Cần biết: dùng terminal nào (VS Code / Windows Terminal / cửa sổ WSL), phím có hiện chữ không, và trước khi sửa `run.sh` đã từng bấm được chưa.
- Lưu ý: phím không giải quyết được 3.1; xe chỉ engage được khi `modeld` chạy.

### 3.3 Camera trong UI không lên
- `host_ui` log lặp `relay connected` / `relay lost (relay disconnected)`.
- Giả thuyết: (a) nhiều `ui_relay.py` cũ cùng chạy tranh cổng 8765 (đã xử lý một phần: `run.sh` giờ dọn trước khi chạy), (b) relay `sendall` timeout 5 s do `host_ui` xử lý frame chậm.
- Cần: `pgrep -af ui_relay`, các dòng `ui_relay:` trong terminal chạy `run.sh`, và `.carla/op_ui/logs/host_ui.log`. Có thể bị che bởi 3.1 (không có modelV2).

### 3.4 Hết dung lượng ổ D: khi commit (Git LFS)
- Lỗi: `Error cleaning Git LFS object ... There is not enough space on the disk`.
- `.git\lfs\objects` ≈ 53,7 GB (không xóa); `.git\lfs\tmp` chứa file tạm từ lần commit lỗi (xóa được).
- Đã xóa `CARLA_0.9.12.zip` (5,4 GB) và dọn Thùng rác. Cần ≥ 3–5 GB trống để commit.
- Khi commit: `git status` báo `bukapilot/models/dmonitoring_model.onnx` bị "modified" (clean filter lỗi). Nếu không chủ ý sửa thì **không `git add`** file này.

## 4. Việc tiếp theo

1. Chạy thử lệnh Docker ở 3.1 → quyết định Docker hay pocl mới.
2. Commit các thay đổi ở mục 2 (sau khi có chỗ trống trên `D:`), không kèm `dmonitoring_model.onnx`.
3. Khi `modeld` chạy: kiểm tra lại camera UI (3.3) và engage (`engageable=True`).
4. Nếu vẫn không bấm được phím: cung cấp thông tin ở 3.2.
