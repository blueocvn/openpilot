# bukapilot + CARLA trên Ubuntu native (Docker)

Chạy `tools/sim` (bridge hiện tại) trên Linux bằng Docker, không cần Windows/WSL.
Thư mục này chứa `run.sh` (điểm vào duy nhất), các script/Dockerfile mà `run.sh` gọi;
code bridge dùng lại từ `../sim`.

## Yêu cầu

- Ubuntu + Docker + NVIDIA driver + `nvidia-container-toolkit` (kiểm tra:
`docker run --rm --gpus all ubuntu:20.04 nvidia-smi`)
- Có màn hình (X11) nếu muốn cửa sổ CARLA / UI; không thì dùng `CARLA_HEADLESS=1`.

## Chạy

```bash
cd bukapilot/tools/sim-linux-docker
bash run.sh setup        # một lần: build image bukapilot-sim:local và carla-sim:0.9.12
bash run.sh setup-ui     # một lần, tùy chọn: UI openpilot (cần mạng, vài phút)
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 16
```

Lần chạy `all` đầu tiên còn build libyuv, tải thư viện acados và chạy scons (lâu).


| Lệnh                                  | Việc                                                                                                                                                                     |
| ------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `bash run.sh setup`                   | build image bukapilot + CARLA (`CARLA_HEADLESS=1`: pull `carlasim/carla:0.9.12`)                                                                                         |
| `bash run.sh setup-ui`                | submodule, `uv sync --frozen --extra tools --extra carla`, `scons --minimal` trong `[[ORCA_RICH_MD:0f7ddb9c681681e71b87a3522b90f157:inline-html:%3Copenpilot%3E]]/.venv` |
| `bash run.sh all [tham số bridge]`    | CARLA (chạy ngầm) + bukapilot + bridge                                                                                                                                   |
| `bash run.sh server`                  | chỉ CARLA, chạy ở foreground                                                                                                                                             |
| `bash run.sh bridge [tham số bridge]` | chỉ bukapilot + bridge (CARLA đã chạy sẵn)                                                                                                                               |
| `bash run.sh status`                  | CARLA và bridge có đang chạy không                                                                                                                                       |
| `bash run.sh stop`                    | dừng CARLA, bridge và UI openpilot trên host                                                                                                                             |



| Biến                            | Ý nghĩa                                                                                                                                                                   |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `CARLA_HEADLESS=1`              | CARLA không mở cửa sổ                                                                                                                                                     |
| `CARLA_QUALITY=Low|High`        | chất lượng render (mặc định `High`)                                                                                                                                        |
| `OP_UI=1|0`                     | 1 = UI openpilot trên host, 0 = UI Qt của bukapilot. Mặc định 1 nếu có `[[ORCA_RICH_MD:0f7ddb9c681681e71b87a3522b90f157:inline-html:%3Copenpilot%3E]]/.venv`, không thì 0 |
| `SKIP_BUILD=1` / `SHELL_ONLY=1` | bỏ qua scons / chỉ mở shell trong container                                                                                                                               |
| `CARLA_PORT`                    | cổng CARLA (mặc định 2000, host `127.0.0.1`)                                                                                                                              |


Ví dụ: `OP_UI=0 CARLA_HEADLESS=1 bash run.sh all --duration 120` (smoke test không cần màn hình).

Tham số bridge, phím điều khiển: xem [../sim/README.md](../sim/README.md).
Log: `.carla/logs/bukapilot.log` (bukapilot), `.carla/op_ui/logs/ui.log` (UI openpilot).
Dừng: phím `q` trong cửa sổ bridge, hoặc `bash run.sh stop`.