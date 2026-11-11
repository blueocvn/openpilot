# Lệnh demo bukapilot + CARLA (Docker)

```bash
cd /home/ubuntu/openpilot/bukapilot/tools/sim-linux-docker
```

## Chuẩn bị (một lần)

```bash
bash run.sh setup        # build image bukapilot-sim:local, carla-sim:0.9.12
bash run.sh setup-ui     # tùy chọn: UI openpilot (OP_UI=1)
```

## Tình huống

```bash
# Đường cong cao tốc (mặc định)
bash run.sh all
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 16

# Đường chữ L
bash run.sh all --carla-town Town05_Opt --carla-spawn-point 1

# Đường thẳng tắp
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170

# Nhiều xe máy - đường thẳng
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 --motorcycle

# Nhiều xe máy - đường cong
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 16 --motorcycle

# Xe máy tự định nghĩa (loại:khoảng_cách_m:tốc_độ_kmh; loại = same|left|right|oncoming)
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 --motorcycle-spec "same:40:25,oncoming:120:55"

# Map khác
bash run.sh all --carla-town Town03 --carla-spawn-point <N>
```

## Tùy chọn bridge

```bash
bash run.sh all --weather HardRainNoon        # ClearNoon, CloudyNoon, WetNoon, MidRainyNoon, HardRainNoon, ClearSunset...
bash run.sh all --vehicle vehicle.tesla.model3
bash run.sh all --low_quality                 # gỡ cây, nhà, props
bash run.sh all --no-auto-engage
bash run.sh all --duration 60
bash run.sh all --joystick

# Kết hợp
bash run.sh all --carla-town Town05_Opt --carla-spawn-point 1 --weather MidRainyNoon --motorcycle
```

## Chạy từng phần / quản lý

```bash
bash run.sh server                                              # chỉ CARLA
bash run.sh bridge --carla-town Town04_Opt --carla-spawn-point 16   # chỉ bukapilot + bridge
bash run.sh status
bash run.sh stop
docker logs carla_server
```

## Biến môi trường

```bash
CARLA_HEADLESS=1 bash run.sh all
CARLA_QUALITY=High bash run.sh all
CARLA_PORT=2001 bash run.sh all
SKIP_BUILD=1 bash run.sh all
OP_UI=0 bash run.sh all      # UI Qt của bukapilot (1 = UI openpilot)
```

## Phím điều khiển

| Phím | Chức năng |
|---|---|
| 1 / 2 / 3 | cruise RES+ / SET- / cancel |
| w a s d | ga / lái trái / phanh / lái phải |
| z / x | xi-nhan trái / phải |
| h | giữ / thả xe |
| i | bật/tắt ignition |
| r | reset về điểm xuất phát |
| q | thoát |
