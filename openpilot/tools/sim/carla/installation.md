# Install openpilot with CARLA on Windows and WSL

This guide installs the `blueocvn/openpilot` CARLA integration on a teammate's
computer. CARLA 0.9.16 runs natively on Windows; openpilot and the Python bridge
run in Ubuntu WSL2.

The setup downloads a large CARLA package and openpilot model assets. Keep at
least **60 GB free** on the drive containing the repository.

## Architecture and driving modes

- Windows runs `CarlaUE4.exe` and listens on TCP ports 2000-2002.
- Ubuntu WSL runs manager, modeld, controls, the openpilot UI, and the bridge.
- The simulated vehicle is a Tesla Model 3.
- One road camera is enabled by default. `--dual_camera` enables the optional
  wide-road camera at additional CPU and memory cost.
- By default, the bridge emulates the Tesla's stock cruise controller while
  openpilot controls steering.
- `--openpilot-longitudinal` lets openpilot command CARLA throttle and braking.

This integration does not feed a CARLA navigation route into openpilot. At
junctions, the model chooses a path from the visible scene rather than a route
destination.

## Prerequisites

- Windows 11 with hardware virtualization enabled.
- WSL2 with WSLg and a distro named `Ubuntu`.
- A current Windows GPU driver with WSLg support. An NVIDIA GPU with at least
  8 GB VRAM is recommended.
- 32 GB system RAM recommended.
- At least 60 GB free space on `D:`.
- PowerShell 5.1 or newer and permission to add one Windows Firewall rule.

From PowerShell, verify that Ubuntu uses WSL2 and that WSLg is available:

```powershell
wsl --status
wsl -l -v
wsl -d Ubuntu -- bash -lc 'printf "DISPLAY=%s WAYLAND=%s\n" "$DISPLAY" "$WAYLAND_DISPLAY"'
```

`Ubuntu` must show version `2`, and `WAYLAND_DISPLAY` should not be empty.

## Clone the fork

Run these commands in Ubuntu WSL:

```bash
sudo apt-get update
sudo apt-get install -y git git-lfs curl clang build-essential

git lfs install
mkdir -p /mnt/d/work

git clone \
  --branch codex/carla-sim \
  --recurse-submodules \
  https://github.com/blueocvn/openpilot.git \
  /mnt/d/work/openpilot

cd /mnt/d/work/openpilot
git submodule update --init --recursive
git lfs pull
```

The `--branch codex/carla-sim` option is needed while the CARLA pull request is
under review. After the branch is merged, omit that option to clone the default
`master` branch.

Another location on `D:` works, but adjust the PowerShell path in the firewall
command below. Runtime state stays under the clone's `.carla/` directory and is
ignored by Git.

## Install openpilot and CARLA

From the repository root in Ubuntu WSL:

```bash
cd /mnt/d/work/openpilot
bash openpilot/tools/sim/carla/run.sh setup
```

The command:

1. installs the required Ubuntu packages and Git LFS hooks;
2. installs `uv` under `.carla/bin` when necessary;
3. creates and synchronizes the openpilot Python environment;
4. installs the CARLA 0.9.16 Python package and native libyuv dependency;
5. builds the generated openpilot components and native libyuv converter; and
6. downloads and extracts Windows CARLA 0.9.16 under `.carla/`.

The command is safe to run again after an interrupted download or dependency
update. Expected runtime paths are:

```text
.carla/CARLA_0.9.16/       Windows CARLA installation
.carla/downloads/          downloaded CARLA archive
.carla/logs/               CARLA, bridge, and openpilot logs
.carla/windows-profile/    isolated Windows application profile
```

## Add the Windows Firewall rule

After setup finishes, open PowerShell **as Administrator** and run:

```powershell
& 'D:\work\openpilot\openpilot\tools\sim\carla\run.ps1' `
  -Command firewall-add `
  -WslDistro Ubuntu
```

The rule permits the current Ubuntu WSL IPv4 address to reach only CARLA TCP
ports 2000-2002. The action is recorded in
`.carla/firewall-events.jsonl`.

WSL's IPv4 address can change after Windows or WSL restarts. If CARLA later
starts but the bridge cannot connect, recreate the rule from elevated
PowerShell:

```powershell
& 'D:\work\openpilot\openpilot\tools\sim\carla\run.ps1' -Command firewall-remove
& 'D:\work\openpilot\openpilot\tools\sim\carla\run.ps1' `
  -Command firewall-add `
  -WslDistro Ubuntu
```

## Run

Change to the CARLA launcher directory in Ubuntu WSL:

```bash
cd /mnt/d/work/openpilot/openpilot/tools/sim/carla
```

Run Town10:

```bash
bash run.sh all \
  --carla-town Town10HD_Opt \
  --carla-spawn-point 16
```

Run Town10 with openpilot controlling acceleration and braking:

```bash
bash run.sh all \
  --openpilot-longitudinal \
  --carla-town Town10HD_Opt \
  --carla-spawn-point 16
```

For a longer junction-free highway test:

```bash
bash run.sh all \
  --carla-town Town04_Opt \
  --carla-spawn-point 40
```

`Town04_Opt` spawn point 40 is also the default, so this shorter command is
equivalent:

```bash
bash run.sh all
```

Useful options and lifecycle commands:

```bash
bash run.sh all --dual_camera                # add the wide-road camera
OPENPILOT_UI=0 bash run.sh all               # do not open the openpilot UI
CARLA_HEADLESS=1 bash run.sh all             # render CARLA offscreen
CARLA_PORT=2001 bash run.sh all               # use another CARLA RPC port

bash run.sh status
bash run.sh stop
```

Startup is ordered automatically: Windows CARLA starts first, openpilot manager
and UI start in WSL, stationary CAN/camera frames initialize the stack, and the
bridge then engages and begins moving.

## Keyboard controls

Focus the bridge terminal before pressing a key.

| Key | Action |
| --- | --- |
| `W` | Temporary throttle override |
| `S` | Temporary brake override |
| `A` / `D` | Temporary steering override |
| `R` | Reset the CARLA vehicle and re-arm engagement |
| `I` | Toggle ignition |
| `H` | Hold or release the vehicle |
| `Q` | Stop the bridge and its openpilot processes |

Use `R` after a collision. If openpilot reports excessive actuation from an
interrupted run, stop and start the launcher; it clears only that simulator
offroad alert before manager starts.

## Longitudinal logging

From `openpilot/tools/sim/carla` while the simulation is running:

```bash
uv run --extra carla python ../log_longitudinal.py --duration 60
```

The CSV is written under `.carla/logs/` unless `--output` is supplied.

## Logs and troubleshooting

Primary logs:

```text
.carla/logs/bridge.log       camera FPS, engagement, speed, and controls
.carla/logs/openpilot.log    manager and managed-process output
.carla/logs/CarlaUE4.log     Windows CARLA output
```

The bridge prints a camera line about every two seconds. With the default
single camera on the validated machine, the expected result is approximately
20 FPS and about 5 ms per BGRA-to-NV12 conversion.

### CARLA cannot connect

```bash
ip route show default
bash run.sh status
```

Confirm that CARLA is listening on the selected port. Then recreate the
firewall rule if WSL's IPv4 address changed.

### `powershell.exe: Exec format error`

WSLInterop is unavailable in the current WSL session. Check it with:

```bash
test -e /proc/sys/fs/binfmt_misc/WSLInterop && echo ready
```

Restarting WSL usually restores interop:

```powershell
wsl --shutdown
```

Open Ubuntu again and rerun the launcher.

### UI window is behind CARLA

Use Alt+Tab and select the Ubuntu openpilot UI. NetworkManager or DBus warnings
in `openpilot.log` are expected in WSL and do not normally stop the driving UI.
