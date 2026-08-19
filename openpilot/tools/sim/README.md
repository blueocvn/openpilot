openpilot in simulator
=====================

openpilot implements a [bridge](run_bridge.py) that allows it to run in the [MetaDrive simulator](https://github.com/metadriverse/metadrive).

## Launching openpilot
First, start openpilot.
``` bash
# Run locally
./openpilot/tools/sim/launch_openpilot.sh
```

## Bridge usage
```
$ ./run_bridge.py -h
usage: run_bridge.py [-h] [--joystick] [--high_quality] [--dual_camera]
Bridge between the simulator and openpilot.

options:
  -h, --help            show this help message and exit
  --joystick
  --high_quality
  --dual_camera
```

#### Bridge Controls:
- Use `W`, `A`, `S`, and `D` for temporary manual control.
- Press `R` to reset the simulation, `I` to toggle ignition, `H` to hold or
  release the car, and `Q` to exit.

#### All inputs:

```
| key  |   functionality       |
|------|-----------------------|
|  r   | Reset Simulation      |
|  i   | Toggle Ignition       |
|  h   | Hold / release car    |
|  q   | Exit all              |
| wasd | Control manually      |
```

## CARLA on Windows/WSL

The CARLA bridge has its own setup and launcher. See
[the CARLA installation guide](carla/installation.md).

## MetaDrive

### Launching Metadrive
Start bridge processes located in openpilot/tools/sim:
``` bash
./run_bridge.py
```
