# CARLA crossing-hazard evaluation

Point-in-time evaluation of the CARLA-integrated openpilot bridge, ported from
the `carla-sim` branch (commit `4304ea8`).

## What is here

| File | Role |
| --- | --- |
| `report.md` | Generated report. **Do not hand-edit** — `generate_report.py` overwrites it, including the provenance header it currently carries. That context is repeated below so it survives. |
| `generate_report.py` | Rebuilds `report.md` from `outline.yaml`, `fields.yaml` and `results/*.json`. |
| `outline.yaml`, `fields.yaml` | Report structure and field mapping. |
| `results/*.json` | Raw Deep Research results, one file per evaluated item. |

The generator needs PyYAML, which openpilot does not depend on, so run it in a
throwaway environment rather than adding it to the project:

```bash
uv run --with pyyaml python carla-openpilot-crossing-evaluation/generate_report.py
```

## Reading the findings

These artifacts are a **historical record of what was measured at `4304ea8`**,
kept verbatim. Two things have moved since, so do not read them as a
description of the current tree:

- **Layout.** The evaluation ran when `selfdrive/`, `system/` and `tools/` sat
  at the repository root. That source now lives under `openpilot/` — for
  example `openpilot/tools/sim/lib/camerad.py`.
- **Simulated car.** The report describes the bridge publishing **Honda** CAN.
  The CARLA path now uses `openpilot/tools/sim/lib/carla_simulated_car.py`
  (Tesla Model 3, `tesla_model3_party` DBC); `simulated_car.py` remains the
  Honda Civic used by MetaDrive.

Only 3 of 14 planned items reached a validated result, so the report is a
partial evaluation, not a completed one.
