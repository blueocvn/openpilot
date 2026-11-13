# Phase 4: Clear-distance source trial

## Context

The driving model gives no "confirmed clear to X metres" signal. A low lead probability does not mean clear: the container reads `prob` about 0.2, close to an empty road. `d_clear` therefore needs a source that is independent of lead detection and does not depend on the object class. This phase tests candidates offline on recorded frames, then integrates the winner only if it passes and the user approves.

## Candidates

| Candidate | Method | Depends on | Fails first when |
|---|---|---|---|
| C1. Ground-plane check | With known ego speed and the verified camera geometry, predict how road-surface pixels move between two frames. Regions inside the lane corridor that move differently stand above the road. `d_clear` is the distance to the nearest such region, from its ground-contact row | Ego motion (needs speed above about 1 m/s) and road texture | Standstill, smooth untextured road, strong pitch changes |
| C2. Road segmentation or depth network | A small network marks drivable surface; `d_clear` is how far the lane corridor stays drivable | The network working on CARLA images | Domain gap, compute per frame |
| C3. Looming time-to-contact | Expansion rate of image structure in the corridor gives time to contact; `d_clear = ttc * v_ego` | Enough texture on the object | Curves, small far objects |

C1 is tried first: it needs no model, no object classes, and uses geometry already verified within 2 px.

## Output contract

One message at camera rate: `d_clear` in metres (capped at 150), a confidence value, and a validity flag. When invalid (standstill, low texture), the planner uses the fallback distance. A second variant later (another camera or another estimator) keeps this contract; only the estimator changes.

## Files

- Phase 1 extended: `--lead-log` can record every frame of a run with ground truth (gap, ego speed, pose).
- Add offline scripts under `tools/sim/tests/manual/`.
- If the trial passes and the user approves: add `selfdrive/controls/clear_distance.py` (new process), a `clearDistance` message in `cereal/log.capnp`, and a manager entry.
- Edit `plans/Report/report.md` with the trial results.

## Steps

1. Record sequences with ground truth at 8 and 15 m/s in both towns: container, street barrier, stationary motorcycle, parked car, clear road, a curve, a slope, a road with painted markings and shadows.
2. Implement C1 offline. Score per sequence: distance at first valid detection, `d_clear` error against ground truth, fraction of clear-road frames reporting less than 60 m.
3. If C1 fails the pass condition, run C2 and C3 the same way.
4. Present results. Go or no-go with the user.
5. On go: implement the process with tests first (synthetic frame pairs with a known plane and a known obstacle), wire the message, rerun the recorded sequences through the live process.

## Validation

A candidate passes when, on the recordings:

- container, barrier and motorcycle are reported at 40 m or more at 8 m/s, and at 60 m or more at 15 m/s (enough to stop from 60 km/h at 2 m/s²);
- `d_clear` is never longer than the true gap by more than 3 m inside 40 m;
- on clear, curved, sloped and shadowed roads, less than 1% of frames report under 60 m.

## Risk and rollback

The trial is offline and has no runtime risk. The integrated process only publishes a message; nothing consumes it until phase 5 turns its flag on.
