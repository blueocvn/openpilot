---
title: "Plan: clone Hoa Lac roads into CARLA from OSM"
date: 2026-11-13
summary: OSM to CARLA probe works but has no lane markings; user chose a trial swap to CARLA 0.9.15 and the Digital Twin Tool
---

# Plan: clone Hoa Lac roads into CARLA from OSM

## What happened

- Planned a clone of the Hoa Lac Hi-Tech Park road area in CARLA, sourced from OpenStreetMap because no field survey is possible (office-only work).
- Location lookup: OSM relation 19849694 bounds the park. News confirms a 24-month autonomous-vehicle sandbox on an 11.5 km road network there, but no source names the roads, and OSM has no dedicated test track.
- Measured the OSM extract (2026-11-13): core N-road grid has 39.6 km of drivable ways; `lanes` covers 59.6 % by length, `oneway` 50.2 %, no `width` tags, no elevation.
- Ran a real probe in throwaway containers (live sim untouched): `carla.Osm2Odr` converted the core grid in 0.1 s (209 roads, 26 junctions); `generate_opendrive_world` on a temporary CARLA 0.9.12 server (port 2100) loaded it in 1.4 s with 370 spawn points.
- Result: the standalone mesh is a plain flat road with no lane markings, ground, buildings or trees, so the camera model cannot be evaluated on it. Junctions of dual carriageways come out as overlapping meshes.

## Decision

- I first recommended moving to CARLA 0.9.15 for its Digital Twin Tool, then revised the recommendation to staying on 0.9.12 with a scripted mesh generator after the user asked why 0.9.12 was not proposed.
- The user chose 0.9.15: trial-swap the sim to 0.9.15 first, switch fully only if that works, then generate the map with the Digital Twin Tool.
- Scope: the N-road grid in the park core (about 1.9 x 1.8 km). No field data exists; validation uses public imagery only.
- Client constraint found: `carla==0.9.15` has a Python 3.8 wheel, 0.9.16 does not, so 0.9.15 is the newest version bukapilot can use.

## Next steps

- Plan: `plans/261113-1055-hoa-lac-carla-map/plan.md` (7 phases, phase 1 is the swap gate). Nothing was implemented.
- Phase 3 needs the user's Epic Games account linked to GitHub.
- Red-team review was not run.
- Unverified: how a map saved by the Digital Twin Tool is packaged for a server without the editor.

> Historical work record — not durable authority. Prefer docs/specs/ADRs for current decisions.
