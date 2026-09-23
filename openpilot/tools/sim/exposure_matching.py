"""Deterministic comparability matching for actual CARLA cut-in exposures."""

CALIPERS = {"entry_gap_m": 2.0, "ego_speed_at_entry_mps": 1.5, "hold_s": 0.5}
DIRECTIONS = ("right_to_left", "left_to_right")


def _valid(event):
  return event.get("complete") and not event.get("evidence_missing") and all(
    isinstance(event.get(key), (int, float)) for key in CALIPERS)


def match_exposures(baseline, candidate):
  """Match valid same-direction events once, preferring the closest normalized edge."""
  pairs, unmatched_baseline, unmatched_candidate = [], [], []
  invalid_baseline = [event.get("actor_id") for event in baseline if not _valid(event)]
  invalid_candidate = [event.get("actor_id") for event in candidate if not _valid(event)]
  per_direction = {}
  for direction in DIRECTIONS:
    left = [(index, value) for index, value in enumerate(baseline) if value.get("direction") == direction and _valid(value)]
    right = [(index, value) for index, value in enumerate(candidate) if value.get("direction") == direction and _valid(value)]
    edges = []
    for li, lhs in left:
      for ri, rhs in right:
        differences = {key: abs(lhs[key] - rhs[key]) for key in CALIPERS}
        if all(differences[key] <= CALIPERS[key] for key in CALIPERS):
          edges.append((sum(differences[key] / CALIPERS[key] for key in CALIPERS), li, ri))
    used_left, used_right = set(), set()
    for score, li, ri in sorted(edges):
      if li not in used_left and ri not in used_right:
        used_left.add(li)
        used_right.add(ri)
        pairs.append({"direction": direction, "baseline": baseline[li], "candidate": candidate[ri], "normalized_difference": score})
    unmatched_baseline.extend(baseline[index].get("actor_id") for index, _ in left if index not in used_left)
    unmatched_candidate.extend(candidate[index].get("actor_id") for index, _ in right if index not in used_right)
    per_direction[direction] = sum(1 for pair in pairs if pair["direction"] == direction)
  return {"pairs": pairs, "unmatched_baseline_ids": sorted(unmatched_baseline),
          "unmatched_candidate_ids": sorted(unmatched_candidate),
          "invalid_baseline_ids": sorted(invalid_baseline), "invalid_candidate_ids": sorted(invalid_candidate),
          "per_direction_counts": per_direction,
          "comparison_valid": (not invalid_baseline and not invalid_candidate and
                               all(per_direction[direction] >= 2 for direction in DIRECTIONS))}
