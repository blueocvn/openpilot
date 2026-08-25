#!/usr/bin/env python3
"""Record openpilot longitudinal decisions from a running simulator bridge."""

import argparse
import csv
import math
import time
from pathlib import Path

from openpilot.cereal import messaging


SERVICES = ['carState', 'carControl', 'carOutput', 'selfdriveState', 'longitudinalPlan', 'modelV2']
FIELDS = [
  'time_s', 'v_ego_mps', 'a_ego_mps2', 'v_cruise_kph', 'cruise_state_speed_mps', 'openpilot_active', 'long_active',
  'requested_accel_mps2', 'controller_accel_mps2',
  'requested_steering_angle_deg', 'controller_steering_angle_deg', 'actual_steering_angle_deg',
  'plan_source', 'plan_accel_mps2', 'plan_should_stop', 'plan_has_lead', 'plan_decel_for_turn',
  'plan_cruise_speed_mps', 'plan_speed_0_mps', 'plan_future_speed_mps', 'plan_accel_0_mps2', 'plan_future_accel_mps2',
  'model_speed_0_mps', 'model_accel_0_mps2',
  'model_path_y_0_m', 'model_path_y_1s_m', 'model_path_y_10m', 'model_path_y_20m',
  'model_hard_brake_predicted', 'lead0_probability', 'lead0_distance_m', 'lead0_speed_mps',
]


def attr(obj, name, default=math.nan):
  try:
    return getattr(obj, name)
  except (AttributeError, RuntimeError):
    return default


def item(values, index=0, default=math.nan):
  try:
    if len(values) == 0:
      return default
    return values[index]
  except (IndexError, TypeError, ZeroDivisionError):
    return default


def interpolate_at_x(xs, ys, target_x, default=math.nan):
  """Return the model path's lateral offset at a fixed distance ahead."""
  try:
    if len(xs) == 0 or len(xs) != len(ys):
      return default
    for index in range(1, len(xs)):
      x0, x1 = xs[index - 1], xs[index]
      if x0 <= target_x <= x1 and x1 != x0:
        return ys[index - 1] + (target_x - x0) * (ys[index] - ys[index - 1]) / (x1 - x0)
  except (IndexError, TypeError, ZeroDivisionError):
    pass
  return default


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--duration', type=float, default=90.0, help='recording duration in seconds')
  parser.add_argument('--output', type=Path, default=None, help='CSV path (default: .carla/logs/)')
  args = parser.parse_args()

  repo_root = Path(__file__).resolve().parents[3]
  output = args.output or repo_root / '.carla/logs' / f"longitudinal-{time.strftime('%Y%m%d-%H%M%S')}.csv"
  output.parent.mkdir(parents=True, exist_ok=True)
  sm = messaging.SubMaster(SERVICES, poll='modelV2')
  deadline = time.monotonic() + args.duration

  print(f"Recording longitudinal decisions for {args.duration:.0f}s: {output}")
  with output.open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=FIELDS)
    writer.writeheader()
    while time.monotonic() < deadline:
      sm.update(1000)
      if not sm.updated['modelV2']:
        continue

      car_state = sm['carState']
      car_control = sm['carControl']
      car_output = sm['carOutput']
      selfdrive_state = sm['selfdriveState']
      plan = sm['longitudinalPlan']
      model = sm['modelV2']
      lead0 = item(attr(model, 'leadsV3', []), default=None)
      model_position = attr(model, 'position', None)
      model_path_x = attr(model_position, 'x', [])
      model_path_y = attr(model_position, 'y', [])
      writer.writerow({
        'time_s': time.monotonic(),
        'v_ego_mps': attr(car_state, 'vEgo'),
        'a_ego_mps2': attr(car_state, 'aEgo'),
        'v_cruise_kph': attr(car_state, 'vCruise'),
        'cruise_state_speed_mps': attr(attr(car_state, 'cruiseState', None), 'speed'),
        'openpilot_active': attr(selfdrive_state, 'active', False),
        'long_active': attr(car_control, 'longActive', False),
        'requested_accel_mps2': attr(attr(car_control, 'actuators', None), 'accel'),
        'controller_accel_mps2': attr(attr(car_output, 'actuatorsOutput', None), 'accel'),
        'requested_steering_angle_deg': attr(attr(car_control, 'actuators', None), 'steeringAngleDeg'),
        'controller_steering_angle_deg': attr(attr(car_output, 'actuatorsOutput', None), 'steeringAngleDeg'),
        'actual_steering_angle_deg': attr(car_state, 'steeringAngleDeg'),
        'plan_source': str(attr(plan, 'longitudinalPlanSource', 'unknown')),
        'plan_accel_mps2': attr(plan, 'aTarget'),
        'plan_should_stop': attr(plan, 'shouldStop', False),
        'plan_has_lead': attr(plan, 'hasLead', False),
        'plan_decel_for_turn': attr(plan, 'decelForTurn', False),
        'plan_cruise_speed_mps': attr(plan, 'vCruise'),
        'plan_speed_0_mps': item(attr(plan, 'speeds', [])),
        'plan_future_speed_mps': item(attr(plan, 'speeds', []), -1),
        'plan_accel_0_mps2': item(attr(plan, 'accels', [])),
        'plan_future_accel_mps2': item(attr(plan, 'accels', []), -1),
        'model_speed_0_mps': item(attr(attr(model, 'velocity', None), 'x', [])),
        'model_accel_0_mps2': item(attr(attr(model, 'acceleration', None), 'x', [])),
        'model_path_y_0_m': item(model_path_y),
        'model_path_y_1s_m': item(model_path_y, 20),
        'model_path_y_10m': interpolate_at_x(model_path_x, model_path_y, 10.0),
        'model_path_y_20m': interpolate_at_x(model_path_x, model_path_y, 20.0),
        'model_hard_brake_predicted': attr(attr(model, 'meta', None), 'hardBrakePredicted', False),
        'lead0_probability': attr(lead0, 'prob'),
        'lead0_distance_m': item(attr(lead0, 'x', [])),
        'lead0_speed_mps': item(attr(lead0, 'v', [])),
      })
      f.flush()
  print(f"Saved {output}")


if __name__ == '__main__':
  main()
