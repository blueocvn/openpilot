import json
import math
import queue
from types import SimpleNamespace

import numpy as np

import pytest

from openpilot.tools.sim.lib.carla_radar import decode_radar, encode_radar, polar_to_vehicle, measurement_payload
from openpilot.tools.sim.bridge.carla.carla_world import CarlaWorld
from openpilot.tools.sim.lib.simulated_sensors import SimulatedSensors


def test_radar_azimuth_and_mount_keep_both_sides_distinct():
  left = polar_to_vehicle(10.0, -math.pi / 6, 0.0, 0.4)
  right = polar_to_vehicle(10.0, math.pi / 6, 0.0, 0.4)
  assert left == pytest.approx((9.060254, -5.0, 0.0), abs=1e-5)
  assert right == pytest.approx((9.060254, 5.0, 0.0), abs=1e-5)


def test_radar_payload_preserves_measurement_timestamp_and_rejects_unknown_version():
  payload = encode_radar(17, 123_000_000, [(8.0, -2.0, 0.3, -1.2)])
  result = decode_radar(payload)
  assert result.frame == 17
  assert result.timestamp_ns == 123_000_000
  assert result.points == ((8.0, -2.0, 0.3, -1.2),)
  data = json.loads(payload)
  data['v'] = 2
  assert decode_radar(json.dumps(data).encode()) is None


def test_radar_payload_rejects_nonfinite_points_and_bounds_size():
  with pytest.raises(ValueError):
    encode_radar(1, 1, [(math.nan, 0.0, 0.0, 0.0)])
  result = decode_radar(encode_radar(1, 1, [(float(x + 1), 0.0, 0.0, 0.0) for x in range(230)]))
  assert len(result.points) == 200
  assert result.truncated == 30


def test_measurement_converts_raw_carla_detection_order():
  raw = np.asarray([[-2.0, math.pi / 6, 0.0, 10.0]], dtype=np.float32).tobytes()
  measurement = SimpleNamespace(frame=41, timestamp=2.05, raw_data=raw)
  result = decode_radar(measurement_payload(measurement, timestamp_ns=91, mount_x_m=0.4))
  assert result.frame == 41
  assert result.timestamp_ns == 91
  assert result.sim_time_s == pytest.approx(2.05)
  assert result.points[0] == pytest.approx((9.060254, 5.0, 0.0, -2.0), abs=1e-5)


def test_world_drops_old_radar_frames_and_only_publishes_current_once():
  world = CarlaWorld.__new__(CarlaWorld)
  world.radar_queue = queue.Queue()
  world.current_frame = 42
  world.last_radar_frame = -1
  world.pending_radar = None
  world.radar_report = None
  world.radar_mount_x_m = 0.4
  raw = np.asarray([[-1.0, 0.0, 0.0, 8.0]], dtype=np.float32).tobytes()
  world.radar_queue.put((SimpleNamespace(frame=41, raw_data=raw), 100))
  world.radar_queue.put((SimpleNamespace(frame=42, raw_data=raw), 200))
  result = decode_radar(world.read_radar_payload())
  assert result.frame == 42
  assert result.timestamp_ns == 200
  assert world.read_radar_payload() is None


def test_simulated_sensor_publishes_observed_radar_bytes_only_when_available(monkeypatch):
  # The bridge is itself simulation, but only the separate manager sets SIMULATION.
  monkeypatch.delenv('SIMULATION', raising=False)
  monkeypatch.setenv('VN_RADAR_CUTIN', '1')
  sent = []
  sensors = SimulatedSensors.__new__(SimulatedSensors)
  sensors.pm = SimpleNamespace(send=lambda service, message: sent.append((service, message)))
  payload = encode_radar(7, 100, [(6.0, 2.0, 0.0, 1.0)])
  world = SimpleNamespace(read_radar_payload=lambda: payload)
  sensors.send_radar_message(world)
  assert len(sent) == 1
  assert sent[0][0] == 'customReservedRawData0'
  assert bytes(sent[0][1].customReservedRawData0) == payload
