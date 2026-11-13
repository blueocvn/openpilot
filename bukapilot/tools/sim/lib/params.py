import os

import cereal.messaging as messaging
from common.params import Params
from selfdrive.test.helpers import set_params_enabled as _set_params_enabled


def parse_sim_params(spec):
  """'Key=1,Other=0' -> [('Key', True), ('Other', False)]: boolean Params to set before the manager starts."""
  out = []
  for item in (spec or "").split(","):
    item = item.strip()
    if not item:
      continue
    key, _, value = item.partition("=")
    if value not in ("0", "1"):
      raise ValueError(f"SIM_PARAMS item '{item}' must be Key=0 or Key=1")
    out.append((key.strip(), value == "1"))
  return out


def set_params_enabled():
  """Persistent state needed for an engageable simulator drive on bukapilot."""
  _set_params_enabled()  # terms, training, OpenpilotEnabledToggle, Passive=False
  params = Params()

  # The simulator has no device keys and bukapilot's register() would abort
  # manager. Mark the device unregistered so registration is skipped offline.
  if not params.get("DongleId"):
    params.put("DongleId", "UnregisteredDevice")

  # 0.8.13 stores calibration as a liveCalibration event.
  msg = messaging.new_message('liveCalibration')
  msg.liveCalibration.validBlocks = 20
  msg.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
  params.put("CalibrationParams", msg.to_bytes())

  # SIM_PARAMS=Key=1,Other=0: boolean Params for a run (the feature flags under test), set the way a user would
  for key, value in parse_sim_params(os.environ.get("SIM_PARAMS")):
    params.put_bool(key, value)


if __name__ == "__main__":
  set_params_enabled()
