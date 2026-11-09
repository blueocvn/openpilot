import cereal.messaging as messaging
from common.params import Params
from selfdrive.test.helpers import set_params_enabled as _set_params_enabled


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


if __name__ == "__main__":
  set_params_enabled()
