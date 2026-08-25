import os

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.version import terms_version, training_version


def set_params_enabled():
  """Initialize the persistent state needed by an enabled simulator drive."""
  os.environ['LOGPRINT'] = "debug"
  params = Params()
  params.put("HasAcceptedTerms", terms_version, block=True)
  params.put("CompletedTrainingVersion", training_version, block=True)
  params.put_bool("OpenpilotEnabledToggle", True, block=True)

  msg = messaging.new_message('extrinsicsCalibration')
  msg.extrinsicsCalibration.validBlocks = 20
  msg.extrinsicsCalibration.rpyCalib = [0.0, 0.0, 0.0]
  params.put("CalibrationParams", msg.to_bytes(), block=True)
