"""Simulated Honda Civic 2016 (Nidec + comma pedal) for bukapilot.

Same CAN layout as the legacy tools/sim/lib/can.py, restructured like
openpilot/tools/sim/lib/simulated_car.py and extended with the driver inputs
the legacy bridge never reported (steering torque, brake pedal, gas pedal,
blinkers), so openpilot sees overrides the way it would in a real car.

The car is fingerprinted as "HONDA CIVIC 2016". GAS_SENSOR (0x201) makes
interface.py set enableGasInterceptor, hence openpilotLongitudinalControl=True
and pcmCruise=False: openpilot owns the set speed and engages on the
SET/RES button edge.
"""
import traceback

import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp  # pylint: disable=no-name-in-module,import-error
from selfdrive.car import crc8_pedal
from tools.sim.lib.common import SimulatorState

GEAR_SHIFTER_D = 8  # VAL_ 401 GEAR_SHIFTER 8 "D"
PEDAL_RELEASED = -0.2  # decodes to INTERCEPTOR_GAS -0.07 / GAS2 -0.25 -> gasPressed False


class SimulatedCar:
  packer = CANPacker("honda_civic_touring_2016_can_generated")
  rpacker = CANPacker("acura_ilx_2016_nidec")

  def __init__(self):
    self.pm = messaging.PubMaster(['can', 'pandaStates'])
    self.sm = messaging.SubMaster(['carState', 'carControl', 'controlsState', 'carParams'])
    self.idx = 0

  def send_can_messages(self, s: SimulatorState):
    if not s.valid:
      return

    idx = self.idx
    msg = []
    packer = self.packer

    # *** powertrain bus ***
    speed = s.speed * 3.6  # m/s -> kph
    msg.append(packer.make_can_msg("ENGINE_DATA", 0, {"XMISSION_SPEED": speed}, idx))
    msg.append(packer.make_can_msg("WHEEL_SPEEDS", 0, {
      "WHEEL_SPEED_FL": speed,
      "WHEEL_SPEED_FR": speed,
      "WHEEL_SPEED_RL": speed,
      "WHEEL_SPEED_RR": speed,
    }, -1))

    msg.append(packer.make_can_msg("SCM_BUTTONS", 0, {"CRUISE_BUTTONS": s.cruise_button}, idx))

    # comma pedal: carstate averages INTERCEPTOR_GAS/GAS2 and anything > 1e-5 is
    # gasPressed. 0.0 does not survive the DBC encoding (offset -83.3, scale
    # 0.254 rounds to raw 328 = +0.0066), so a released pedal is sent slightly
    # negative, which decodes to <= 0.
    gas = float(s.user_gas) * 100.0 if s.user_gas > 0 else PEDAL_RELEASED
    values = {"COUNTER_PEDAL": idx & 0xF, "INTERCEPTOR_GAS": gas, "INTERCEPTOR_GAS2": gas}
    values["CHECKSUM_PEDAL"] = crc8_pedal(packer.make_can_msg("GAS_SENSOR", 0, values, -1)[2][:-1])
    msg.append(packer.make_can_msg("GAS_SENSOR", 0, values, -1))

    msg.append(packer.make_can_msg("GEARBOX", 0, {"GEAR": 4, "GEAR_SHIFTER": GEAR_SHIFTER_D}, idx))
    msg.append(packer.make_can_msg("GAS_PEDAL_2", 0, {}, idx))
    msg.append(packer.make_can_msg("SEATBELT_STATUS", 0, {"SEATBELT_DRIVER_LATCHED": 1}, idx))
    msg.append(packer.make_can_msg("STEER_STATUS", 0, {"STEER_TORQUE_SENSOR": s.user_torque}, idx))
    msg.append(packer.make_can_msg("STEERING_SENSORS", 0, {"STEER_ANGLE": s.steering_angle}, idx))
    msg.append(packer.make_can_msg("VSA_STATUS", 0, {}, idx))
    msg.append(packer.make_can_msg("STANDSTILL", 0, {"WHEELS_MOVING": 1 if s.speed >= 1.0 else 0}, idx))
    msg.append(packer.make_can_msg("STEER_MOTOR_TORQUE", 0, {}, idx))
    msg.append(packer.make_can_msg("EPB_STATUS", 0, {}, idx))
    msg.append(packer.make_can_msg("DOORS_STATUS", 0, {}, idx))
    msg.append(packer.make_can_msg("CRUISE_PARAMS", 0, {}, idx))
    msg.append(packer.make_can_msg("CRUISE", 0, {}, idx))
    msg.append(packer.make_can_msg("SCM_FEEDBACK", 0, {
      "MAIN_ON": 1,
      "LEFT_BLINKER": s.left_blinker,
      "RIGHT_BLINKER": s.right_blinker,
    }, idx))
    msg.append(packer.make_can_msg("POWERTRAIN_DATA", 0, {
      # Nidec with the pedal is not pcmCruise, so ACC_STATUS only mirrors state.
      "ACC_STATUS": int(s.is_engaged),
      "BRAKE_PRESSED": int(s.user_brake > 0),
    }, idx))
    msg.append(packer.make_can_msg("HUD_SETTING", 0, {}))

    # *** cam bus ***
    msg.append(packer.make_can_msg("STEERING_CONTROL", 2, {}, idx))
    msg.append(packer.make_can_msg("ACC_HUD", 2, {}, idx))
    msg.append(packer.make_can_msg("BRAKE_COMMAND", 2, {}, idx))

    # *** radar bus: no targets ***
    if idx % 5 == 0:
      msg.append(self.rpacker.make_can_msg("RADAR_DIAGNOSTIC", 1, {"RADAR_STATE": 0x79}, -1))
      for i in range(16):
        msg.append(self.rpacker.make_can_msg("TRACK_%d" % i, 1, {"LONG_DIST": 255.5}, -1))

    self.pm.send('can', can_list_to_can_capnp(msg))

  def send_panda_state(self, s: SimulatorState):
    self.sm.update(0)

    # Mirror what controlsd expects (safetyModel and safetyParam) once carParams
    # is published, so a param mismatch can't trip controlsMismatch.
    safety_configs = self.sm['carParams'].safetyConfigs
    safety_model = safety_configs[0].safetyModel if len(safety_configs) else 'hondaNidec'
    safety_param = safety_configs[0].safetyParam if len(safety_configs) else 0

    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    dat.pandaStates[0] = {
      'ignitionLine': s.ignition,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': safety_model,
      'safetyParam': safety_param,
    }
    self.pm.send('pandaStates', dat)

  def update(self, s: SimulatorState):
    try:
      self.send_can_messages(s)
      if self.idx % 50 == 0:  # pandaStates at 2 Hz
        self.send_panda_state(s)
      self.idx += 1
    except Exception:
      traceback.print_exc()
      raise
