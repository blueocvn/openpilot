import traceback
import numpy as np
from openpilot.cereal import messaging

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car.tesla.values import CANBUS, TeslaSafetyFlags
from openpilot.common.params import Params
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from openpilot.tools.sim.lib.common import SimulatorState

# DI_gear value for "D" (drive), see opendbc/dbc/tesla_model3_party.dbc VAL_ 280 DI_gear.
DI_GEAR_D = 4
# DI_cruiseState values, see opendbc/dbc/tesla_model3_party.dbc VAL_ 646 DI_cruiseState.
DI_CRUISE_STANDBY = 1
DI_CRUISE_ENABLED = 2
# EPAS3S_eacStatus value with no fault/inhibit, see VAL_ 880 EPAS3S_eacStatus.
EAC_ACTIVE = 2


class CarlaSimulatedCar:
  """Simulates a tesla model 3 (panda state + can messages) to OpenPilot"""
  packer = CANPacker("tesla_model3_party")

  def __init__(self):
    self.pm = messaging.PubMaster(['can', 'pandaStates'])
    self.sm = messaging.SubMaster(['carState', 'carControl', 'carOutput', 'controlsState', 'carParams', 'selfdriveState'])
    self.cp = self.get_car_can_parser()
    self.idx = 0
    self.params = Params()
    self.alpha_longitudinal_enabled = self.params.get_bool("AlphaLongitudinalEnabled")
    self.obd_multiplexing = False

  @staticmethod
  def get_car_can_parser():
    dbc_f = 'tesla_model3_party'
    checks = []
    return CANParser(dbc_f, checks, 0)

  def send_can_messages(self, simulator_state: SimulatorState):
    if not simulator_state.valid:
      return

    msg = []

    # *** party bus (vehicle state, read by CarState via Bus.party) ***

    speed = simulator_state.speed * 3.6  # convert m/s to KPH
    cruise_speed = simulator_state.cruise_speed if simulator_state.cruise_speed is not None else simulator_state.speed
    msg.append(self.packer.make_can_msg("DI_speed", CANBUS.party, {"DI_vehicleSpeed": speed}))
    msg.append(self.packer.make_can_msg("DI_systemStatus", CANBUS.party, {"DI_gear": DI_GEAR_D}))
    msg.append(self.packer.make_can_msg("ESP_status", CANBUS.party, {"ESP_driverBrakeApply": 2 if simulator_state.user_brake > 0 else 0}))
    msg.append(self.packer.make_can_msg("ESP_B", CANBUS.party, {"ESP_vehicleStandstillSts": 0 if simulator_state.speed >= 1.0 else 1}))

    # CarState reads steeringAngleDeg/steeringTorque negated from these raw signals.
    msg.append(self.packer.make_can_msg("EPAS3S_sysStatus", CANBUS.party,
                                    {
                                      "EPAS3S_handsOnLevel": 0,
                                      "EPAS3S_internalSAS": -simulator_state.steering_angle,
                                      # Rough scaling from the bridge's synthetic manual-override
                                      # torque units down into the Nm range EPAS3S actually reports.
                                      "EPAS3S_torsionBarTorque": float(np.clip(-simulator_state.user_torque / 500.0, -20.0, 20.0)),
                                      "EPAS3S_eacStatus": EAC_ACTIVE,
                                      "EPAS3S_eacErrorCode": 0,
                                    }))

    # Engagement still starts from a vehicle cruise-enabled edge even when
    # openpilot owns longitudinal actuation. Tying this signal to
    # selfdriveState.active creates a deadlock: active waits for cruise while
    # cruise waits for active.
    cruise_enabled = simulator_state.stock_cruise_enabled
    msg.append(self.packer.make_can_msg("DI_state", CANBUS.party,
                                    {
                                      "DI_cruiseState": DI_CRUISE_ENABLED if cruise_enabled else DI_CRUISE_STANDBY,
                                      "DI_speedUnits": 1, # KPH
                                      "DI_autoparkState": 0, # UNAVAILABLE
                                      # Tesla's DI_digitalSpeed is the configured cruise speed, not the
                                      # instantaneous vehicle speed. Feeding it vEgo makes longitudinal MPC
                                      # chase a setpoint that falls toward zero as the car slows.
                                      "DI_digitalSpeed": cruise_speed * 3.6,
                                    }))
    msg.append(self.packer.make_can_msg("UI_warning", CANBUS.party,
                                    {
                                      "buckleStatus": 1,
                                      "leftBlinkerBlinking": simulator_state.left_blinker,
                                      "rightBlinkerBlinking": simulator_state.right_blinker,
                                      "anyDoorOpen": 0,
                                    }))

    # *** autopilot party bus (DAS_* state, read by CarState via Bus.ap_party) ***
    msg.append(self.packer.make_can_msg("SCCM_steeringAngleSensor", CANBUS.autopilot_party, {}))
    msg.append(self.packer.make_can_msg("DAS_status", CANBUS.autopilot_party, {"DAS_blindSpotRearLeft": 0, "DAS_blindSpotRearRight": 0}))
    msg.append(self.packer.make_can_msg("DAS_control", CANBUS.autopilot_party,
                                    {"DAS_aebEvent": 0, "DAS_controlCounter": self.idx % 8}))
    msg.append(self.packer.make_can_msg("DAS_steeringControl", CANBUS.autopilot_party, {"DAS_steeringControlType": 0})) # NONE
    msg.append(self.packer.make_can_msg("DAS_settings", CANBUS.autopilot_party, {"DAS_autosteerEnabled": 0}))

    self.pm.send('can', can_list_to_can_capnp(msg))

  def send_panda_state(self, simulator_state):
    self.sm.update(0)

    if self.params.get_bool("ObdMultiplexingEnabled") != self.obd_multiplexing:
      self.obd_multiplexing = not self.obd_multiplexing
      self.params.put_bool("ObdMultiplexingChanged", True, block=True)

    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    safety_configs = self.sm["carParams"].safetyConfigs
    expected_safety_model = safety_configs[0].safetyModel if len(safety_configs) else "tesla"
    expected_safety_param = (safety_configs[0].safetyParam if len(safety_configs)
                             else (TeslaSafetyFlags.LONG_CONTROL.value if self.alpha_longitudinal_enabled else 0))
    dat.pandaStates[0] = {
      'ignitionLine': simulator_state.ignition,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': expected_safety_model,
      'alternativeExperience': self.sm["carParams"].alternativeExperience,
      'safetyParam': expected_safety_param,
    }
    self.pm.send('pandaStates', dat)

  def update(self, simulator_state: SimulatorState):
    try:
      self.send_can_messages(simulator_state)

      if self.idx % 50 == 0: # only send panda states at 2hz
        self.send_panda_state(simulator_state)

      self.idx += 1
    except Exception:
      traceback.print_exc()
      raise
