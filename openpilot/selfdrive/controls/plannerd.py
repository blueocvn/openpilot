#!/usr/bin/env python3
import os
from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ldw import LaneDepartureWarning
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
import openpilot.cereal.messaging as messaging


def update_with_observer(planner, sm, observer):
  if observer is not None:
    try:
      observer.record(sm)
    except Exception as exc:
      cloudlog.error("simulation planner observer disabled: %s", exc)
      observer = None
  planner.update(sm)
  if observer is not None:
    try:
      observer.record_output(planner)
    except Exception as exc:
      cloudlog.error("simulation planner output observer disabled: %s", exc)
      observer = None
  return observer


def main():
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("plannerd got CarParams: %s", CP.brand)

  ldw = LaneDepartureWarning()
  longitudinal_planner = LongitudinalPlanner(CP)
  observer = None
  if os.environ.get("SIMULATION") == "1" and (report_path := os.environ.get("VN_TRAFFIC_REPORT_PATH")):
    try:
      from openpilot.tools.sim.planner_trace import PlannerTraceRecorder
      observer = PlannerTraceRecorder(report_path, CP)
    except Exception as exc:
      cloudlog.error("simulation planner observer unavailable: %s", exc)
  pm = messaging.PubMaster(['longitudinalPlan', 'driverAssistance'])
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'vehicleParameters', 'radarState', 'modelV2', 'selfdriveState'],
                           poll='modelV2')

  while True:
    sm.update()
    if sm.updated['modelV2']:
      observer = update_with_observer(longitudinal_planner, sm, observer)
      longitudinal_planner.publish(sm, pm)

      ldw.update(sm.frame, sm['modelV2'], sm['carState'], sm['carControl'])
      msg = messaging.new_message('driverAssistance')
      msg.valid = sm.all_checks()
      msg.driverAssistance.leftLaneDeparture = ldw.left
      msg.driverAssistance.rightLaneDeparture = ldw.right
      pm.send('driverAssistance', msg)


if __name__ == "__main__":
  main()
