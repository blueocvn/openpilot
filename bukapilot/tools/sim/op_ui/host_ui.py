#!/usr/bin/env python3
"""Show bukapilot's drive in the current openpilot UI.

Runs on the host with openpilot's Python 3.12 venv. Receives bukapilot state
and the road camera from lib/ui_relay.py (inside the bukapilot container),
translates them from the 0.8.13 schema to the current one (controlsState ->
selfdriveState, liveCalibration -> extrinsicsCalibration, I420 -> padded
NV12, ...), publishes them on the host's msgq/VisionIPC and starts
openpilot/selfdrive/ui/ui.py, which draws them like any openpilot drive.

The container has its own /dev/shm, so these host topics never mix with
bukapilot's. Don't run openpilot's own simulator at the same time.

  PYTHONPATH=<openpilot repo> <openpilot repo>/.venv/bin/python host_ui.py
"""
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import time

OP_ROOT = os.environ.get("OPENPILOT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))
STATE_DIR = os.environ.get("OP_UI_STATE", os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.carla/op_ui")))
# keep the UI's params away from a real openpilot setup on this machine
os.environ.setdefault("PARAMS_ROOT", os.path.join(STATE_DIR, "params"))
sys.path.insert(0, OP_ROOT)

import numpy as np  # noqa: E402

from openpilot.cereal import log, messaging  # noqa: E402
from opendbc.car.structs import car  # noqa: E402
from openpilot.cereal.visionipc import VisionStreamType  # noqa: E402
from msgq.visionipc import VisionIpcServer  # noqa: E402
from openpilot.common.params import Params  # noqa: E402
from openpilot.common.version import terms_version, training_version  # noqa: E402
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info  # noqa: E402

HEADER = struct.Struct(">II")
W, H = 1928, 1208
HEIGHT_INIT = 1.22  # calibrationd default device height, which the 0.8.13 model assumes too

SERVICES = ['deviceState', 'pandaStates', 'selfdriveState', 'controlsState', 'carState', 'modelV2', 'radarState',
            'extrinsicsCalibration', 'carParams', 'carControl', 'carOutput', 'longitudinalPlan',
            'driverMonitoringState', 'driverStateV2', 'narrowRoadCameraState']


def recv_exact(sock, n):
  buf = bytearray(n)
  view = memoryview(buf)
  got = 0
  while got < n:
    r = sock.recv_into(view[got:], n - got)
    if r == 0:
      raise ConnectionError("relay disconnected")
    got += r
  return buf


class Translator:
  def __init__(self):
    self.pm = messaging.PubMaster(SERVICES)
    self.stride, self.y_height, self.uv_height, self.size = get_nv12_info(W, H)
    self.vipc = VisionIpcServer("camerad")
    self.vipc.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_NARROW_ROAD, 5, W, H,
                                        self.size, self.stride, self.stride * self.y_height)
    self.vipc.start_listener()
    self.nv12 = np.zeros(self.size, dtype=np.uint8)
    self.frame_id = 0
    self.last_slow = 0.0
    self.car_params_sent = False

  def i420_to_nv12(self, i420):
    y = i420[:W * H].reshape(H, W)
    u = i420[W * H:W * H * 5 // 4].reshape(H // 2, W // 2)
    v = i420[W * H * 5 // 4:].reshape(H // 2, W // 2)
    out = self.nv12
    out[:self.stride * self.y_height].reshape(self.y_height, self.stride)[:H, :W] = y
    uv = out[self.stride * self.y_height:self.stride * self.y_height + self.stride * self.uv_height]
    uv = uv.reshape(self.uv_height, self.stride)
    uv[:H // 2, 0:W:2] = u
    uv[:H // 2, 1:W:2] = v
    return out.tobytes()

  def send_slow(self):
    """deviceState/pandaStates/driver monitoring: just enough to be onroad."""
    dat = messaging.new_message('deviceState', valid=True)
    dat.deviceState.started = True
    dat.deviceState.deviceType = log.InitData.DeviceType.pc
    self.pm.send('deviceState', dat)

    dat = messaging.new_message('pandaStates', 1, valid=True)
    dat.pandaStates[0].ignitionLine = True
    dat.pandaStates[0].pandaType = log.PandaState.PandaType.blackPanda
    self.pm.send('pandaStates', dat)

    # same fake driver monitoring as openpilot/tools/sim/lib/simulated_sensors.py
    dat = messaging.new_message('driverMonitoringState', valid=True)
    dm = dat.driverMonitoringState
    dm.alertLevel = log.DriverMonitoringState.AlertLevel.none
    dm.activePolicy = log.DriverMonitoringState.MonitoringPolicy.vision
    dm.visionPolicyState.faceDetected = True
    dm.visionPolicyState.isDistracted = False
    dm.visionPolicyState.awarenessPercent = 100
    self.pm.send('driverMonitoringState', dat)

    dat = messaging.new_message('driverStateV2', valid=True)
    dat.driverStateV2.leftDriverData.faceOrientation = [0., 0., 0.]
    dat.driverStateV2.leftDriverData.faceProb = 1.0
    dat.driverStateV2.rightDriverData.faceOrientation = [0., 0., 0.]
    dat.driverStateV2.rightDriverData.faceProb = 1.0
    self.pm.send('driverStateV2', dat)

    dat = messaging.new_message('longitudinalPlan', valid=True)
    dat.longitudinalPlan.allowThrottle = True
    self.pm.send('longitudinalPlan', dat)

  def send_car_params(self, cp):
    msg = car.CarParams.new_message()
    msg.carFingerprint = cp["carFingerprint"]
    msg.openpilotLongitudinalControl = cp["openpilotLongitudinalControl"]
    Params().put("CarParamsPersistent", msg.to_bytes())
    dat = messaging.new_message('carParams', valid=True)
    dat.carParams = msg
    self.pm.send('carParams', dat)

  def publish(self, p, frame):
    now = time.monotonic()
    if now - self.last_slow > 0.25:
      self.send_slow()
      self.last_slow = now

    cs, cst, m = p["controlsState"], p["carState"], p["modelV2"]

    if frame is not None:
      self.vipc.send(VisionStreamType.VISION_STREAM_NARROW_ROAD, self.i420_to_nv12(frame), self.frame_id,
                     time.monotonic_ns(), time.monotonic_ns())
      dat = messaging.new_message('narrowRoadCameraState', valid=True)
      dat.narrowRoadCameraState.frameId = self.frame_id
      self.pm.send('narrowRoadCameraState', dat)
      self.frame_id += 1

    if not p["alive"].get("controlsState"):
      return  # bukapilot not up yet: stay on the onroad screen without stale state

    dat = messaging.new_message('selfdriveState', valid=True)
    ss = dat.selfdriveState
    ss.enabled = cs["enabled"]
    ss.active = cs["active"]
    ss.engageable = cs["engageable"]
    ss.state = cs["state"]
    ss.alertText1 = cs["alertText1"]
    ss.alertText2 = cs["alertText2"]
    ss.alertSize = cs["alertSize"]
    ss.alertStatus = cs["alertStatus"]
    ss.alertType = cs["alertType"]
    self.pm.send('selfdriveState', dat)

    dat = messaging.new_message('controlsState', valid=True)
    dat.controlsState.curvature = cs["curvature"]
    dat.controlsState.desiredCurvature = cs["curvature"]
    dat.controlsState.deprecated.vCruise = cs["vCruise"]
    dat.controlsState.lateralControlState.init('pidState')
    self.pm.send('controlsState', dat)

    dat = messaging.new_message('carState', valid=True)
    c = dat.carState
    c.vEgo = cst["vEgo"]
    c.vEgoCluster = cst["vEgoCluster"] or cst["vEgo"]
    c.aEgo = cst["aEgo"]
    c.standstill = cst["standstill"]
    c.steeringAngleDeg = cst["steeringAngleDeg"]
    c.steeringTorque = cst["steeringTorque"]
    c.steeringPressed = cst["steeringPressed"]
    c.leftBlinker = cst["leftBlinker"]
    c.rightBlinker = cst["rightBlinker"]
    # Honda with the pedal is not pcmCruise: the set speed lives in controlsState.vCruise
    c.vCruise = cs["vCruise"]
    c.vCruiseCluster = cs["vCruise"] if cs["enabled"] else 0.0
    c.cruiseState.enabled = cst["cruiseEnabled"]
    c.cruiseState.available = cst["cruiseAvailable"]
    c.cruiseState.speed = cst["cruiseSpeed"]
    self.pm.send('carState', dat)

    cc = p["carControl"]
    dat = messaging.new_message('carControl', valid=True)
    dat.carControl.enabled = cc["enabled"]
    dat.carControl.latActive = cc["active"]
    dat.carControl.longActive = cc["active"]
    self.pm.send('carControl', dat)
    dat = messaging.new_message('carOutput', valid=True)
    dat.carOutput.actuatorsOutput.torque = cc["steer"]
    self.pm.send('carOutput', dat)

    if p["updated"].get("modelV2") or frame is not None:
      dat = messaging.new_message('modelV2', valid=True)
      mv = dat.modelV2
      mv.frameId = m["frameId"]
      mv.position = m["position"]
      mv.laneLines = m["laneLines"]
      mv.laneLineProbs = m["laneLineProbs"]
      mv.roadEdges = m["roadEdges"]
      mv.roadEdgeStds = m["roadEdgeStds"]
      mv.acceleration.x = m["accelerationX"]
      mv.meta.disengagePredictions.brakeDisengageProbs = m["brakeDisengageProbs"]
      mv.meta.disengagePredictions.steerOverrideProbs = m["steerOverrideProbs"]
      self.pm.send('modelV2', dat)

      rs = p["radarState"]
      dat = messaging.new_message('radarState', valid=True)
      for name in ("leadOne", "leadTwo"):
        src, dst = rs[name], getattr(dat.radarState, name)
        dst.dRel, dst.yRel, dst.vRel, dst.present = src["dRel"], src["yRel"], src["vRel"], src["status"]  # status -> present
      self.pm.send('radarState', dat)

      cal = p["liveCalibration"]
      dat = messaging.new_message('extrinsicsCalibration', valid=True)
      dat.extrinsicsCalibration.calStatus = 'calibrated' if cal["calStatus"] == 1 else 'uncalibrated'
      dat.extrinsicsCalibration.rpyCalib = cal["rpyCalib"] if len(cal["rpyCalib"]) == 3 else [0., 0., 0.]
      dat.extrinsicsCalibration.height = [HEIGHT_INIT]
      self.pm.send('extrinsicsCalibration', dat)

    if not self.car_params_sent and p["carParams"]["carFingerprint"]:
      self.send_car_params(p["carParams"])
      self.car_params_sent = True


def init_params():
  os.makedirs(os.environ["PARAMS_ROOT"], exist_ok=True)
  params = Params()
  params.put("HasAcceptedTerms", terms_version)
  params.put("CompletedTrainingVersion", training_version)
  params.put_bool("OpenpilotEnabledToggle", True)
  params.put_bool("IsMetric", True)
  params.put_bool("ExperimentalMode", False)


def start_ui():
  env = os.environ.copy()
  env["PYTHONPATH"] = OP_ROOT
  env.setdefault("SCALE", "1.5")  # same window size as openpilot's .carla/run-linux.sh
  log_dir = os.path.join(STATE_DIR, "logs")
  os.makedirs(log_dir, exist_ok=True)
  log = open(os.path.join(log_dir, "ui.log"), "w")
  return subprocess.Popen([sys.executable, os.path.join(OP_ROOT, "openpilot/selfdrive/ui/ui.py")],
                          cwd=OP_ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def main():
  port = int(os.environ.get("OP_UI_PORT", "8765"))
  init_params()
  tr = Translator()

  server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  server.bind(("127.0.0.1", port))
  server.listen(1)
  server.settimeout(0.25)
  print(f"host_ui: waiting for bukapilot ui_relay on 127.0.0.1:{port}", flush=True)

  ui = start_ui()
  stop = False

  def on_signal(*_):
    nonlocal stop
    stop = True
  signal.signal(signal.SIGTERM, on_signal)
  signal.signal(signal.SIGINT, on_signal)

  try:
    while not stop and ui.poll() is None:
      try:
        conn, _ = server.accept()
      except socket.timeout:
        tr.send_slow()  # keep the UI onroad-ready while bukapilot boots
        continue
      print("host_ui: relay connected", flush=True)
      conn.settimeout(5.0)
      try:
        while not stop and ui.poll() is None:
          json_len, blob_len = HEADER.unpack(recv_exact(conn, HEADER.size))
          payload = json.loads(bytes(recv_exact(conn, json_len)))
          frame = np.frombuffer(recv_exact(conn, blob_len), dtype=np.uint8) if blob_len else None
          tr.publish(payload, frame)
      except (ConnectionError, socket.timeout, OSError) as e:
        print(f"host_ui: relay lost ({e}), waiting again", flush=True)
      finally:
        conn.close()
  finally:
    if ui.poll() is None:
      ui.terminate()
      try:
        ui.wait(5)
      except subprocess.TimeoutExpired:
        ui.kill()
    server.close()


if __name__ == "__main__":
  main()
