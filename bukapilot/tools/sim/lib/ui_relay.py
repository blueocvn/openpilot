#!/usr/bin/env python3
"""Relay bukapilot state to the openpilot UI running on the host.

Runs inside the bukapilot container (Python 3.8, cereal 0.8.13). Reads the
topics and road camera the bukapilot Qt UI would draw, flattens them to plain
JSON plus the raw I420 frame, and streams them over TCP to
op_ui/host_ui.py (openpilot's Python 3.12 venv), which republishes them in
the current openpilot schema for openpilot/selfdrive/ui.

Wire format per frame: 4-byte big-endian JSON length, 4-byte big-endian frame
length, JSON, I420 frame bytes.
"""
import json
import os
import socket
import struct
import time

import cereal.messaging as messaging
from cereal.visionipc.visionipc_pyx import VisionIpcClient, VisionStreamType  # pylint: disable=no-name-in-module, import-error

HEADER = struct.Struct(">II")


def xyz(d):
  return {"x": list(d.x), "y": list(d.y), "z": list(d.z)}


def lead(l):
  return {"dRel": l.dRel, "yRel": l.yRel, "vRel": l.vRel, "status": l.status}


def snapshot(sm):
  m = sm['modelV2']
  cs = sm['controlsState']
  car = sm['carState']
  rs = sm['radarState']
  cal = sm['liveCalibration']
  cc = sm['carControl']
  dp = m.meta.disengagePredictions
  return {
    "updated": {k: bool(sm.updated[k]) for k in sm.updated},
    "alive": {k: bool(sm.alive[k]) for k in sm.alive},
    "modelV2": {
      "frameId": m.frameId,
      "position": xyz(m.position),
      "laneLines": [xyz(l) for l in m.laneLines],
      "laneLineProbs": list(m.laneLineProbs),
      "roadEdges": [xyz(e) for e in m.roadEdges],
      "roadEdgeStds": list(m.roadEdgeStds),
      "accelerationX": list(m.acceleration.x),
      "brakeDisengageProbs": list(dp.brakeDisengageProbs),
      "steerOverrideProbs": list(dp.steerOverrideProbs),
    },
    "controlsState": {
      "enabled": cs.enabled,
      "active": cs.active,
      "engageable": cs.engageable,
      "state": str(cs.state),
      "alertText1": cs.alertText1,
      "alertText2": cs.alertText2,
      "alertSize": str(cs.alertSize),
      "alertStatus": str(cs.alertStatus),
      "alertType": cs.alertType,
      "vCruise": cs.vCruise,
      "curvature": cs.curvature,
    },
    "carState": {
      "vEgo": car.vEgo,
      "vEgoCluster": car.vEgoCluster,
      "aEgo": car.aEgo,
      "standstill": car.standstill,
      "steeringAngleDeg": car.steeringAngleDeg,
      "steeringTorque": car.steeringTorque,
      "steeringPressed": car.steeringPressed,
      "leftBlinker": car.leftBlinker,
      "rightBlinker": car.rightBlinker,
      "cruiseSpeed": car.cruiseState.speed,
      "cruiseEnabled": car.cruiseState.enabled,
      "cruiseAvailable": car.cruiseState.available,
    },
    "radarState": {"leadOne": lead(rs.leadOne), "leadTwo": lead(rs.leadTwo)},
    "liveCalibration": {"calStatus": cal.calStatus, "rpyCalib": list(cal.rpyCalib)},
    "carControl": {"enabled": cc.enabled, "active": cc.active, "steer": cc.actuators.steer},
    "carParams": {"openpilotLongitudinalControl": sm['carParams'].openpilotLongitudinalControl,
                  "carFingerprint": sm['carParams'].carFingerprint},
  }


def connect(host, port):
  while True:
    try:
      s = socket.create_connection((host, port), timeout=5)
      s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
      print(f"ui_relay: connected to openpilot UI host at {host}:{port}", flush=True)
      return s
    except OSError:
      time.sleep(1)


def main():
  host = os.environ.get("OP_UI_HOST", "127.0.0.1")
  port = int(os.environ.get("OP_UI_PORT", "8765"))

  sm = messaging.SubMaster(['modelV2', 'controlsState', 'carState', 'radarState', 'liveCalibration',
                            'carControl', 'carParams'])
  vipc = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
  sock = connect(host, port)

  while True:
    if not vipc.is_connected():
      vipc.connect(False)
      if not vipc.is_connected():
        time.sleep(0.1)
        continue
    frame = vipc.recv(100)
    sm.update(0)
    payload = snapshot(sm)
    blob = b""
    if frame is not None:
      payload["frame"] = {"width": vipc.width, "height": vipc.height}
      blob = frame.tobytes()
    data = json.dumps(payload).encode()
    try:
      sock.sendall(HEADER.pack(len(data), len(blob)) + data)
      if blob:
        sock.sendall(blob)
    except OSError:
      print("ui_relay: host disconnected, reconnecting", flush=True)
      sock.close()
      sock = connect(host, port)


if __name__ == "__main__":
  main()
