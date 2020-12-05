#!/usr/bin/env python3
from cereal import car
from common.params import Params
from common.realtime import Priority, config_realtime_process
from common.op_params import opParams, ENABLE_COASTING
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.planner import Planner
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.pathplanner import PathPlanner
import cereal.messaging as messaging


def plannerd_thread(sm=None, pm=None):

  config_realtime_process(2, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  OP = opParams()
  PL = Planner(CP, OP=OP)
  PP = PathPlanner(CP)

  VM = VehicleModel(CP)

  if sm is None:
    # TODO should modelV2 be polled?
    sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'radarState', 'model', 'liveParameters', 'modelV2'],
                             poll=['radarState', 'model'])

  if pm is None:
    pm = messaging.PubMaster(['plan', 'liveLongitudinalMpc', 'pathPlan', 'liveMpc'])

  sm['liveParameters'].valid = True
  sm['liveParameters'].sensorValid = True
  sm['liveParameters'].steerRatio = CP.steerRatio
  sm['liveParameters'].stiffnessFactor = 1.0

  while True:
    sm.update()

    if sm.updated['model']:
      PP.update(sm, pm, CP, VM)
    if sm.updated['radarState'] or (OP.get(ENABLE_COASTING) and sm.updated['modelV2']):
      PL.update(sm, pm, CP, VM, PP) # TODO look into whether this should run when the model updates too


def main(sm=None, pm=None):
  plannerd_thread(sm, pm)


if __name__ == "__main__":
  main()
