#!/usr/bin/env python3
# =============================================================
# build_simple_drone.py  (docs/PLAN.md 2-1)
#
# config/drone_simple.yaml → 기본 도형 간이 드론 USD (단일 강체). 치수·질량을 바꾸면 다시 빌드한다.
#   몸체 박스 + 팔 4 + 모터 4 + 로터 원판 4 + 다리 4. 모든 도형이 충돌 형상이자 시각 형상 (도형별 질량)
#   물체 재질·씬 위치는 넣지 않는다 (씬 레이어에서 drone.add_drone 이 정함)
#
# 실행:
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/build_simple_drone.py
# =============================================================

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)

parser = argparse.ArgumentParser(description="간이 드론 USD 빌드 (PLAN 2-1)")
parser.add_argument("--config", default=os.path.join(SIM_ROOT, "config/drone_simple.yaml"))
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})   # pxr 는 app 시작 후에만 import 가능

import yaml  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

KEYS = {"body": ("size", "color"),
        "arms": ("motor_x", "motor_y", "inset", "z", "width", "thickness", "mass", "color"),
        "motors": ("radius", "height", "mass", "color"),
        "rotors": ("radius", "thickness", "gap", "mass", "color"),
        "legs": ("radius", "clearance", "mass", "color")}
CORNERS = ((1, 1), (-1, 1), (-1, -1), (1, -1))   # (x 부호, y 부호): 앞왼 0, 뒤왼 1, 뒤오른 2, 앞오른 3


def load(path):
    with open(path, encoding="utf-8") as f:
        c = yaml.safe_load(f)
    for k in ("usd", "name", "mass", "grasp"):
        if k not in c:
            raise KeyError(f"{path}: '{k}' 항목이 없습니다")
    for sec, keys in KEYS.items():
        for k in keys:
            if k not in c.get(sec, {}):
                raise KeyError(f"{path}: '{sec}.{k}' 항목이 없습니다")
    return c


def check_geometry(c):
    L, W, H = c["body"]["size"]
    a, m, r = c["arms"], c["motors"], c["rotors"]
    if not (abs(a["z"]) + a["thickness"] / 2 <= H / 2):
        raise ValueError("팔이 몸체 높이 밖에서 시작함")
    if a["motor_x"] <= L / 2 - a["inset"] or a["motor_y"] <= W / 2:
        raise ValueError("모터가 몸체 안쪽에 있음 (팔 길이 0 이하)")
    # 로터 원판끼리 겹치지 않는지 (앞뒤·좌우 이웃)
    if 2 * r["radius"] >= min(2 * a["motor_x"], 2 * a["motor_y"]):
        raise ValueError("로터 원판이 서로 겹침")
    rotor_z = a["z"] + m["height"] + r["gap"]
    if rotor_z - r["thickness"] / 2 <= H / 2 and a["motor_y"] - r["radius"] < W / 2:
        raise ValueError("로터 원판이 몸체와 겹침")
    rest = c["mass"] - 4 * (a["mass"] + m["mass"] + r["mass"] + c["legs"]["mass"])
    if rest <= 0:
        raise ValueError(f"몸체 질량이 0 이하: {rest}")
    return rest


def collider(prim, mass, color, gprim):
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(float(mass))
    gprim.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])


def add_box(stage, path, size, pos, yaw_deg, mass, color):
    g = UsdGeom.Cube.Define(stage, path)
    g.CreateSizeAttr().Set(1.0)
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddRotateZOp().Set(float(yaw_deg))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    collider(g.GetPrim(), mass, color, g)


def add_cyl(stage, path, radius, height, pos, mass, color):
    g = UsdGeom.Cylinder.Define(stage, path)
    g.CreateAxisAttr().Set("Z")
    g.CreateRadiusAttr().Set(float(radius))
    g.CreateHeightAttr().Set(float(height))
    g.CreateExtentAttr().Set([Gf.Vec3f(-radius, -radius, -height / 2), Gf.Vec3f(radius, radius, height / 2)])
    UsdGeom.Xformable(g).AddTranslateOp().Set(Gf.Vec3d(*pos))
    collider(g.GetPrim(), mass, color, g)


def build(c, out):
    body_mass = check_geometry(c)
    L, W, H = c["body"]["size"]
    a, m, r, lg = c["arms"], c["motors"], c["rotors"], c["legs"]

    os.makedirs(os.path.dirname(out), exist_ok=True)
    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{c['name']}")
    stage.SetDefaultPrim(root.GetPrim())
    rp = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(rp)
    PhysxSchema.PhysxRigidBodyAPI.Apply(rp).CreateSleepThresholdAttr().Set(0.0)   # 공중에 떠 있는 동안 잠들지 않게
    rp.SetCustomDataByKey("source_config", os.path.relpath(args.config, SIM_ROOT))

    base = f"/{c['name']}"
    add_box(stage, f"{base}/body", (L, W, H), (0, 0, 0), 0.0, body_mass, c["body"]["color"])
    rotor_z = a["z"] + m["height"] + r["gap"] + r["thickness"] / 2
    leg_top, leg_bot = a["z"] - a["thickness"] / 2, -H / 2 - lg["clearance"]
    for i, (sx, sy) in enumerate(CORNERS):
        x0, y0 = sx * (L / 2 - a["inset"]), sy * W / 2
        x1, y1 = sx * a["motor_x"], sy * a["motor_y"]
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.degrees(math.atan2(y1 - y0, x1 - x0))
        add_box(stage, f"{base}/arm{i}", (length, a["width"], a["thickness"]), ((x0 + x1) / 2, (y0 + y1) / 2, a["z"]),
                yaw, a["mass"], a["color"])
        add_cyl(stage, f"{base}/motor{i}", m["radius"], m["height"], (x1, y1, a["z"] + m["height"] / 2),
                m["mass"], m["color"])
        add_cyl(stage, f"{base}/rotor{i}", r["radius"], r["thickness"], (x1, y1, rotor_z), r["mass"], r["color"])
        add_cyl(stage, f"{base}/leg{i}", lg["radius"], leg_top - leg_bot, (x1, y1, (leg_top + leg_bot) / 2),
                lg["mass"], lg["color"])
    stage.GetRootLayer().customLayerData = {"generated_by": "isaacsim/scripts/build_simple_drone.py",
                                            "config": os.path.relpath(args.config, SIM_ROOT)}
    stage.GetRootLayer().Save()
    return body_mass, rotor_z, leg_bot


def main():
    c = load(args.config)
    out = c["usd"] if os.path.isabs(c["usd"]) else os.path.join(SIM_ROOT, c["usd"])
    body_mass, rotor_z, leg_bot = build(c, out)
    L, W, H = c["body"]["size"]
    print(f"[build_simple_drone] {out}", flush=True)
    print(f"  몸체 {L * 1000:.0f} × {W * 1000:.0f} × {H * 1000:.0f} mm, {body_mass:.3f} kg / 총 {c['mass']} kg", flush=True)
    print(f"  모터 (±{c['arms']['motor_x'] * 1000:.0f}, ±{c['arms']['motor_y'] * 1000:.0f}) mm, "
          f"로터 Ø{c['rotors']['radius'] * 2000:.0f} mm (z {rotor_z * 1000:.0f} mm), 다리 끝 z {leg_bot * 1000:.0f} mm", flush=True)


if __name__ == "__main__":
    code = 0
    try:
        main()
    except Exception as e:
        print(f"[ERROR] {type(e).__name__}: {e}", flush=True)
        code = 1
    finally:
        simulation_app.close(exit_code=code)
    sys.exit(code)
