#!/usr/bin/env python3
# =============================================================
# build_stewart_usd.py  (docs/PLAN.md 8단계 S3, 움직이는 베이스: Stewart 플랫폼)
#
# config/base_stewart.yaml (design_stewart.py 로 정한 크기) → Stewart 플랫폼 USD 에셋. 실물이 없는 sim 전용, 기본 도형만.
#   최상위 prim /stewart = 바닥 가운데 (z 0 = 바닥, x = 로봇 정면, z 위)
#   base_plate : 아랫판 원기둥 (정적 collider, 움직이지 않음)
#   top_plate  : 윗판 = kinematic 강체. prim 원점 = 회전 중심 = 윗판 윗면 가운데 (= 로봇 base 자리, 가운데 자세 z = height)
#                └ geom (원기둥, 충돌), imu_link (윗판 아랫면 가운데, 물리 없음), joint_1 … 6 (윗판 쪽 관절 구, 보이기만)
#   legs/leg_i : 보이기만 (충돌·질량 없음). cylinder (아랫판 부착점에서 다리 방향), rod (윗판 부착점에서 아래로).
#                가운데 자세로 놓아 둔다. 움직일 때의 길이·방향은 moving_base.py (S4) 가 역기구학으로 갱신
#   base_joint_1 … 6 : 아랫판 쪽 관절 구 (보이기만)
#   부착 각도 = daniel-s-ingram/stewart (MIT, f276d049) src/ik.cpp. 씬 위치·로봇 연결은 넣지 않는다 (moving_base_scene.py)
#
# 실행 (레포 최상위):
#   ~/isaacsim/python.sh isaacsim/scripts/build_stewart_usd.py --config isaacsim/config/base_stewart.yaml
# 결과: isaacsim/assets/bases/stewart/stewart.usda (손으로 고치지 않는다, 설정을 바꾸고 다시 빌드)
# =============================================================

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

parser = argparse.ArgumentParser(description="Stewart 플랫폼 USD 빌드 (PLAN 8단계 S3)")
parser.add_argument("--config", default=os.path.join(SIM_ROOT, "config/base_stewart.yaml"))
parser.add_argument("--out", default=os.path.join(SIM_ROOT, "assets/bases/stewart/stewart.usda"))
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})   # pxr 는 app 시작 후에만 import 가능

import numpy as np  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

import design_stewart as dst  # noqa: E402   (부착점·회전 식을 같은 코드로)

COLORS = {"base": (0.35, 0.35, 0.37), "top": (0.25, 0.42, 0.62), "cylinder": (0.15, 0.15, 0.16), "rod": (0.75, 0.76, 0.78),
          "joint": (0.10, 0.10, 0.10)}


def quat_z_to(d):
    """+z 를 단위 벡터 d 로 돌리는 쿼터니언 (w, x, y, z)."""
    d = d / np.linalg.norm(d)
    z = np.array([0.0, 0.0, 1.0])
    c = float(z @ d)
    if c < -1 + 1e-9:
        return (0.0, 1.0, 0.0, 0.0)
    ax = np.cross(z, d)
    w = math.sqrt((1 + c) / 2)
    return (w, *(ax / (2 * w)))


def xform(stage, path, t=(0, 0, 0), q=(1, 0, 0, 0)):
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, t)))
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[0]), Gf.Vec3d(*map(float, q[1:]))))
    return x


def cylinder(stage, path, radius, height, z_center, color, collision=False):
    g = UsdGeom.Cylinder.Define(stage, path)
    g.CreateAxisAttr("Z")
    g.CreateRadiusAttr(float(radius))
    g.CreateHeightAttr(float(height))
    g.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -height / 2), Gf.Vec3f(radius, radius, height / 2)])
    UsdGeom.Xformable(g).AddTranslateOp().Set(Gf.Vec3d(0, 0, float(z_center)))
    g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    return g


def sphere(stage, path, center, radius, color):
    s = UsdGeom.Sphere.Define(stage, path)
    s.CreateRadiusAttr(float(radius))
    s.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -radius), Gf.Vec3f(radius, radius, radius)])
    UsdGeom.Xformable(s).AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    s.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return s


def leg_parts(c):
    """실린더·로드 길이와 확인. 실린더는 아랫판 부착점에서, 로드는 윗판 부착점에서 서로를 향해 뻗는다."""
    lg = c["legs"]
    for k in ("length_min", "length_max", "cylinder_radius", "rod_radius", "overlap", "joint_radius"):
        if lg.get(k) is None:
            raise KeyError(f"legs.{k} 가 없음 (design_stewart.py 결과로 채울 것)")
    lmin, lmax, ov = float(lg["length_min"]), float(lg["length_max"]), float(lg["overlap"])
    part = (lmax + ov) / 2                  # 가장 길 때 겹침 = ov
    clear = lmin - part                     # 가장 짧을 때 로드 끝 ↔ 아랫판 부착점 (실린더 끝 ↔ 윗판 부착점) 여유
    if clear < 0:
        raise ValueError(f"다리 신축 비율이 너무 큼: length_min {lmin} < 실린더·로드 길이 {part:.3f} "
                         f"(length_max {lmax} + overlap {ov}) / 2 → 가장 짧을 때 로드가 실린더 밖으로 나옴")
    if float(lg["rod_radius"]) >= float(lg["cylinder_radius"]):
        raise ValueError("rod_radius 는 cylinder_radius 보다 작아야 함 (로드가 실린더 안으로 들어감)")
    return part, clear


def main():
    c = dst.load(args.config)
    g = c["geometry"]
    H = float(g["height"])
    B, P = dst.anchors(g)                   # B: world (아랫판 윗면), P: 윗판 좌표 (회전 중심 기준, 윗판 아랫면)
    tb, tt = float(g["base_plate"]["thickness"]), float(g["top_plate"]["thickness"])
    if float(g["base_radius"]) + float(c["legs"]["joint_radius"]) > float(g["base_plate"]["radius"]):
        raise ValueError("아랫판이 부착점 (관절 구 포함) 보다 작음")
    if float(g["top_radius"]) + float(c["legs"]["joint_radius"]) > float(g["top_plate"]["radius"]):
        raise ValueError("윗판이 부착점 (관절 구 포함) 보다 작음")
    if abs(float(c["imu"]["offset"][2]) + tt) > 1e-9 or any(abs(float(v)) > 1e-9 for v in c["imu"]["offset"][:2]):
        raise ValueError(f"imu.offset 은 윗판 아랫면 가운데 (0, 0, −top_plate.thickness = {-tt}) 여야 함: {c['imu']['offset']}")
    part, clear = leg_parts(c)
    top_w = P + [0, 0, H]                   # 가운데 자세의 윗판 부착점 (world)
    L0 = np.linalg.norm(top_w - B, axis=1)
    lg = c["legs"]
    if not (float(lg["length_min"]) <= L0.min() and L0.max() <= float(lg["length_max"])):
        raise ValueError(f"가운데 자세 다리 길이 {L0.round(3).tolist()} 가 legs 범위 밖")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    stage = Usd.Stage.CreateNew(args.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/stewart")
    stage.SetDefaultPrim(root.GetPrim())

    # 아랫판: 정적 collider
    UsdGeom.Xform.Define(stage, "/stewart/base_plate")
    cylinder(stage, "/stewart/base_plate/geom", g["base_plate"]["radius"], tb, tb / 2, COLORS["base"], collision=True)
    for i in range(6):
        sphere(stage, f"/stewart/base_plate/joint_{i + 1}", B[i], lg["joint_radius"], COLORS["joint"])

    # 윗판: kinematic 강체, 원점 = 회전 중심 (윗판 윗면 가운데)
    top = xform(stage, "/stewart/top_plate", (0, 0, H))
    UsdPhysics.RigidBodyAPI.Apply(top.GetPrim()).CreateKinematicEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(top.GetPrim()).CreateMassAttr(50.0)        # kinematic 이라 움직임에는 쓰이지 않음 (표시용)
    cylinder(stage, "/stewart/top_plate/geom", g["top_plate"]["radius"], tt, -tt / 2, COLORS["top"], collision=True)
    xform(stage, "/stewart/top_plate/imu_link", c["imu"]["offset"])
    for i in range(6):
        sphere(stage, f"/stewart/top_plate/joint_{i + 1}", P[i], lg["joint_radius"], COLORS["joint"])

    # 다리: 보이기만. leg_i/cylinder (아랫판 부착점, +z = 다리 방향), leg_i/rod (윗판 부착점, +z = 아래 (아랫판 쪽))
    UsdGeom.Scope.Define(stage, "/stewart/legs")
    for i in range(6):
        d = (top_w[i] - B[i]) / L0[i]
        xform(stage, f"/stewart/legs/leg_{i + 1}")
        xform(stage, f"/stewart/legs/leg_{i + 1}/cylinder", B[i], quat_z_to(d))
        cylinder(stage, f"/stewart/legs/leg_{i + 1}/cylinder/geom", lg["cylinder_radius"], part, part / 2, COLORS["cylinder"])
        xform(stage, f"/stewart/legs/leg_{i + 1}/rod", top_w[i], quat_z_to(-d))
        cylinder(stage, f"/stewart/legs/leg_{i + 1}/rod/geom", lg["rod_radius"], part, part / 2, COLORS["rod"])

    stage.GetRootLayer().customLayerData = {
        "generated_by": "isaacsim/scripts/build_stewart_usd.py",
        "config": os.path.relpath(args.config, SIM_ROOT),
        "anchor_angles_source": "daniel-s-ingram/stewart (MIT) f276d049 src/ik.cpp",
        "height": H, "base_radius": float(g["base_radius"]), "top_radius": float(g["top_radius"]),
        "leg_length_min": float(lg["length_min"]), "leg_length_max": float(lg["length_max"]), "leg_part_length": float(part)}
    stage.GetRootLayer().Save()

    # 다시 열어 확인
    s2 = Usd.Stage.Open(args.out)
    tp = s2.GetPrimAtPath("/stewart/top_plate")
    n_coll = sum(1 for p in s2.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI))
    n_rb = sum(1 for p in s2.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI))
    if not (tp.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(tp).GetKinematicEnabledAttr().Get()):
        raise RuntimeError("윗판이 kinematic 강체가 아님")
    if n_coll != 2 or n_rb != 1:
        raise RuntimeError(f"충돌 형상 {n_coll} 개 (기대 2: 아랫판·윗판), 강체 {n_rb} 개 (기대 1: 윗판)")
    print(f"[build_stewart_usd] {args.out} ({os.path.getsize(args.out) / 1e3:.1f} kB)", flush=True)
    print(f"  윗판 회전 중심 z {H} m, 아랫판 r {g['base_plate']['radius']} / 윗판 r {g['top_plate']['radius']} m, "
          f"부착 반지름 {g['base_radius']} / {g['top_radius']} m", flush=True)
    print(f"  다리 6 개: 가운데 자세 {L0.mean():.3f} m (범위 {lg['length_min']} ~ {lg['length_max']}), 실린더·로드 각 {part:.4f} m, "
          f"가장 짧을 때 여유 {clear * 1000:.1f} mm, 가장 길 때 겹침 {float(lg['overlap']) * 1000:.0f} mm", flush=True)
    print(f"  imu_link = 윗판 아랫면 가운데 (회전 중심 기준 {c['imu']['offset']}), 강체 {n_rb} (윗판 kinematic), 충돌 {n_coll} (아랫판·윗판)", flush=True)


if __name__ == "__main__":
    code = 0
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"[ERROR] {type(e).__name__}: {e}", flush=True)
        code = 1
    finally:
        simulation_app.close(exit_code=code)
    sys.exit(code)
