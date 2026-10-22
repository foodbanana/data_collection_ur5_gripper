#!/usr/bin/env python3
# =============================================================
# build_d455_usd.py  (docs/PLAN.md 6단계 "수집 전 확인", third view 카메라 모양)
#
# realsense2_description (ROS 2 Jazzy, Apache-2.0) 의 D455 메시·프레임 → third view 카메라 USD 에셋.
#   실물 third view 는 D456 이지만 메시가 없어 외형이 같은 D455 (124 × 29 × 26 mm) 를 쓴다.
#   최상위 prim = 아랫면 1/4-20 나사 자리 (_d455.urdf.xacro 의 bottom_screw_frame, x 앞·y 왼쪽·z 위).
#   그 아래: visual (메시, 보이기만), collision (몸체 상자, 정적 collider, 렌더 안 함), color_optical_frame (RGB 렌즈, +z 앞·+y 아래·+x 오른쪽)
#   프레임 값은 xacro 에서 읽는다 (손으로 옮겨 적지 않음). 씬 위치·받침대는 넣지 않는다 (drone_scene.add_third_view_camera)
#
# 실행:
#   source /opt/ros/jazzy/setup.bash   (realsense2_description 위치)
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/build_d455_usd.py
# 결과: isaacsim/assets/cameras/d455/d455.usdc
# =============================================================

import argparse
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)
PKG = "/opt/ros/jazzy/share/realsense2_description"

parser = argparse.ArgumentParser(description="D455 third view 카메라 USD 빌드")
parser.add_argument("--pkg", default=PKG, help="realsense2_description 위치")
parser.add_argument("--out", default=os.path.join(SIM_ROOT, "assets/cameras/d455/d455.usdc"))
parser.add_argument("--color", type=float, nargs=3, default=[0.18, 0.18, 0.19], help="몸체 색 (linear)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})   # pxr 는 app 시작 후에만 import 가능

import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt  # noqa: E402

PROPS = ("d455_cam_depth_to_color_offset", "d455_cam_width", "d455_cam_height", "d455_cam_depth",
         "d455_cam_mount_from_center_offset", "d455_glass_to_front", "d455_zero_depth_to_glass", "d455_cam_depth_py")
SIZE_TOL = 0.003      # 메시 외곽 ↔ xacro 몸체 치수 허용 차 [m]


def rpy(r, p, y):
    """URDF rpy (고정축 X → Y → Z) 회전 행렬."""
    cx, sx, cy, sy, cz, sz = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def read_props(path):
    txt = open(path, encoding="utf-8").read()
    raw = dict(re.findall(r'<xacro:property name="(d455_[a-z_0-9]+)" value="([^"]+)"', txt))
    val = {}

    def ev(k):
        if k not in val:
            expr = raw[k].strip()
            m = re.fullmatch(r"\$\{(.+)\}", expr)
            expr = m.group(1) if m else expr
            for name in sorted(set(re.findall(r"d455_[a-z_0-9]+", expr)), key=len, reverse=True):
                expr = expr.replace(name, repr(ev(name)))
            val[k] = float(eval(expr, {"__builtins__": {}}))
        return val[k]

    for k in PROPS + ("d455_mesh_x_offset", "d455_cam_depth_pz"):
        if k not in raw:
            raise KeyError(f"{path}: xacro property {k} 없음 (realsense2_description 버전 확인)")
        ev(k)
    return val


def main():
    xacro = os.path.join(args.pkg, "urdf/_d455.urdf.xacro")
    stl = os.path.join(args.pkg, "meshes/d455.stl")
    for p in (xacro, stl):
        if not os.path.isfile(p):
            raise FileNotFoundError(f"{p} 없음 (source /opt/ros/jazzy/setup.bash, ros-jazzy-realsense2-description)")
    v = read_props(xacro)
    # bottom_screw_frame → camera_link (xacro: d455_mesh_x_offset, d455_cam_depth_py, d455_cam_depth_pz)
    t_link = np.array([v["d455_mesh_x_offset"], v["d455_cam_depth_py"], v["d455_cam_depth_pz"]])
    # camera_link → visual 메시 (xacro: origin xyz = (zero_depth_to_glass + glass_to_front, −depth_py, 0), rpy = (π/2, 0, π/2), scale 0.001)
    t_vis = np.array([v["d455_zero_depth_to_glass"] + v["d455_glass_to_front"], -v["d455_cam_depth_py"], 0.0])
    R_vis = rpy(math.pi / 2, 0.0, math.pi / 2)
    # camera_link → color_frame (0, depth_to_color, 0) → color_optical_frame rpy (−π/2, 0, −π/2)
    t_opt = t_link + np.array([0.0, v["d455_cam_depth_to_color_offset"], 0.0])
    R_opt = rpy(-math.pi / 2, 0.0, -math.pi / 2)
    if not (np.allclose(R_opt[:, 2], [1, 0, 0]) and np.allclose(R_opt[:, 1], [0, 0, -1])):
        raise RuntimeError("optical frame 축이 예상 (+z = 앞, +y = 아래) 과 다름")

    mesh = trimesh.load(stl, force="mesh")
    V = (np.asarray(mesh.vertices) * 0.001) @ R_vis.T + t_vis + t_link     # 나사 자리 좌표
    F = np.asarray(mesh.faces)
    lo, hi = V.min(0), V.max(0)
    size = hi - lo
    want = np.array([v["d455_cam_depth"], v["d455_cam_width"], v["d455_cam_height"]])
    if np.any(np.abs(size - want) > SIZE_TOL) or abs(lo[2]) > SIZE_TOL:
        raise RuntimeError(f"메시 외곽 {np.round(size * 1000, 1).tolist()} mm (아랫면 z {lo[2] * 1000:.1f}) ↔ xacro 몸체 "
                           f"{np.round(want * 1000, 1).tolist()} mm: 변환이 틀렸거나 메시가 다름")
    bottom_center = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    if os.path.exists(args.out):
        os.remove(args.out)
    stage = Usd.Stage.CreateNew(args.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/d455")
    stage.SetDefaultPrim(root.GetPrim())
    g = UsdGeom.Mesh.Define(stage, "/d455/visual")
    g.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(V.astype(np.float32)))
    g.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, dtype=np.int32)))
    g.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.reshape(-1).astype(np.int32)))
    g.CreateSubdivisionSchemeAttr("none")
    g.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])
    g.CreateDisplayColorAttr([Gf.Vec3f(*args.color)])
    # 충돌: 몸체 상자 (정적 collider, 렌더 안 함)
    c = UsdGeom.Cube.Define(stage, "/d455/collision")
    c.CreateSizeAttr(1.0)
    cx = UsdGeom.Xformable(c)
    cx.AddTranslateOp().Set(Gf.Vec3d(*map(float, (lo + hi) / 2)))
    cx.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    c.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    # RGB 렌즈
    o = UsdGeom.Xform.Define(stage, "/d455/color_optical_frame")
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = R_opt, t_opt
    UsdGeom.Xformable(o).AddTransformOp().Set(Gf.Matrix4d(M.T.tolist()))
    stage.GetRootLayer().customLayerData = {
        "generated_by": "isaacsim/scripts/build_d455_usd.py",
        "source": "realsense2_description (ROS 2 Jazzy, Apache-2.0): meshes/d455.stl, urdf/_d455.urdf.xacro",
        "root_frame": "bottom_screw_frame (1/4-20, bottom face)",
        "bottom_center": Gf.Vec3d(*map(float, bottom_center))}
    stage.GetRootLayer().Save()

    print(f"[build_d455_usd] {args.out} ({os.path.getsize(args.out) / 1e6:.2f} MB, 삼각형 {len(F)})", flush=True)
    print(f"  몸체 {np.round(size * 1000, 1).tolist()} mm (xacro {np.round(want * 1000, 1).tolist()}), "
          f"나사 자리 기준 x {lo[0] * 1000:.1f} ~ {hi[0] * 1000:.1f}, y {lo[1] * 1000:.1f} ~ {hi[1] * 1000:.1f}, z {lo[2] * 1000:.1f} ~ {hi[2] * 1000:.1f} mm", flush=True)
    print(f"  color_optical_frame (나사 자리 기준) {np.round(t_opt * 1000, 2).tolist()} mm, "
          f"bottom_center {np.round(bottom_center, 4).tolist()} m (third_view_camera.yaml model.bottom_center)", flush=True)


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
