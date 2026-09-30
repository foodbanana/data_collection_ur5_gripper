#!/usr/bin/env python3
# =============================================================
# check_wrist_camera.py  (docs/PLAN.md 1-7)
#
# 손목 D435i color 카메라(isaacsim/config/wrist_camera.yaml)를 씬 레이어에 만들고 영상을 확인한다. 로봇 USD 원본은 그대로.
#   1. intrinsics 확인: 카메라 좌표계의 알려진 위치에 작은 구를 두고, 렌더링된 픽셀 중심과 OpenCV 투영(K)이 1 px 안에서 같은지
#   2. tilt 비교: xacro cam_tilt 를 흉내 내 Camera prim 자세만 바꿔 (카메라 몸체·충돌 형상은 그대로) 같은 장면을 찍는다
#      장면: 홈 자세 / 파지 기준 자세(grasp_tests 와 같음, G2 박스가 손가락 사이) × 그리퍼 열림 / 닫힘
#      숫자: 화면에서 오른쪽·왼쪽 손가락, 박스가 차지하는 비율 %, TCP 의 영상 좌표 (u, v)
#   영상은 isaacsim/reports/wrist_camera_<시각>/ 에 PNG 로 저장 (tilt 별 한 장씩 모은 그림도)
#
# 실행:
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_wrist_camera.py --headless
#   tilt 고르기: --tilts 0 10 15 20   (degree, 양수 = 공구 축 쪽으로 기울임)
#   GUI 로 손목 시점 보기: --headless 빼고 --hold --tilts 0  (끝나면 손목 카메라 창이 추가로 뜸, 창을 닫으면 종료)
# =============================================================

import argparse
import datetime
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

parser = argparse.ArgumentParser(description="손목 카메라 intrinsics 확인과 cam_tilt 비교 (PLAN 1-7)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--usd", default=ts.DEFAULT_USD)
parser.add_argument("--physics-variant", default="physx")
parser.add_argument("--config", default=ts.DEFAULT_CONFIG)
parser.add_argument("--camera-config", default=ts.WRIST_CAMERA_CONFIG)
parser.add_argument("--tilts", type=float, nargs="+", default=[0.0, 10.0, 15.0, 20.0], help="비교할 cam_tilt [degree]")
parser.add_argument("--hold", action="store_true",
                    help="GUI: 시험 뒤 손목 카메라 뷰포트 창을 하나 더 띄우고 창을 닫을 때까지 유지 (메인 뷰포트는 그대로, 파지 자세·박스·그리퍼 닫힘, 첫 번째 tilt)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.semantics as sem_utils  # noqa: E402
import numpy as np  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

import collision_geom as cg  # noqa: E402
from robot_drive import RobotDrive  # noqa: E402

GRIP_CLOSED = 1.1351
# 파지 기준 자세 = grasp_tests.arm_at(0) (공구가 아래를 향함). G2 박스(160 × 60 × 100 mm) 윗면은 그리퍼 base 에서 z 103.9 mm
# (grasp_tests K1 기하 계산, 2026-09-30) → 박스 중심 z = 103.9 + 50 mm
GRASP_POSE = [0.0, -1.0352, 1.9697, -2.5053, -1.5708, 0.0]
BOX = (0.16, 0.06, 0.10)
BOX_CENTER_Z = 0.1039 + BOX[2] / 2
TCP_Z = 0.11                           # rh_p12_rn_tcp (그리퍼 base z 0.11, URDF)
BOX_PATH = "/World/CameraCheck/box"
MARKERS = {"marker_a": (0.0, 0.0, 0.40), "marker_b": (0.10, -0.06, 0.35), "marker_c": (-0.12, 0.08, 0.30)}  # optical 좌표 [m]
LABELS = {"finger_r": "rh_p12_rn_r1", "finger_l": "rh_p12_rn_l1"}   # r1 에 붙인 라벨은 자식 r2 에도 적용
SETTLE_RENDER = 8                      # 자세를 바꾼 뒤 렌더링을 기다리는 app update 수


def mat_np(m):
    """Gf 행렬(행벡터 규약) → numpy 4x4 (열벡터 규약)."""
    return np.array([[m[i][j] for j in range(4)] for i in range(4)]).T


def build(cfg, cam_cfg):
    stage, robot, info = ts.build_test_scene(args.usd, args.physics_variant, base="fixed", config=cfg, light=True)
    cam_path = ts.add_wrist_camera(stage, cam_cfg, math.radians(args.tilts[0]))
    for label, link in LABELS.items():
        sem_utils.add_labels(ts.find_link_path(stage, link), labels=label)
    UsdGeom.Xform.Define(stage, "/World/CameraCheck")
    box = UsdGeom.Xform.Define(stage, BOX_PATH)
    box.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 0.5))
    box.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))
    UsdPhysics.RigidBodyAPI.Apply(box.GetPrim()).CreateKinematicEnabledAttr().Set(True)
    geom = UsdGeom.Cube.Define(stage, BOX_PATH + "/geom")
    geom.CreateSizeAttr().Set(1.0)
    geom.AddScaleOp().Set(Gf.Vec3f(*BOX))
    geom.CreateDisplayColorAttr().Set([Gf.Vec3f(0.9, 0.45, 0.1)])  # 바닥(파란 격자)과 구분되는 주황
    UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
    ts.bind_object_material(stage, geom.GetPrim())
    sem_utils.add_labels(BOX_PATH, labels="box")
    for name in MARKERS:  # intrinsics 확인용 작은 구 (충돌 없음)
        s = UsdGeom.Sphere.Define(stage, f"/World/CameraCheck/{name}")
        s.CreateRadiusAttr().Set(0.004)
        s.AddTranslateOp().Set(Gf.Vec3d(0, 0, -5))
        s.CreateDisplayColorAttr().Set([Gf.Vec3f(1, 0, 0)])
        sem_utils.add_labels(s.GetPrim(), labels=name)
    w, h = (int(x) for x in cam_cfg["resolution"])
    sensor = CameraSensor(cam_path, resolution=(h, w), annotators=["rgb", "semantic_segmentation"])
    return stage, robot, info, cam_path, sensor


def set_box_pose(stage, box, p, q):
    """kinematic 박스 이동: PhysX(tensor) 와 USD 변환을 둘 다 (tensor 만 바꾸면 물리에는 있지만 렌더링에는 옛 위치로 보임)."""
    box.set_world_poses(positions=np.asarray(p, dtype=float).reshape(1, 3), orientations=np.asarray(q, dtype=float).reshape(1, 4))
    t_op, o_op = UsdGeom.Xformable(stage.GetPrimAtPath(BOX_PATH)).GetOrderedXformOps()[:2]
    t_op.Set(Gf.Vec3d(*[float(x) for x in p]))
    o_op.Set(Gf.Quatf(*[float(x) for x in q]))


def render(sensor, n=SETTLE_RENDER):
    for _ in range(n):
        simulation_app.update()
    for _ in range(200):
        if sensor.has_data():
            break
        simulation_app.update()
    rgb, _ = sensor.get_data("rgb")
    seg, info = sensor.get_data("semantic_segmentation")
    if rgb is None or seg is None:
        raise RuntimeError("카메라 데이터를 받지 못함")
    return rgb.numpy()[..., :3].astype(np.uint8), seg.numpy()[..., 0], info["idToLabels"]


def label_mask(seg, id_to_labels, name):
    ids = [int(k) for k, v in id_to_labels.items() if name in str(v.get("class", "") if isinstance(v, dict) else v)]
    return np.isin(seg, ids)


def optical_T(stage, cam_link_view, cam_path, cam_link_path):
    """현재 (tilt 적용된) optical 좌표계의 world 4x4 (열벡터). Camera prim 은 optical 을 x 축 180° 돌린 것."""
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    rel = mat_np(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(cam_path))
                 * cache.GetLocalToWorldTransform(stage.GetPrimAtPath(cam_link_path)).GetInverse())  # 카메라 링크 → Camera
    p, q = (x.numpy()[0] for x in cam_link_view.get_world_poses())
    T_link = np.eye(4)
    T_link[:3, :3], T_link[:3, 3] = cg.quat_to_R(q), p
    flip = np.diag([1.0, -1.0, -1.0, 1.0])
    return T_link @ rel @ flip


def project(K, T_opt, p_world):
    p = np.linalg.inv(T_opt) @ np.append(p_world, 1.0)
    if p[2] <= 0:
        return None
    return K[0, 0] * p[0] / p[2] + K[0, 2], K[1, 1] * p[1] / p[2] + K[1, 2]


def hold_view(stage, cam_path):
    """메인 뷰포트는 그대로 두고, 손목 카메라 뷰포트 창을 하나 더 띄워 창을 닫을 때까지 유지
    (마지막 장면 = 파지 자세, 박스, 그리퍼 닫힘). 메인 뷰포트의 카메라 메뉴에서 wrist_color_camera 를 골라도 된다."""
    from omni.kit.viewport.utility import create_viewport_window

    ts.set_wrist_camera_tilt(stage, cam_path, math.radians(args.tilts[0]))
    w, h = (int(x) for x in ts.load_camera_config(args.camera_config)["resolution"])
    win = create_viewport_window("Wrist camera", width=w, height=h, camera_path=Sdf.Path(cam_path))
    win.viewport_api.resolution = (w, h)
    print(f"[hold] 'Wrist camera' 창 = 손목 카메라 (tilt {args.tilts[0]:g}°, {w}x{h}). 메인 뷰포트는 그대로. "
          f"Isaac Sim 창을 닫으면 종료합니다.", flush=True)
    while simulation_app.is_running():
        simulation_app.update()


def main():
    lines = []

    def L(m=""):
        print(m, flush=True)
        lines.append(m)

    cfg = ts.load_config(args.config)
    cam_cfg = ts.load_camera_config(args.camera_config)
    k = cam_cfg["intrinsics"]
    K = np.array([[k["fx"], 0, k["cx"]], [0, k["fy"], k["cy"]], [0, 0, 1.0]])
    w, h = (int(x) for x in cam_cfg["resolution"])
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    out_dir = os.path.join(ts.REPORT_DIR, f"wrist_camera_{stamp}")
    os.makedirs(out_dir, exist_ok=True)
    L(f"check_wrist_camera  {stamp}  camera config {os.path.basename(args.camera_config)} (serial {cam_cfg.get('serial')})")
    L(f"  {w}x{h}, fx {k['fx']:.2f} fy {k['fy']:.2f} cx {k['cx']:.2f} cy {k['cy']:.2f}, "
      f"가로 시야각 {math.degrees(2 * math.atan(w / 2 / k['fx'])):.1f}°, 세로 {math.degrees(2 * math.atan(h / 2 / k['fy'])):.1f}°")

    stage, robot, info, cam_path, sensor = build(cfg, cam_cfg)
    app_utils.play()
    simulation_app.update()
    problems, _ = ts.verify_articulation(stage, robot, info, cfg)
    if problems:
        raise RuntimeError(f"articulation 확인 실패: {problems}")
    drive = RobotDrive(robot, cfg)
    drive.apply()
    drive.start()
    cam_link_path = ts.find_link_path(stage, "wrist_camera_link")
    cam_link = RigidPrim(cam_link_path)
    base = RigidPrim(ts.find_link_path(stage, ts.PAYLOAD_LINK))
    box = RigidPrim(BOX_PATH)
    try:
        # ── 1. intrinsics 확인 (첫 tilt, 홈 자세) ──
        drive.reset_pose(cfg["home_pose"], 0.0)
        for _ in range(30):
            simulation_app.update()
        T_opt = optical_T(stage, cam_link, cam_path, cam_link_path)
        for name, p_opt in MARKERS.items():
            pw = (T_opt @ np.append(p_opt, 1.0))[:3]
            UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/CameraCheck/{name}")).GetOrderedXformOps()[0].Set(Gf.Vec3d(*pw))
        rgb, seg, id2l = render(sensor)
        L("")
        L("[1] intrinsics 확인: 카메라 좌표계의 알려진 점 → 렌더링된 구의 픽셀 중심 vs OpenCV 투영 (기준 < 1 px)")
        worst = 0.0
        for name, p_opt in MARKERS.items():
            m = label_mask(seg, id2l, name)
            if m.sum() < 5:
                raise RuntimeError(f"{name} 가 영상에 보이지 않음 (픽셀 {m.sum()})")
            vs, us = np.nonzero(m)
            u_r, v_r = us.mean(), vs.mean()   # OpenCV 규약: 픽셀 (i, j) 의 중심 좌표가 (i, j)
            u_p, v_p = K[0, 0] * p_opt[0] / p_opt[2] + K[0, 2], K[1, 1] * p_opt[1] / p_opt[2] + K[1, 2]
            e = math.hypot(u_r - u_p, v_r - v_p)
            worst = max(worst, e)
            L(f"  {name} 카메라 좌표 {p_opt}: 렌더링 ({u_r:.2f}, {v_r:.2f}) / 투영 ({u_p:.2f}, {v_p:.2f}) → 차이 {e:.2f} px")
        L(f"  => {'PASS' if worst < 1.0 else 'FAIL'} (최대 {worst:.2f} px)")
        Image.fromarray(rgb).save(os.path.join(out_dir, "intrinsics_markers.png"))
        for name in MARKERS:
            UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/CameraCheck/{name}")).GetOrderedXformOps()[0].Set(Gf.Vec3d(0, 0, -5))

        # ── 2. tilt 비교 ──
        views = []   # (이름, 팔 자세, 그리퍼, 박스 있음)
        for pose_name, pose in (("home", cfg["home_pose"]), ("grasp", GRASP_POSE)):
            for grip in ("open", "closed"):
                views.append((f"{pose_name}_{grip}", pose, grip, pose_name == "grasp"))
        results = {}
        for vname, pose, grip, with_box in views:
            drive.reset_pose(pose, 0.0)
            set_box_pose(stage, box, [2.0, 0.0, 0.5], [1.0, 0, 0, 0])
            for _ in range(20):
                simulation_app.update()
            if with_box:
                bp, bq = (x.numpy()[0] for x in base.get_world_poses())
                c = bp + cg.quat_to_R(bq) @ np.array([0, 0, BOX_CENTER_Z])
                set_box_pose(stage, box, c, bq)
            if grip == "closed":
                drive.set_gripper_goal(GRIP_CLOSED)
                n = int((GRIP_CLOSED / drive.gripper.velocity + 0.5) / ts.PHYSICS_DT)
                for _ in range(n):
                    simulation_app.update()
            drive.check()
            q_grip = math.degrees(float(robot.get_dof_positions().numpy()[0][drive.grip_i]))
            bp, bq = (x.numpy()[0] for x in base.get_world_poses())
            tcp = bp + cg.quat_to_R(bq) @ np.array([0, 0, TCP_Z])
            for tilt in args.tilts:
                ts.set_wrist_camera_tilt(stage, cam_path, math.radians(tilt))
                rgb, seg, id2l = render(sensor)
                T_opt = optical_T(stage, cam_link, cam_path, cam_link_path)
                uv = project(K, T_opt, tcp)
                frac = {n: 100.0 * label_mask(seg, id2l, n).mean() for n in ("finger_r", "finger_l", "box")}
                Image.fromarray(rgb).save(os.path.join(out_dir, f"tilt{tilt:g}_{vname}.png"))
                results[(tilt, vname)] = dict(frac=frac, uv=uv, q_grip=q_grip, rgb=rgb)
        L("")
        L("[2] cam_tilt 비교 (Camera prim 자세만 바꿈): 화면에서 차지하는 비율 % (오른쪽 손가락 / 왼쪽 손가락 / 박스), TCP 영상 좌표 (u, v)")
        L(f"  영상 {w}x{h}. TCP 가 화면 밖이면 '밖'. 저장: {out_dir}")
        for vname, *_ in views:
            L(f"  --- {vname} (그리퍼 {results[(args.tilts[0], vname)]['q_grip']:.1f}°)")
            for tilt in args.tilts:
                r = results[(tilt, vname)]
                f = r["frac"]
                uv = r["uv"]
                inside = uv is not None and 0 <= uv[0] < w and 0 <= uv[1] < h
                uv_s = f"({uv[0]:.0f}, {uv[1]:.0f})" if uv is not None else "뒤"
                L(f"    tilt {tilt:4.1f}°: {f['finger_r']:5.1f} / {f['finger_l']:5.1f} / {f['box']:5.1f} %,  TCP {uv_s}"
                  f"{'' if inside else ' 밖'}")
        # tilt 별로 네 장면을 한 장에
        for tilt in args.tilts:
            imgs = [results[(tilt, v[0])]["rgb"] for v in views]
            top, bottom = np.concatenate(imgs[:2], axis=1), np.concatenate(imgs[2:], axis=1)
            Image.fromarray(np.concatenate([top, bottom], axis=0)).save(os.path.join(out_dir, f"tilt{tilt:g}_all.png"))
        L("  tilt 별 모음: tilt<값>_all.png (왼쪽 위 home_open, 오른쪽 위 home_closed, 왼쪽 아래 grasp_open, 오른쪽 아래 grasp_closed)")
        if args.hold and not args.headless:
            hold_view(stage, cam_path)
    finally:
        drive.stop()
        app_utils.stop()
        simulation_app.update()

    path = os.path.join(out_dir, "report.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"report: {path}", flush=True)


if __name__ == "__main__":
    code = 0
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        code = 1
    finally:
        simulation_app.close(exit_code=code)
    sys.exit(code)
