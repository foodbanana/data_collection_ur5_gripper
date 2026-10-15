#!/usr/bin/env python3
# =============================================================
# wrist_camera_clearance.py — 손목 카메라 장착 후보의 카메라 몸체 ↔ 로봇 링크 여유 거리 (compare_wrist_camera.py 의 2 단계)
#
#   로봇 USD 그대로. 카메라 링크 (wrist_camera_link) 의 충돌 형상에만 후보 변환을 곱해 판정한다 (메모리 계산만).
#   후보 정의는 compare_wrist_camera.py 와 같다 (mount 좌표계 s 옆, d 공구 축에서 거리, b 뒤로, t 기울기 °).
#   자세: 시작 자세 under_drone + 텔레옵 범위 (TCP 기준 base x·y 축 ±10·20·30°, z 축 ±30·60°, 위 15·28 cm, 아래·앞·뒤·좌·우 10 cm)
#         + 잡는 높이 (30 cm 위) + 잡는 높이에서 x·y 축 ±20°. 각 자세를 그리퍼 열림·닫힘으로
#   여유 = 두 convex hull 사이 거리의 근사 (면 방정식으로 잰 값. 밖에서는 실제 거리보다 작거나 같다 = 보수적, 음수 = 파고듦 깊이)
#   제외: 카메라와 관절로 이어진 링크, 마운트 (wrist_mount: 새 CAD 로 바뀔 부품. 따로 출력)
#
# 실행: ~/isaacsim/python.sh isaacsim/scripts/wrist_camera_clearance.py --headless [--cand "T7:-0.0325,0.0575,0,7" ...]
# =============================================================

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "teleop"))

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--cand", nargs="+", default=["C0:0,0.0575,0,0", "T7:-0.0325,0.0575,0,7", "C4:-0.0325,0.0575,0,10"])
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

import collision_geom as cg  # noqa: E402
import ik_model as km  # noqa: E402
import test_scene as ts  # noqa: E402
from robot_drive import RobotDrive  # noqa: E402

CAM_LINK, MOUNT_LINK = "wrist_camera_link", "wrist_mount"
D_NOW = 0.0575


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    T[1:3, 1:3] = [[c, -s], [s, c]]
    return T


def axis_R(ax, deg):
    return km.axis_angle_R(np.eye(3)["xyz".index(ax)], math.radians(deg))


def clearance(sa, Ta, sb, Tb):
    """convex hull 두 개 사이 거리 근사 [m] (음수 = 파고듦)."""
    best = np.inf
    for s1, T1, s2, T2 in ((sa, Ta, sb, Tb), (sb, Tb, sa, Ta)):
        R1, t1 = T1
        p = s1.fast_pts @ R1.T + t1
        eq = cg._eq_world(s2, *T2)
        v = (p @ eq[:, :3].T + eq[:, 3]).max(axis=1)    # 점의 평면 거리 최댓값 (밖 > 0, 안 < 0)
        best = min(best, float(v.min()))
    return best


def main():
    cands = []
    for t in args.cand:
        n, v = t.split(":")
        s, d, b, deg = (float(x) for x in v.split(","))
        cands.append((n, np.eye(4) @ np.array([[1, 0, 0, s], [0, 1, 0, d - D_NOW], [0, 0, 1, -b], [0, 0, 0, 1.0]]) @ Rx(math.radians(deg)), (s, d, b, deg)))

    cfg = ts.load_config()
    sc = yaml.safe_load(open(os.path.join(ts.SIM_ROOT, "config", "scene_drone.yaml"), encoding="utf-8"))
    q0 = np.asarray(sc["init_poses"]["under_drone"], float)
    stage, robot, info = ts.build_test_scene(ts.DEFAULT_USD, ts.PHYSICS_VARIANT, base="fixed", config=cfg, ground=False)
    app_utils.play()
    simulation_app.update()
    drive = RobotDrive(robot, cfg)
    drive.apply()
    drive.start()

    shapes, link_paths = cg.collect(stage)
    names = sorted(shapes)
    if CAM_LINK not in shapes:
        raise RuntimeError(f"{CAM_LINK} 충돌 형상이 없음 (링크: {names})")
    skip = cg.joint_pairs(stage)
    view = RigidPrim([link_paths[n] for n in names])
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    W = lambda p: cg.mat_np(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(p))).T  # noqa: E731
    mic = np.linalg.inv(W(link_paths[CAM_LINK])) @ W(ts.find_link_path(stage, ts.CAMERA_TILT_PIVOT))   # mount in camera link

    # 자세 (URDF IK, base 좌표계)
    model = km.ArmModel()
    T0 = model.fk(q0)

    def solve(p, R, q=q0):
        q = np.array(q, float)
        for _ in range(3000):
            st = km.ik_step(model, q, q, p, R, max_dq=0.02, max_lead=1.0)
            q = st.q
            if st.pos_err < 1e-5 and st.rot_err < 1e-4:
                return q
        return None

    poses = [("start", q0)]
    for ax, degs in (("x", (-30, -20, -10, 10, 20, 30)), ("y", (-30, -20, -10, 10, 20, 30)), ("z", (-60, -30, 30, 60))):
        for dg in degs:
            poses.append((f"rot {ax} {dg:+d}°", solve(T0[:3, 3], axis_R(ax, dg) @ T0[:3, :3])))
    for name, dp in (("up 15", (0, 0, 0.15)), ("up 28", (0, 0, 0.28)), ("down 10", (0, 0, -0.10)), ("fwd 10", (0.10, 0, 0)),
                     ("back 10", (-0.10, 0, 0)), ("left 10", (0, 0.10, 0)), ("right 10", (0, -0.10, 0))):
        poses.append((name, solve(T0[:3, 3] + np.array(dp), T0[:3, :3])))
    pg = T0[:3, 3] + np.array([0, 0, 0.30])
    qg = solve(pg, T0[:3, :3])
    poses.append(("grasp height", qg))
    for ax in "xy":
        for dg in (-20, 20):
            poses.append((f"grasp {ax} {dg:+d}°", solve(pg, axis_R(ax, dg) @ T0[:3, :3], qg)))
    bad = [n for n, q in poses if q is None]
    poses = [(n, q) for n, q in poses if q is not None]

    res = {c[0]: [] for c in cands}       # (자세, 그리퍼, 가장 가까운 링크, 여유, 마운트 여유)
    for pname, q in poses:
        for grip, gq in (("open", 0.0), ("closed", 1.1351)):
            drive.reset_pose(q, gq)
            for _ in range(5):
                simulation_app.update()
            T = cg.link_transforms(view, names)
            Rc, tc = T[CAM_LINK]
            Tc = np.eye(4)
            Tc[:3, :3], Tc[:3, 3] = Rc, tc
            for cname, M, _ in cands:
                Tn = Tc @ mic @ M @ np.linalg.inv(mic)
                Tcand = (Tn[:3, :3], Tn[:3, 3])
                best, who, mount = np.inf, None, np.inf
                for n in names:
                    if n == CAM_LINK or frozenset((n, CAM_LINK)) in skip:
                        continue
                    d = min(clearance(sa, Tcand, sb, T[n]) for sa in shapes[CAM_LINK] for sb in shapes[n])
                    if n == MOUNT_LINK:
                        mount = d
                        continue
                    if d < best:
                        best, who = d, n
                res[cname].append((pname, grip, who, best, mount))

    print("\n=== 카메라 몸체 여유 (convex hull 근사, mm. 음수 = 겹침). 마운트 (wrist_mount) 는 따로", flush=True)
    print(f"자세 {len(poses)} 개 × 그리퍼 2 = {2 * len(poses)} 가지" + (f" (IK 못 푼 자세: {bad})" if bad else ""), flush=True)
    for cname, _, (s, d, b, deg) in cands:
        r = res[cname]
        worst = min(r, key=lambda x: x[3])
        n_neg = sum(1 for x in r if x[3] < 0)
        by_link = {}
        for x in r:
            by_link[x[2]] = min(by_link.get(x[2], np.inf), x[3])
        print(f"\n{cname} (s {s * 1000:+.1f}, d {d * 1000:.1f}, b {b * 1000:.0f} mm, t {deg:g}°): 최소 여유 {worst[3] * 1000:+.1f} mm "
              f"({worst[0]}, {worst[1]}, {worst[2]}), 겹침 {n_neg} / {len(r)}, 마운트와 {min(x[4] for x in r) * 1000:+.1f} mm", flush=True)
        print("  링크별 최소: " + ", ".join(f"{k} {v * 1000:+.1f}" for k, v in sorted(by_link.items(), key=lambda kv: kv[1])), flush=True)
        for x in sorted(r, key=lambda x: x[3])[:5]:
            print(f"    {x[0]:14s} {x[1]:6s} {x[2]:22s} {x[3] * 1000:+.1f} mm", flush=True)
    drive.stop()
    app_utils.stop()


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
