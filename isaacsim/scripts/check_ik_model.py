#!/usr/bin/env python3
# =============================================================
# check_ik_model.py  (docs/PLAN.md 5-B B-1)
#
# 텔레옵 노드의 URDF 기구학 (teleop/ik_model.py) 이 sim 로봇과 같은지 검사한다.
#   씬 = test_scene.build_test_scene() (fixed base, 바닥 없음). 관절을 여러 자세로 순간이동 (RobotDrive.reset_pose) 한 뒤
#   측정 관절각의 FK (URDF) 와 sim 의 그리퍼 base 링크 (rh_p12_rn_base, 강체) 자세를 로봇 base (최상위 prim) 기준으로 비교.
#   같은 물리 스텝의 관절각과 링크 자세를 비교하므로 중력 처짐·자기 충돌로 팔이 밀려도 판정에 영향이 없다
#   TCP (rh_p12_rn_tcp) 는 물리 없는 Xform 이라, 그리퍼 base 에서의 고정 위치를 USD 와 URDF 로 따로 비교한다
#
#   1. TCP 고정 위치: USD 의 rh_p12_rn_base → rh_p12_rn_tcp vs URDF (재생 전, 0.1 mm·0.01° 이내)
#   2. 자세 N 개 (홈 근처 절반, 관절 범위 넓게 절반): 그리퍼 base 위치 차이 < 0.5 mm, 자세 차이 < 0.05°
#   3. 홈 자세 TCP 위치 (base 기준) 출력
#
# 실행:
#   ~/isaacsim/python.sh isaacsim/scripts/check_ik_model.py --headless
#   --urdf : 빌드한 URDF (기본 isaacsim/ur5_rh_p12_description/out/ur5_rh_p12_d435i.urdf, sim USD 를 만든 것과 같아야 함)
#   --n 200 --seed 0
# 결과: isaacsim/reports/check_ik_model_<날짜시간>.txt. 하나라도 FAIL 이면 종료 코드 1
# =============================================================

import argparse
import datetime
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_scene as ts  # noqa: E402

REPO = os.path.dirname(ts.SIM_ROOT)
sys.path.insert(1, os.path.join(REPO, "teleop"))
import ik_model as km  # noqa: E402  (numpy 만 씀)

parser = argparse.ArgumentParser(description="URDF 기구학 (teleop/ik_model.py) vs sim (PLAN 5-B B-1)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--urdf", default=km.DEFAULT_URDF)
parser.add_argument("--n", type=int, default=200, help="비교할 자세 수")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--report-dir", default=ts.REPORT_DIR)
args, _ = parser.parse_known_args()

POS_TOL = 0.5e-3          # [m] 그리퍼 base 위치 차이
ROT_TOL = 0.05            # [°]
TCP_POS_TOL = 0.1e-3      # [m] USD vs URDF TCP 고정 위치
TCP_ROT_TOL = 0.01        # [°]
GRIPPER_BASE = "rh_p12_rn_base"

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import numpy as np  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

import collision_geom as cg  # noqa: E402
from robot_drive import RobotDrive  # noqa: E402

LINES = []


def log(s=""):
    print(s, flush=True)
    LINES.append(s)


def usd_world(stage, path):
    """USD 저작 자세 (재생 전) → 4x4 열벡터 규약."""
    M = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    T = np.eye(4)
    T[:3, :3] = np.array(M.ExtractRotationMatrix()).T          # Gf 는 행벡터 규약
    T[:3, 3] = np.array(M.ExtractTranslation())
    return T


def rot_deg(Ra, Rb):
    return math.degrees(np.linalg.norm(km.R_to_rotvec(Ra @ Rb.T)))


def main():
    log(f"check_ik_model  {datetime.datetime.now().isoformat(timespec='seconds')}")
    log(f"URDF   : {args.urdf}")
    log(f"USD    : {ts.DEFAULT_USD}")
    arm_tcp = km.ArmModel(args.urdf, tip=km.TIP_LINK, arm_joints=ts.ARM_JOINTS)
    arm_gb = km.ArmModel(args.urdf, tip=GRIPPER_BASE, arm_joints=ts.ARM_JOINTS)
    if tuple(ts.ARM_JOINTS) != km.ARM_JOINTS:
        raise RuntimeError(f"관절 순서가 다름: sim {ts.ARM_JOINTS} vs ik_model {km.ARM_JOINTS}")

    config = ts.load_config()
    stage, robot, info = ts.build_test_scene(ts.DEFAULT_USD, ts.PHYSICS_VARIANT, base="fixed", config=config,
                                             ground=False, light=not args.headless)
    simulation_app.update()
    results = []

    # ── 1. TCP 고정 위치 (재생 전 USD 저작 값) ──
    gb_path, tcp_path = ts.find_link_path(stage, GRIPPER_BASE), ts.find_link_path(stage, ts.TCP_FRAME)
    T_rel_usd = np.linalg.inv(usd_world(stage, gb_path)) @ usd_world(stage, tcp_path)
    T_rel_urdf = np.linalg.inv(arm_gb.fk(np.zeros(6))) @ arm_tcp.fk(np.zeros(6))
    dp = np.linalg.norm(T_rel_usd[:3, 3] - T_rel_urdf[:3, 3])
    dr = rot_deg(T_rel_usd[:3, :3], T_rel_urdf[:3, :3])
    ok1 = dp < TCP_POS_TOL and dr < TCP_ROT_TOL
    log(f"\n[1] TCP 고정 위치 ({GRIPPER_BASE} → {ts.TCP_FRAME}): USD {np.round(T_rel_usd[:3, 3], 5).tolist()}"
        f" vs URDF {np.round(T_rel_urdf[:3, 3], 5).tolist()}, 차이 {dp * 1000:.3f} mm, {dr:.4f}° → {'PASS' if ok1 else 'FAIL'}")
    results.append(ok1)

    # ── 재생, drive ──
    app_utils.play()
    simulation_app.update()
    problems, _ = ts.verify_articulation(stage, robot, info, config)
    if problems:
        raise RuntimeError(f"articulation 확인 실패: {problems}")
    drive = RobotDrive(robot, config)
    drive.apply()
    drive.start()
    gb = RigidPrim(gb_path)
    root_T = usd_world(stage, ts.ROBOT_PATH)                 # 로봇 base (최상위 prim, 움직이지 않음)
    arm_i = np.asarray(drive.arm_i)

    # ── 2. 자세 N 개 ──
    rng = np.random.default_rng(args.seed)
    lo, hi = arm_gb.limits
    home = np.array(km.HOME)
    poses = [home]
    for k in range(args.n - 1):
        if k % 2 == 0:
            poses.append(home + rng.uniform(-0.5, 0.5, 6))                       # 홈 근처
        else:
            poses.append(np.clip(rng.uniform(-math.pi, math.pi, 6), lo + 0.1, hi - 0.1))   # 넓게
    errs_p, errs_r, worst = [], [], None
    for q_cmd in poses:
        drive.reset_pose(q_cmd)
        simulation_app.update()
        q = robot.get_dof_positions().numpy()[0][arm_i].astype(float)
        p, qw = (x.numpy().reshape(-1) for x in gb.get_world_poses())
        T_sim = np.linalg.inv(root_T) @ np.block([[cg.quat_to_R(qw), p.reshape(3, 1)], [np.zeros((1, 3)), np.ones((1, 1))]])
        T_fk = arm_gb.fk(q)
        ep = float(np.linalg.norm(T_sim[:3, 3] - T_fk[:3, 3]))
        er = rot_deg(T_sim[:3, :3], T_fk[:3, :3])
        errs_p.append(ep)
        errs_r.append(er)
        if worst is None or ep > worst[0]:
            worst = (ep, er, q.tolist(), float(np.max(np.abs(q - q_cmd))))
    errs_p, errs_r = np.array(errs_p), np.array(errs_r)
    ok2 = bool(errs_p.max() < POS_TOL and errs_r.max() < ROT_TOL)
    log(f"\n[2] 그리퍼 base 자세: sim (측정 관절각의 실제 링크) vs URDF FK (같은 관절각), {len(poses)} 자세")
    log(f"    위치 차이 [mm] 중앙값 {np.median(errs_p) * 1000:.4f}, 최대 {errs_p.max() * 1000:.4f} (기준 {POS_TOL * 1000:g})")
    log(f"    자세 차이 [°]  중앙값 {np.median(errs_r):.5f}, 최대 {errs_r.max():.5f} (기준 {ROT_TOL:g})")
    log(f"    최대 위치 차이 자세: q {np.round(worst[2], 4).tolist()} (순간이동 명령과 측정 차이 최대 {worst[3]:.4f} rad)")
    log(f"    → {'PASS' if ok2 else 'FAIL'}")
    results.append(ok2)

    # ── 3. 홈 자세 TCP ──
    drive.reset_pose(home)
    simulation_app.update()
    q = robot.get_dof_positions().numpy()[0][arm_i].astype(float)
    T_tcp = arm_tcp.fk(q)
    log(f"\n[3] 홈 자세 TCP (로봇 base 기준, URDF FK): {np.round(T_tcp[:3, 3], 4).tolist()} m,"
        f" world {np.round((root_T @ T_tcp)[:3, 3], 4).tolist()} m, 공구 축 {np.round(T_tcp[:3, 2], 3).tolist()}")

    drive.stop()
    app_utils.stop()
    simulation_app.update()
    return all(results), results


def write_report(ok, results):
    log(f"\n결과: {sum(results)}/{len(results)} PASS" + ("" if ok else "  ← FAIL 또는 에러"))
    os.makedirs(args.report_dir, exist_ok=True)
    path = os.path.join(args.report_dir, f"check_ik_model_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(LINES) + "\n")
    print(f"리포트: {path}", flush=True)


if __name__ == "__main__":
    ok, results = False, []
    try:
        ok, results = main()
    except Exception as e:  # 중간에 멈춰도 리포트는 남긴다
        import traceback
        log(f"\n[ERROR] {type(e).__name__}: {e}")
        log(traceback.format_exc())
        ok = False
    finally:
        write_report(ok, results)
        # close() 가 프로세스를 끝내므로 종료 코드는 여기서 넘긴다 (check_articulation.py 와 같음. 뒤의 sys.exit 는 실행되지 않음)
        simulation_app.close(exit_code=0 if ok else 1)
    sys.exit(0 if ok else 1)
