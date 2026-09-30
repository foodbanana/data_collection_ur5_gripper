#!/usr/bin/env python3
# =============================================================
# check_articulation.py  (docs/PLAN.md 1-4)
#
# 로봇 USD 구조 회귀 검사. 로봇 USD 를 재import 할 때마다 실행한다.
#   테스트 씬 = physics scene + ground plane + 로봇 USD reference (root_joint 유지, 추가 FixedJoint 없음)
#   로봇 USD 원본은 수정하지 않는다. 검사용 drive 값은 메모리 위 stage 에만 적는다 (저장하지 않음).
#
# 실행:
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_articulation.py --headless
#
# 결과: 항목별 PASS/FAIL 을 콘솔과 isaacsim/reports/check_articulation_<날짜시간>.txt 에 기록.
#       하나라도 FAIL 이면 종료 코드 1.
#
# 단위: USD joint 속성(한계, drive target, stiffness)은 degree 기준, tensor API(DOF 위치·한계)는 rad.
# =============================================================

import argparse
import datetime
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)  # ~/data_collection_ur5_gripper/isaacsim
DEFAULT_USD = os.path.join(SIM_ROOT, "assets/robots/ur5_rh_p12_d435i/ur5_rh_p12_d435i.usda")
DEFAULT_REPORT_DIR = os.path.join(SIM_ROOT, "reports")

parser = argparse.ArgumentParser(description="로봇 USD 구조 검증 (PLAN 1-4)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--usd", default=DEFAULT_USD, help="로봇 USD 진입 파일")
parser.add_argument("--physics-variant", default="physx",
                    help="로봇 USD 의 'Physics' variant 선택 (physx | physics | mujoco | none). 기본 physx")
parser.add_argument("--report-dir", default=DEFAULT_REPORT_DIR)
args, _ = parser.parse_known_args()

if not os.path.isfile(args.usd):
    sys.exit(f"[ERROR] 로봇 USD 가 없습니다: {args.usd}")

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import numpy as np  # noqa: E402
from isaacsim.core.experimental.objects import GroundPlane  # noqa: E402
from isaacsim.core.experimental.prims import Articulation, XformPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

# ── 기대값 ──
ROBOT_PATH = "/World/ur5_rh_p12_d435i"
ROBOT_BASE_Z = 0.762  # m, 테이블 상판 높이. 0 이면 관절 0 자세에서 팔이 ground plane 에 걸친다 (1-5 에서도 같은 씬)
ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]  # 실물 UR 드라이버와 같은 이름·순서
GRIPPER_DRIVE = "rh_r1_joint"
GRIPPER_MIMIC = ["rh_r2", "rh_l1", "rh_l2"]
GRIPPER_JOINTS = [GRIPPER_DRIVE] + GRIPPER_MIMIC
GRIPPER_UPPER = 1.1351  # rad
LIMIT_TOL = 1e-3  # rad
FRAMES = ["rh_p12_rn_tcp", "wrist_camera_color_optical_frame"]
# 기구학 기준값 (URDF FK, 참고용 비교: 어긋나면 WARN)
EXPECTED_REL = {
    "rh_p12_rn_tcp": ("tool0", (0.0, 0.0, 0.125)),
    "wrist_camera_color_optical_frame": ("wrist_mount", (0.0325, 0.0575, 0.02575)),
}
REL_TOL = 1e-3  # m

PHYSICS_DT = 1.0 / 120.0
SETTLE_SEC = 3.0            # 7번
GRIP_SEC = 2.0              # 8번
GRIP_TARGET_DEG = 30.0
GRIP_STIFFNESS = 1000.0     # USD 단위 (degree 기준), GUI 확인 때와 같은 값
GRIP_DAMPING = 100.0
GRIP_SYM_TOL_DEG = 1.0
BLOWUP_VEL = 100.0          # rad/s, 이보다 크면 폭주로 판정
ROOT_DRIFT_TOL = 1e-3       # m, 고정 베이스 루트 이동 허용치


class Report:
    def __init__(self):
        self.lines = []
        self.results = []  # (번호, 제목, PASS/FAIL)

    def log(self, msg=""):
        print(msg, flush=True)
        self.lines.append(msg)

    def section(self, no, title):
        self.log("")
        self.log(f"[{no}] {title}")

    def result(self, no, title, ok, why=""):
        tag = "PASS" if ok else "FAIL"
        self.results.append((no, title, tag))
        self.log(f"  => {tag}" + (f" ({why})" if why else ""))


R = Report()


def find_prim_by_name(stage, root, name):
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.GetName() == name:
            return prim
    return None


def joint_prims(stage):
    out = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH)):
        if prim.IsA(UsdPhysics.Joint):
            out[prim.GetName()] = prim
    return out


def attr(prim, name):
    a = prim.GetAttribute(name)
    return a.Get() if a and a.IsValid() else None


def rel_targets(prim, name):
    r = prim.GetRelationship(name)
    return [str(t) for t in r.GetTargets()] if r and r.IsValid() else []


def world_pos(cache, prim):
    return np.array(cache.GetLocalToWorldTransform(prim).ExtractTranslation())


def main():
    R.log(f"check_articulation  {datetime.datetime.now().isoformat(timespec='seconds')}")
    R.log(f"robot USD        : {args.usd}")
    R.log(f"Physics variant  : {args.physics_variant}")

    # ── 테스트 씬 ──
    stage_utils.create_new_stage()
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0, kilograms_per_unit=1.0)
    stage = stage_utils.get_current_stage(backend="usd")
    stage_utils.define_prim("/World", "Xform")

    if not SimulationManager.switch_physics_engine("physx"):
        raise RuntimeError("PhysX 로 전환하지 못했습니다")
    engine = SimulationManager.get_active_physics_engine()
    R.log(f"physics engine   : {engine}")
    if engine != "physx":
        raise RuntimeError(f"physics engine 이 physx 가 아닙니다: {engine}")
    SimulationManager.setup_simulation(dt=PHYSICS_DT)
    GroundPlane("/World/GroundPlane")
    stage_utils.add_reference_to_stage(
        usd_path=args.usd, path=ROBOT_PATH, variants=[("Physics", args.physics_variant)])
    # 배치 규칙(1-4): 위치는 최상위 prim Transform 으로만 지정 (root_joint 가 이 위치에 고정)
    XformPrim(ROBOT_PATH, reset_xform_op_properties=True).set_world_poses(positions=[0.0, 0.0, ROBOT_BASE_Z])
    R.log(f"robot base       : {ROBOT_PATH} at z = {ROBOT_BASE_Z} m")
    simulation_app.update()

    joints = joint_prims(stage)
    R.log(f"USD joints ({len(joints)}): {sorted(joints)}")

    # ── 3. drive·mimic 설정 (USD, 원본 값 그대로 먼저 기록) ──
    R.section(3, "drive·mimic 설정 (USD 원본 값)")
    ok3, why3 = True, []
    for name in ARM_JOINTS + GRIPPER_JOINTS:
        p = joints.get(name)
        if p is None:
            ok3 = False
            why3.append(f"{name} 없음")
            continue
        drive = (f"stiffness={attr(p, 'drive:angular:physics:stiffness')} "
                 f"damping={attr(p, 'drive:angular:physics:damping')} "
                 f"maxForce={attr(p, 'drive:angular:physics:maxForce')} "
                 f"target={attr(p, 'drive:angular:physics:targetPosition')}")
        line = f"  {name:22s} {drive}"
        if name in GRIPPER_MIMIC:
            tgt = rel_targets(p, "newton:mimicJoint")
            en, c1, c0 = attr(p, "newton:mimicEnabled"), attr(p, "newton:mimicCoef1"), attr(p, "newton:mimicCoef0")
            line += f"\n  {'':22s} mimic -> {tgt}  enabled={en} coef1(배율)={c1} coef0(오프셋, deg)={c0}"
            if not p.HasAPI("NewtonMimicAPI"):
                ok3 = False
                why3.append(f"{name}: NewtonMimicAPI 없음")
            if len(tgt) != 1 or not tgt[0].endswith(f"/{GRIPPER_DRIVE}"):
                ok3 = False
                why3.append(f"{name}: mimic 대상이 {GRIPPER_DRIVE} 아님")
            if en is False or (c1 is not None and abs(c1 - 1.0) > 1e-6) or (c0 is not None and abs(c0) > 1e-6):
                ok3 = False
                why3.append(f"{name}: mimic 계수 이상")
            if (attr(p, "drive:angular:physics:stiffness") or 0.0) != 0.0:
                ok3 = False
                why3.append(f"{name}: mimic 조인트에 stiffness 가 있음")
        R.log(line)
    R.result(3, "drive·mimic 설정", ok3, "; ".join(why3))

    # ── 4. root_joint ──
    R.section(4, "root_joint")
    mount_path = f"{ROBOT_PATH}/Geometry/robot_mount"
    rj = joints.get("root_joint")
    ok4, why4 = True, []
    if rj is None:
        ok4, why4 = False, ["root_joint 없음"]
    else:
        b0, b1 = rel_targets(rj, "physics:body0"), rel_targets(rj, "physics:body1")
        R.log(f"  body0 = {b0}")
        R.log(f"  body1 = {b1}")
        if b0 != [ROBOT_PATH]:
            ok4 = False
            why4.append(f"body0 != {ROBOT_PATH}")
        if b1 != [mount_path]:
            ok4 = False
            why4.append(f"body1 != {mount_path}")
        top = stage.GetPrimAtPath(ROBOT_PATH)
        if top.HasAPI(UsdPhysics.RigidBodyAPI):
            ok4 = False
            why4.append("최상위 prim 이 강체임")
    fixed_to_mount = [n for n, p in joints.items()
                      if p.IsA(UsdPhysics.FixedJoint) and mount_path in rel_targets(p, "physics:body1")]
    R.log(f"  robot_mount 를 body1 로 가진 FixedJoint: {fixed_to_mount}")
    if len(fixed_to_mount) != 1:
        ok4 = False
        why4.append("robot_mount 고정 조인트가 1개가 아님 (중복 고정)")
    roots = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PATH))
             if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    R.log(f"  ArticulationRootAPI: {roots}")
    if len(roots) != 1:
        ok4 = False
        why4.append(f"ArticulationRootAPI {len(roots)}개")
    R.result(4, "root_joint", ok4, "; ".join(why4))

    # ── 5. 프레임 (재생 전, 모든 관절 0 자세의 authored 좌표) ──
    R.section(5, "프레임 존재와 world 좌표 (관절 0 자세)")
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    ok5, why5 = True, []
    for fname in FRAMES:
        fp = find_prim_by_name(stage, ROBOT_PATH, fname)
        if fp is None:
            ok5 = False
            why5.append(f"{fname} 없음")
            R.log(f"  {fname}: 없음")
            continue
        wp_ = world_pos(cache, fp)
        R.log(f"  {fname}: {fp.GetPath()}")
        R.log(f"    world (m) = {np.round(wp_, 4).tolist()}")
        if not np.all(np.isfinite(wp_)):
            ok5 = False
            why5.append(f"{fname} 좌표 비정상")
        ref_name, expect = EXPECTED_REL[fname]
        rp = find_prim_by_name(stage, ROBOT_PATH, ref_name)
        if rp is not None:
            # USD 는 행벡터 규약: fp 의 ref 좌표계 표현 = M_fp * M_ref^-1
            rel_m = cache.GetLocalToWorldTransform(fp) * cache.GetLocalToWorldTransform(rp).GetInverse()
            rel_local = np.array(rel_m.ExtractTranslation())
            R.log(f"    {ref_name} 기준 (m) = {np.round(rel_local, 4).tolist()}  기대 {list(expect)}")
            if np.max(np.abs(rel_local - np.array(expect))) > REL_TOL:
                R.log(f"    WARN: {ref_name} 기준 위치가 URDF 기준값과 {REL_TOL * 1000:.0f} mm 이상 다름")
    R.result(5, "프레임", ok5, "; ".join(why5))

    # 8번용 임시 drive 는 메모리 stage 에만 적는다 (재생 전에 적어야 PhysX 가 확실히 읽음, target 0 = 열림 유지)
    gp = joints.get(GRIPPER_DRIVE)
    if gp is None:
        raise RuntimeError(f"{GRIPPER_DRIVE} 가 없습니다")
    d = UsdPhysics.DriveAPI.Apply(gp, "angular")
    d.CreateStiffnessAttr().Set(GRIP_STIFFNESS)
    d.CreateDampingAttr().Set(GRIP_DAMPING)
    d.CreateTargetPositionAttr().Set(0.0)

    # ── 재생 ──
    robot = Articulation(ROBOT_PATH)
    app_utils.play()
    simulation_app.update()
    simulation_app.update()

    dof_names = list(robot.dof_names)

    # ── 1. DOF ──
    R.section(1, "DOF 이름·순서·개수")
    R.log(f"  num_dofs = {robot.num_dofs}")
    for i, n in enumerate(dof_names):
        R.log(f"    {i}: {n}")
    missing_arm = [n for n in ARM_JOINTS if n not in dof_names]
    arm_order = [n for n in dof_names if n in ARM_JOINTS]
    mimic_in = [n for n in GRIPPER_MIMIC if n in dof_names]
    R.log(f"  mimic 조인트 중 DOF 로 잡힌 것: {mimic_in} ({len(mimic_in)}/3)")
    ok1, why1 = True, []
    if missing_arm:
        ok1 = False
        why1.append(f"팔 조인트 없음: {missing_arm}")
    elif arm_order != ARM_JOINTS:
        ok1 = False
        why1.append(f"팔 순서 다름: {arm_order}")
    if GRIPPER_DRIVE not in dof_names:
        ok1 = False
        why1.append(f"{GRIPPER_DRIVE} 없음")
    R.result(1, "DOF", ok1, "; ".join(why1))

    # ── 2. 관절 한계 ──
    R.section(2, "관절 한계 (rad)")
    lower, upper = (x.numpy()[0] for x in robot.get_dof_limits())
    expect = {n: (-2 * math.pi, 2 * math.pi) for n in ARM_JOINTS}
    expect["elbow_joint"] = (-math.pi, math.pi)
    for n in GRIPPER_JOINTS:
        expect[n] = (0.0, GRIPPER_UPPER)
    ok2, why2 = True, []
    for n, (lo_e, hi_e) in expect.items():
        if n in dof_names:
            i = dof_names.index(n)
            lo, hi, src = float(lower[i]), float(upper[i]), "tensor"
        elif n in joints:
            lo = math.radians(attr(joints[n], "physics:lowerLimit"))
            hi = math.radians(attr(joints[n], "physics:upperLimit"))
            src = "USD(deg→rad)"
        else:
            ok2 = False
            why2.append(f"{n} 없음")
            continue
        good = abs(lo - lo_e) <= LIMIT_TOL and abs(hi - hi_e) <= LIMIT_TOL
        R.log(f"  {n:22s} [{lo:+.4f}, {hi:+.4f}]  기대 [{lo_e:+.4f}, {hi_e:+.4f}]  {src}  {'ok' if good else 'MISMATCH'}")
        if not good:
            ok2 = False
            why2.append(n)
    R.result(2, "관절 한계", ok2, "다름: " + ", ".join(why2) if why2 else "")

    # 3번 보충: 실행 중 tensor 값 (PhysX 내부 단위)
    stiff, damp = (x.numpy()[0] for x in robot.get_dof_gains())
    maxf = robot.get_dof_max_efforts().numpy()[0]
    R.log("")
    R.log("  [3 보충] tensor API 로 읽은 gain (8번용 rh_r1 임시 drive 적용 후, rad 기준 단위)")
    for i, n in enumerate(dof_names):
        R.log(f"    {n:22s} stiffness={stiff[i]:.4g} damping={damp[i]:.4g} maxEffort={maxf[i]:.4g}")

    # ── 6. 질량·관성 ──
    R.section(6, "링크 질량·관성")
    masses = robot.get_link_masses().numpy()[0]
    inertias = robot.get_link_inertias().numpy()[0].reshape(-1, 3, 3)
    ok6, why6 = True, []
    for n, m, I in zip(robot.link_names, masses, inertias):
        ev = np.linalg.eigvalsh((I + I.T) / 2)
        warn = []
        if not np.isfinite(m) or m <= 0:
            warn.append("질량 0 이하")
        if not np.all(np.isfinite(ev)) or np.min(ev) <= 0:
            warn.append("관성 고유값 0 이하")
        elif ev[0] + ev[1] < ev[2] * (1 - 1e-6):
            warn.append("관성 삼각 부등식 위반")
        R.log(f"  {n:34s} {m:8.4f} kg  I주축={np.round(ev, 7).tolist()}" + (f"  WARN: {', '.join(warn)}" if warn else ""))
        if warn:
            ok6 = False
            why6.append(n)
    R.log(f"  합계 = {float(np.sum(masses)):.4f} kg ({len(masses)} 링크)")
    R.result(6, "질량·관성", ok6, "경고 링크: " + ", ".join(why6) if why6 else "")

    # ── 7. 3초 시뮬레이션 ──
    R.section(7, f"{SETTLE_SEC:.0f}초 시뮬레이션 (NaN·폭주)")
    root0 = robot.get_world_poses()[0].numpy()[0].copy()
    t0 = SimulationManager.get_simulation_time()
    max_vel, bad = 0.0, None
    while SimulationManager.get_simulation_time() - t0 < SETTLE_SEC:
        simulation_app.update()
        q = robot.get_dof_positions().numpy()[0]
        v = robot.get_dof_velocities().numpy()[0]
        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(v))):
            bad = "NaN/inf"
            break
        max_vel = max(max_vel, float(np.max(np.abs(v))))
    sim_t = SimulationManager.get_simulation_time() - t0
    q = robot.get_dof_positions().numpy()[0]
    root1 = robot.get_world_poses()[0].numpy()[0]
    drift = float(np.linalg.norm(root1 - root0))
    R.log(f"  sim time = {sim_t:.3f} s, 최대 |DOF 속도| = {max_vel:.3f} rad/s, 루트 이동 = {drift * 1000:.3f} mm")
    R.log("  최종 DOF 위치 (rad): " + ", ".join(f"{n}={x:+.3f}" for n, x in zip(dof_names, q)))
    ok7, why7 = True, []
    if bad:
        ok7, why7 = False, [bad]
    if max_vel > BLOWUP_VEL:
        ok7 = False
        why7.append(f"속도 {max_vel:.1f} > {BLOWUP_VEL}")
    if drift > ROOT_DRIFT_TOL:
        ok7 = False
        why7.append(f"루트가 {drift * 1000:.1f} mm 움직임 (고정 안 됨)")
    R.result(7, "시뮬레이션 안정성", ok7, "; ".join(why7))

    # ── 8. 그리퍼 mimic 대칭 ──
    R.section(8, f"{GRIPPER_DRIVE} target {GRIP_TARGET_DEG:.0f}° → {GRIP_SEC:.0f}초 후 네 조인트 차 ≤ {GRIP_SYM_TOL_DEG}°")
    ok8, why8 = True, []
    not_dof = [n for n in GRIPPER_JOINTS if n not in dof_names]
    if not_dof:
        ok8, why8 = False, [f"DOF 가 아니라 읽을 수 없음: {not_dof}"]
    else:
        gi = dof_names.index(GRIPPER_DRIVE)
        robot.set_dof_position_targets(math.radians(GRIP_TARGET_DEG), dof_indices=gi)
        t0 = SimulationManager.get_simulation_time()
        while SimulationManager.get_simulation_time() - t0 < GRIP_SEC:
            simulation_app.update()
        q = robot.get_dof_positions().numpy()[0]
        deg = {n: math.degrees(float(q[dof_names.index(n)])) for n in GRIPPER_JOINTS}
        for n, x in deg.items():
            R.log(f"  {n:12s} {x:8.3f}°")
        spread = max(deg.values()) - min(deg.values())
        R.log(f"  최대-최소 = {spread:.3f}°")
        if spread > GRIP_SYM_TOL_DEG:
            ok8 = False
            why8.append(f"차이 {spread:.2f}°")
        if abs(deg[GRIPPER_DRIVE] - GRIP_TARGET_DEG) > 5.0:
            ok8 = False
            why8.append(f"{GRIPPER_DRIVE} 가 목표에 도달 못함 ({deg[GRIPPER_DRIVE]:.1f}°)")
    R.result(8, "그리퍼 mimic 대칭", ok8, "; ".join(why8))

    app_utils.stop()


def write_report():
    R.log("")
    R.log("=" * 50)
    for no, title, tag in sorted(R.results):
        R.log(f"  {no}. {title:20s} {tag}")
    n_pass = sum(t == "PASS" for _, _, t in R.results)
    R.log(f"  {n_pass}/8 PASS")
    os.makedirs(args.report_dir, exist_ok=True)
    path = os.path.join(args.report_dir, f"check_articulation_{datetime.datetime.now():%Y%m%d_%H%M%S}.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(R.lines) + "\n")
    print(f"report: {path}", flush=True)
    return n_pass == 8 and len(R.results) == 8


if __name__ == "__main__":
    ok = False
    try:
        main()
    except Exception as e:  # 중간에 멈춰도 리포트는 남긴다
        import traceback
        R.log("")
        R.log(f"[ERROR] {type(e).__name__}: {e}")
        R.log(traceback.format_exc())
    finally:
        ok = write_report()
        # close() 가 프로세스를 끝내므로 종료 코드는 여기서 넘겨야 한다 (뒤의 sys.exit 는 실행되지 않음)
        simulation_app.close(exit_code=0 if ok else 1)
    sys.exit(0 if ok else 1)
