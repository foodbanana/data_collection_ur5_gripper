#!/usr/bin/env python3
# =============================================================
# check_drone.py  (docs/PLAN.md 2-1)
#
# 드론 에셋 + 씬 레이어 덮어쓰기(drone.add_drone) 점검. 드론 설정·에셋을 바꿀 때마다 실행한다.
#   씬: physics scene(PhysX, 1/120 s) + ground plane + 드론 (로봇 없음). 드론 USD 원본은 수정하지 않는다.
#   1. 질량: PhysX 강체 질량 합계 = 설정 mass (link_masses 가 있으면 링크별로도)
#   2. 무게중심: 전체 무게중심이 잡는 곳(grasp.center_xy)에서 수평 거리 < 1 mm (body 링크 좌표계 = 드론 좌표계)
#   3. 재질·잡는 폭: 모든 충돌 형상에 물체 재질. 잡는 곳의 손가락 폭 slab 안 body 충돌 형상 폭이
#      그리퍼 완전 열림 − 여유 이하 (실제 충돌 형상을 잘라서 잼)
#   4. 낙하: DROP_Z 에서 떨어뜨려 DROP_SEC 동안 NaN·폭주 없음, 놓인 기울기 < rest_tilt_max_deg (설정, 기본 1°), 멈춤.
#      관절을 고정했으면 DOF 0 개, 몸체 기준 링크 상대 자세 변화 < 0.05°·0.1 mm.
#      설정에 legs 가 있으면(간이 드론) 다리로 서 있음 (몸체가 바닥에 안 닿음)
#
# 실행:
#   ~/isaacsim/python.sh ~/data_collection_ur5_gripper/isaacsim/scripts/check_drone.py --headless
#   GUI 로 보기: --headless 빼고 --hold (점검 후 드론을 공중 DROP_Z 에 멈춘 채로 보여 줌. Play 를 누르면 낙하, 창을 닫으면 종료)
#
# 결과: 항목별 PASS/FAIL 을 콘솔과 isaacsim/reports/check_drone_<날짜시간>.txt 에 기록. 하나라도 FAIL 이면 종료 코드 1.
# =============================================================

import argparse
import datetime
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scene as ts  # noqa: E402

parser = argparse.ArgumentParser(description="드론 에셋 점검 (PLAN 2-1)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--drone-config", default=None, help="드론 설정 yaml (기본 config/drone_iris.yaml)")
parser.add_argument("--config", default=ts.DEFAULT_CONFIG, help="물체 재질(contact.object_material)을 읽을 yaml")
parser.add_argument("--report-dir", default=ts.REPORT_DIR)
parser.add_argument("--hold", action="store_true", help="GUI: 점검 후 창을 닫을 때까지 드론을 보여 줌")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import numpy as np  # noqa: E402
from isaacsim.core.experimental.objects import DomeLight, GroundPlane  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

import collision_geom as cg  # noqa: E402
import drone as dr  # noqa: E402

N_CHECKS = 4
DROP_Z = 0.5          # m, 몸체 원점 높이
DROP_SEC = 3.0
MASS_TOL = 1e-4       # kg
COM_TOL = 1e-3        # m, 무게중심–잡는 곳 수평 거리
# 그리퍼 완전 열림 때 손가락 안쪽 면 사이 [m]: TCP 좌표계 충돌 형상 r2·l2 안쪽 끝 ±53.5 mm (2026-10-01 측정, PLAN 2-1)
GRIPPER_OPEN_GAP = 0.107
GRIP_MARGIN = 0.010   # m, 한쪽 여유 (접근 오차)
TILT_TOL = 1.0        # degree, 설정 rest_tilt_max_deg 가 없을 때
LOCK_TOL = 0.05       # degree, 몸체 기준 링크 상대 자세 변화
LOCK_TOL_MM = 0.1     # mm
STILL_TOL = 0.005     # m/s, 낙하 끝 속도
REST_TOL = 1e-3       # m, 다리로 섰을 때 몸체 중심 높이 오차
BLOWUP_VEL = 50.0     # m/s 또는 rad/s


class Report:
    def __init__(self):
        self.lines, self.results = [], []

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


def whole_com(links):
    """RigidPrim(여러 강체) → (전체 무게중심 world, 질량 합)."""
    m = links.get_masses().numpy().reshape(-1)
    pos, quat = (x.numpy() for x in links.get_world_poses())
    com_local = links.get_coms()[0].numpy().reshape(-1, 3)
    w = np.array([p + cg.quat_to_R(q) @ c for p, q, c in zip(pos, quat, com_local)])
    return (m[:, None] * w).sum(axis=0) / m.sum(), float(m.sum())


def rel_poses(links, k):
    """각 강체의 강체 k 기준 (R, t)."""
    pos, quat = (x.numpy() for x in links.get_world_poses())
    Rb, tb = cg.quat_to_R(quat[k]), pos[k]
    return [(Rb.T @ cg.quat_to_R(q), Rb.T @ (p - tb)) for p, q in zip(pos, quat)]


def main():
    cfg = dr.load_drone_config(args.drone_config or dr.DEFAULT_DRONE_CONFIG)
    config = ts.load_config(args.config)
    R.log(f"드론 설정: {cfg['path']}")
    R.log(f"드론 USD: {cfg['usd_abs']} (scale {cfg['scale']})")
    R.log(f"설정 질량 {cfg['mass']} kg, 고정 관절 {len(cfg['lock_joints'])} 개")

    stage_utils.create_new_stage()
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0, kilograms_per_unit=1.0)
    stage = stage_utils.get_current_stage(backend="usd")
    stage_utils.define_prim("/World", "Xform")
    if not SimulationManager.switch_physics_engine("physx"):
        raise RuntimeError("PhysX 로 전환하지 못했습니다")
    SimulationManager.setup_simulation(dt=ts.PHYSICS_DT)
    GroundPlane("/World/GroundPlane")
    if not args.headless:
        DomeLight("/World/DomeLight").set_intensities(1000)   # 조명이 없으면 뷰포트가 비어 보임
    ts.define_physics_material(stage, ts.OBJECT_MATERIAL_PATH, config["contact"]["object_material"])
    info = dr.add_drone(stage, cfg, position=(0.0, 0.0, DROP_Z))
    path = info["path"]

    # 3. 재질·잡는 폭 (USD)
    R.section(3, "재질·잡는 폭")
    why3 = []
    cols = dr.collision_prims(stage, path)
    wrong = [str(p.GetPath()) for p in cols if ts.bound_physics_material(p) != ts.OBJECT_MATERIAL_PATH]
    R.log(f"  충돌 형상 {len(cols)} 개, 물체 재질({ts.OBJECT_MATERIAL_PATH}) 안 걸린 것 {len(wrong)} 개")
    if wrong:
        why3.append(f"재질 없음: {wrong}")
    width, (lo, hi) = dr.grasp_width(stage, cfg, path)
    limit = GRIPPER_OPEN_GAP - 2 * GRIP_MARGIN
    R.log(f"  잡는 곳 폭 {width * 1000:.1f} mm (잡는 방향 {lo * 1000:+.1f} ~ {hi * 1000:+.1f} mm, 손가락 폭 slab "
          f"±{dr.FINGER_HALF_WIDTH * 1000:.0f} mm). 그리퍼 열림 {GRIPPER_OPEN_GAP * 1000:.0f} mm − 여유 2 × "
          f"{GRIP_MARGIN * 1000:.0f} = {limit * 1000:.0f} mm 이하여야 함")
    if width > limit:
        why3.append(f"잡는 곳 폭 {width * 1000:.1f} mm > {limit * 1000:.0f} mm")
    R.result(3, "재질·잡는 폭", not why3, "; ".join(why3))
    Rd, td = dr.drone_frame(stage, path)   # 재생 전 드론 좌표계
    Rb0, tb0 = dr.drone_frame(stage, info["body_path"])
    if not (np.allclose(Rb0, Rd, atol=1e-6) and np.allclose(tb0, td, atol=1e-6)):
        raise RuntimeError("body 링크 좌표계가 드론 좌표계와 다름 (잡는 곳·무게중심 좌표 기준이 어긋남)")

    app_utils.play()
    simulation_app.update()
    names = info["bodies"]
    links = RigidPrim(names)
    k_body = names.index(info["body_path"])

    # 1. 질량
    R.section(1, "질량 (PhysX)")
    why1 = []
    masses = links.get_masses().numpy().reshape(-1)
    for n, m in zip(names, masses):
        rel = n[len(path) + 1:]
        e = cfg["link_masses"].get(rel) if cfg["link_masses"] else None
        R.log(f"  {rel or '(최상위)':10s} {m:.5f} kg" + (f"  (설정 {e})" if e is not None else ""))
        if e is not None and abs(m - e) > MASS_TOL:
            why1.append(f"{rel} {m:.5f} != {e}")
    R.log(f"  합계 {masses.sum():.5f} kg (설정 {cfg['mass']})")
    if abs(masses.sum() - cfg["mass"]) > MASS_TOL:
        why1.append(f"합계 {masses.sum():.5f} != {cfg['mass']}")
    inert = links.get_inertias().numpy().reshape(-1, 3, 3)[k_body]
    R.log(f"  body 관성 대각 (kg·m²) {np.round(np.diag(inert), 6).tolist()}")
    R.result(1, "질량", not why1, "; ".join(why1))

    # 2. 무게중심 (드론 좌표계)
    R.section(2, "무게중심 vs 잡는 곳")
    # body 링크 좌표계 (같은 순간의 PhysX 값끼리. 재생 직후에도 이미 떨어지는 중이라 USD 초기 위치와 섞지 않음)
    #   body 링크는 드론 최상위 prim 과 같은 자세·위치(간이 드론은 같은 prim, Iris 는 local 단위 변환)라고 보고 확인한다
    com_w, _ = whole_com(links)
    pb, qb = (x.numpy()[k_body] for x in links.get_world_poses())
    com = cg.quat_to_R(qb).T @ (com_w - pb)
    cx, cy = (float(v) for v in cfg["grasp"]["center_xy"])
    horiz = float(np.hypot(com[0] - cx, com[1] - cy))
    R.log(f"  전체 무게중심 (드론 좌표계, mm): {np.round(com * 1000, 2).tolist()}, 잡는 곳 ({cx * 1000:.0f}, {cy * 1000:.0f}) 에서 "
          f"수평 {horiz * 1000:.3f} mm")
    R.result(2, "무게중심", horiz <= COM_TOL, f"{horiz * 1000:.2f} mm > {COM_TOL * 1000:.0f}" if horiz > COM_TOL else "")

    # 4. 낙하
    R.section(4, f"낙하 ({DROP_Z} m, {DROP_SEC} s)")
    why4 = []
    if info["articulation"]:
        from isaacsim.core.experimental.prims import Articulation

        art = Articulation(path)
        R.log(f"  articulation DOF {art.num_dofs} 개" + (" (관절을 모두 고정하면 0)" if cfg["lock_joints"] else ""))
        if cfg["lock_joints"] and art.num_dofs != 0:
            why4.append(f"DOF {art.num_dofs} 개 남음 {list(art.dof_names)}")
    rel0 = rel_poses(links, k_body)
    lock_deg, lock_mm = 0.0, 0.0
    for i in range(int(round(DROP_SEC / ts.PHYSICS_DT))):
        simulation_app.update()
        lin, ang = (x.numpy() for x in links.get_velocities())
        if not (np.all(np.isfinite(lin)) and np.all(np.isfinite(ang))):
            why4.append(f"NaN (t = {(i + 1) * ts.PHYSICS_DT:.3f} s)")
            break
        if np.abs(lin).max() > BLOWUP_VEL or np.abs(ang).max() > BLOWUP_VEL:
            why4.append(f"폭주 (t = {(i + 1) * ts.PHYSICS_DT:.3f} s)")
            break
        for (Ra, ta), (Rb_, tb_) in zip(rel0, rel_poses(links, k_body)):
            lock_deg = max(lock_deg, math.degrees(math.acos(np.clip((np.trace(Ra.T @ Rb_) - 1) / 2, -1, 1))))
            lock_mm = max(lock_mm, float(np.linalg.norm(tb_ - ta)) * 1000)
    pos, quat = (x.numpy()[k_body] for x in links.get_world_poses())
    tilt = math.degrees(math.acos(np.clip(cg.quat_to_R(quat)[2, 2], -1, 1)))
    speed = float(np.linalg.norm(links.get_velocities()[0].numpy()[k_body]))
    R.log(f"  최종 body 위치 (m) {np.round(pos, 4).tolist()}, 기울기 {tilt:.3f}°, 속도 {speed * 1000:.3f} mm/s")
    tilt_max = float(cfg.get("rest_tilt_max_deg", TILT_TOL))
    if tilt > tilt_max:
        why4.append(f"기울기 {tilt:.2f}° > {tilt_max}")
    if speed > STILL_TOL:
        why4.append(f"멈추지 않음 ({speed * 1000:.1f} mm/s)")
    if len(names) > 1:
        R.log(f"  body 기준 링크 상대 자세 최대 변화 {lock_deg:.4f}°, {lock_mm:.4f} mm (낙하·충돌 포함)")
        if cfg["lock_joints"] and (lock_deg > LOCK_TOL or lock_mm > LOCK_TOL_MM):
            why4.append(f"링크 상대 자세 변화 {lock_deg:.3f}°, {lock_mm:.3f} mm > {LOCK_TOL}°, {LOCK_TOL_MM} mm")
    if "legs" in cfg:
        H = cfg["body"]["size"][2]
        rest_z = H / 2 + cfg["legs"]["clearance"]
        R.log(f"  몸체 중심 높이 {pos[2] * 1000:.2f} mm (다리로 서면 {rest_z * 1000:.1f} mm)")
        if abs(pos[2] - rest_z) > REST_TOL:
            why4.append(f"몸체 중심 높이 {pos[2] * 1000:.1f} mm != {rest_z * 1000:.1f} (다리로 서 있지 않음)")
    R.result(4, "낙하", not why4, "; ".join(why4))
    app_utils.stop()   # 재생 전 상태(공중 DROP_Z)로 돌아감
    if args.hold and not args.headless:
        hold_view()


def hold_view():
    from isaacsim.core.rendering_manager import ViewportManager

    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[0.45, -0.45, DROP_Z + 0.25], target=[0.0, 0.0, DROP_Z])
    print(f"[hold] 드론이 공중 {DROP_Z} m 에 멈춰 있음. Play 를 누르면 낙하. 창을 닫으면 종료", flush=True)
    while simulation_app.is_running():
        simulation_app.update()


def write_report():
    R.log("")
    R.log("=" * 50)
    for no, title, tag in sorted(R.results):
        R.log(f"  {no}. {title:16s} {tag}")
    n_pass = sum(t == "PASS" for _, _, t in R.results)
    R.log(f"  {n_pass}/{N_CHECKS} PASS")
    os.makedirs(args.report_dir, exist_ok=True)
    path = os.path.join(args.report_dir, f"check_drone_{datetime.datetime.now():%Y%m%d_%H%M%S}.txt")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(R.lines) + "\n")
    print(f"report: {path}", flush=True)
    return n_pass == N_CHECKS and len(R.results) == N_CHECKS


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
        simulation_app.close(exit_code=0 if ok else 1)
    sys.exit(0 if ok else 1)
